// Standalone link scanner for the TMC2209 UART: which of pins 8/9 is RX, and is
// anything there at all.
//
// Why a separate program rather than another command in main.cpp
// --------------------------------------------------------------
// The silkscreen on the driver module is unreadable, so the two wires have to be
// tried both ways round -- while somebody moves them by hand. That rules out
// AltSoftSerial, which the main firmware uses: on an ATmega328 it is welded to
// Timer1 and therefore to pin 8 as RX (ICP1) and pin 9 as TX (OC1A). It cannot be
// asked to swap them.
//
// SoftwareSerial can take any pair, but two instances cost ~128 bytes of buffers
// and the main firmware has 144 bytes of RAM left. So both orientations are driven
// by the bit-banged UART below instead. That is not a compromise: it makes the two
// directions symmetric *by construction* -- same code, same timing, same sampling
// point -- where two different libraries would have been two different experiments.
//
// Dropping the motor, TMCStepper and AltOSoftSerial also buys back ~150 bytes and
// keeps a round down to tens of milliseconds, which is what makes the thing feel
// live while the wires are being poked.
//
// What one round measures, per orientation
// ----------------------------------------
//   link  DC continuity: drive the transmit pin, read the receive pin. Uses neither
//         a UART nor the chip, so it separates "no copper" from "no answer". This is
//         also the honest form of "do I hear my own echo": the line is single-wire,
//         so hearing yourself *is* the two pins sharing a node.
//   raw   bytes that came back after a hand-built read datagram. 0 = nothing at all,
//         8 = the chip answered.
//   ver   VERSION out of IOIN, 0x21 on a healthy TMC2209 -- the chip's own voice,
//         as opposed to an echo of ours.
//   ifcnt movement of the chip's write counter across one harmless write. The only
//         proof of a *write* that does not involve turning the shaft.

#include <Arduino.h>

static const uint8_t PIN_A = 8;
static const uint8_t PIN_B = 9;

// The board carries two RGB LEDs and one single LED, which happens to be exactly what
// two orientations plus a sign of life need. Talking through them beats talking out
// loud: the state changes several times a second while somebody wiggles a wire, and
// no voice can keep up with that without becoming unbearable.
//
// Active high, same as the main firmware's clearStatusLedsV2().
static const uint8_t LED_A_R = 6;    // the RGB that cycles colour per step normally
static const uint8_t LED_A_G = 5;
static const uint8_t LED_A_B = 3;
static const uint8_t LED_B_R = A3;   // the mode LED
static const uint8_t LED_B_G = A2;
static const uint8_t LED_B_B = A5;
static const uint8_t LED_ALIVE = 11; // power LED, toggled every round

// Motor supply sense, same divider the main firmware uses. A driver with no power
// answers exactly like a driver that is not wired: silence. Worth ruling out before
// blaming the wire.
static const uint8_t POWER_SENSE_PIN = A1;
static const float ADC_INTERNAL_VREF = 1.1f;
static const float POWER_DIVIDER_RATIO = 23.93555f;

// Known VERSION bytes in IOIN. Not used to *accept* a reply -- a datagram whose CRC
// checks out is a reply whatever it says -- only to name the chip afterwards. Assuming
// 0x21 here was a real bug: the board carries a TMC2225, which is a TMC2208 variant and
// answers 0x20, so a perfectly good link would have been reported as "no reply".
static const uint8_t VERSION_TMC220X = 0x20;  // TMC2208 / TMC2224 / TMC2225
static const uint8_t VERSION_TMC2209 = 0x21;

// TMC2208/2225 have no address pins at all -- the slave address is always 0. (On a
// TMC2209 it would be the MS1/MS2 strapping.) The board here carries a TMC2225.
static const uint8_t TMC_ADDRESS = 0b00;
static const uint8_t REG_IFCNT = 0x02;
static const uint8_t REG_IOIN = 0x06;
static const uint8_t REG_TPOWERDOWN = 0x11;
static const uint8_t TMC_WRITE = 0x80;
static const uint8_t TMC_SYNC = 0x05;

// 9600 baud. Low enough that bit-banging is comfortable, and the chip detects the
// rate from the sync nibble of every datagram, so nothing has to agree in advance.
static const uint16_t BIT_US = 104;
// digitalRead/digitalWrite cost a few microseconds on AVR; subtracting them keeps
// the sampling point in the middle of the bit instead of drifting off the end of
// the byte.
static const uint16_t TX_TRIM = 5;
static const uint16_t RX_TRIM = 6;

static uint8_t rxBuf[12];
// What the line carried back while the sync byte was being sent; TMC_SYNC when the
// wire is intact and the timing is right.
static uint8_t lastEcho = 0;

// Once the chip has answered, remember it. The one real reply of the whole session
// lasted a single round -- the green LED blinked for 150 ms while a wire was being
// moved, which no eye could catch and which the log only gave up afterwards. A latch
// turns "it worked for an instant" from something found by archaeology into something
// the board tells you while your hand is still on the wire.
static uint8_t everRepliedVersion[2] = {0, 0};
static uint32_t everRepliedCount[2] = {0, 0};

static uint8_t tmcCrc8(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    uint8_t current = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (current & 0x01)) crc = (uint8_t)((crc << 1) ^ 0x07);
      else crc = (uint8_t)(crc << 1);
      current = (uint8_t)(current >> 1);
    }
  }
  return crc;
}

// Send a byte and read the line back *while sending it*, sampling in the middle of
// every bit. This is the loopback the scanner was missing: the echo of our own bytes
// happens during transmission, and the receive path below only starts listening after
// the last one has gone, so `raw` could never have counted an echo -- only a genuine
// reply. A returned byte equal to the one sent proves three things at once that were
// previously assumed: the bit-banged transmitter works, the receiver works, and the
// two pins really are on the same wire.
static uint8_t bbWriteEcho(uint8_t txPin, uint8_t rxPin, uint8_t value) {
  uint8_t echo = 0;
  noInterrupts();
  digitalWrite(txPin, LOW);
  delayMicroseconds(BIT_US - TX_TRIM);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(txPin, (value & 0x01) ? HIGH : LOW);
    value = (uint8_t)(value >> 1);
    delayMicroseconds(BIT_US / 2);
    if (digitalRead(rxPin)) echo |= (uint8_t)(1 << i);
    delayMicroseconds(BIT_US / 2 - TX_TRIM);
  }
  digitalWrite(txPin, HIGH);
  interrupts();
  delayMicroseconds(BIT_US);
  return echo;
}

static void bbWrite(uint8_t txPin, uint8_t value) {
  noInterrupts();
  digitalWrite(txPin, LOW);
  delayMicroseconds(BIT_US - TX_TRIM);
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(txPin, (value & 0x01) ? HIGH : LOW);
    value = (uint8_t)(value >> 1);
    delayMicroseconds(BIT_US - TX_TRIM);
  }
  digitalWrite(txPin, HIGH);
  interrupts();
  delayMicroseconds(BIT_US);
}

// One byte, or -1 on timeout. Interrupts are off only for the byte itself, so the
// micros() the timeout runs on keeps ticking between bytes.
static int16_t bbRead(uint8_t rxPin, uint32_t timeoutUs) {
  const uint32_t started = micros();
  while (digitalRead(rxPin) == HIGH) {
    if (micros() - started > timeoutUs) return -1;
  }
  noInterrupts();
  delayMicroseconds(BIT_US + BIT_US / 2 - RX_TRIM);
  uint8_t value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (digitalRead(rxPin)) value |= (uint8_t)(1 << i);
    delayMicroseconds(BIT_US - RX_TRIM);
  }
  interrupts();
  return (int16_t)value;
}

static void idle(uint8_t rxPin, uint8_t txPin) {
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, HIGH);
  // Pulled up, so an unconnected pin reads high and cannot be mistaken for a line
  // that is being held low by something real.
  pinMode(rxPin, INPUT_PULLUP);
}

// How fast the line falls once it is released from high, in microseconds.
//
// `link` only says the two pins share copper -- which is equally true when both reach
// PDN_UART and when the two wires simply touch each other past the chip. This tells
// those apart. A bare wire is a capacitor with nowhere to discharge and stays high for
// milliseconds; a real PDN_UART hangs a weak pull-down on the node and collapses in
// tens of microseconds. Returns the timeout value when it never falls.
static const uint16_t DECAY_TIMEOUT_US = 5000;

static uint16_t decayMicros(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(500);
  pinMode(pin, INPUT);  // released, and deliberately without the internal pull-up
  const uint32_t started = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - started > DECAY_TIMEOUT_US) return DECAY_TIMEOUT_US;
  }
  return (uint16_t)(micros() - started);
}

// Drive one pin, read the other. The module joins both of its pads at PDN_UART, so
// on a correctly wired board the receive pin must follow the transmit pin down.
static bool dcLinked(uint8_t rxPin, uint8_t txPin) {
  pinMode(txPin, OUTPUT);
  pinMode(rxPin, INPUT_PULLUP);
  digitalWrite(txPin, HIGH);
  delayMicroseconds(500);
  const bool high = digitalRead(rxPin) == HIGH;
  digitalWrite(txPin, LOW);
  delayMicroseconds(500);
  const bool low = digitalRead(rxPin) == LOW;
  digitalWrite(txPin, HIGH);
  delayMicroseconds(500);
  return high && low;
}

static uint8_t sendAndCollect(uint8_t rxPin, uint8_t txPin, const uint8_t* datagram, uint8_t len) {
  idle(rxPin, txPin);
  delayMicroseconds(200);
  lastEcho = bbWriteEcho(txPin, rxPin, datagram[0]);
  for (uint8_t i = 1; i < len; i++) bbWrite(txPin, datagram[i]);

  // The chip answers within a few bit times of the request. The MCU keeps driving
  // the transmit pin high meanwhile: the module's series resistor lets the chip pull
  // the shared node low regardless, which is the whole point of that resistor.
  uint8_t count = 0;
  uint32_t timeout = 20000;
  while (count < sizeof(rxBuf)) {
    const int16_t byteIn = bbRead(rxPin, timeout);
    if (byteIn < 0) break;
    rxBuf[count++] = (uint8_t)byteIn;
    timeout = 3000;
  }
  return count;
}

// Returns true and fills `value` when a whole, CRC-correct reply arrived.
static bool tmcRead(uint8_t rxPin, uint8_t txPin, uint8_t reg, uint32_t* value, uint8_t* rawCount) {
  uint8_t request[4] = {TMC_SYNC, TMC_ADDRESS, reg, 0};
  request[3] = tmcCrc8(request, 3);
  const uint8_t count = sendAndCollect(rxPin, txPin, request, 4);
  *rawCount = count;
  if (count < 8) return false;

  // The reply may sit behind an echo of the request, so look for the newest header
  // rather than assuming it starts at byte 0.
  for (int8_t start = (int8_t)(count - 8); start >= 0; start--) {
    const uint8_t* frame = rxBuf + start;
    if (frame[0] != TMC_SYNC || frame[1] != 0xFF) continue;
    if (tmcCrc8(frame, 7) != frame[7]) continue;
    *value = ((uint32_t)frame[3] << 24) | ((uint32_t)frame[4] << 16) |
             ((uint32_t)frame[5] << 8) | (uint32_t)frame[6];
    return true;
  }
  return false;
}

static void tmcWrite(uint8_t rxPin, uint8_t txPin, uint8_t reg, uint32_t value) {
  uint8_t request[8] = {TMC_SYNC, TMC_ADDRESS, (uint8_t)(reg | TMC_WRITE),
                        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                        (uint8_t)(value >> 8), (uint8_t)value, 0};
  request[7] = tmcCrc8(request, 7);
  idle(rxPin, txPin);
  delayMicroseconds(200);
  for (uint8_t i = 0; i < 8; i++) bbWrite(txPin, request[i]);
  delay(2);
}

// What the colour means. Ordered worst to best, and the order is used: the host is
// told only when the *best* of the two orientations changes rank.
enum LinkState : uint8_t {
  STATE_DEAD = 0,      // dark   -- line idles high and nothing is attached
  STATE_GROUNDED = 1,  // red    -- line is held low, a wire is sitting on ground
  STATE_LINKED = 2,    // blue   -- the two pins share copper, but the chip is silent
  STATE_TALKING = 3    // green  -- the TMC2209 answered, this is the orientation
};

struct Orientation {
  bool linked;
  // Level of the receive pin while nothing is being sent. A UART line idles HIGH, so
  // a low reading means something holds it down -- and then the bit-banged receiver
  // sees a start bit that never ends and fills its buffer with rubbish. Without this
  // field that rubbish reads as `raw=12` and looks exactly like a real reply.
  bool idleHigh;
  uint8_t echo;
  uint16_t decayUs;
  uint8_t raw;
  uint8_t version;
  int16_t ifcntDelta;
  bool replied;
};

static Orientation probe(uint8_t rxPin, uint8_t txPin, uint8_t slot) {
  Orientation out = {false, false, 0, 0, 0, 0, -1, false};
  out.linked = dcLinked(rxPin, txPin);

  // Release the transmit pin *first*. Measuring the idle level while it still drives
  // the line high reports that driver, not the line: with the two pins linked it read
  // high every time and said nothing at all. Against the ~40k internal pull-up alone,
  // a low reading now means a pull-down stronger than about 20k -- which is a
  // standalone-mode strap to ground, not a UART line waiting to talk.
  pinMode(txPin, INPUT);
  pinMode(rxPin, INPUT_PULLUP);
  delayMicroseconds(500);
  out.idleHigh = digitalRead(rxPin) == HIGH;

  out.decayUs = decayMicros(rxPin);

  uint32_t ioin = 0;
  out.replied = tmcRead(rxPin, txPin, REG_IOIN, &ioin, &out.raw);
  out.echo = lastEcho;
  if (!out.replied) return out;

  out.version = (uint8_t)(ioin >> 24);
  everRepliedVersion[slot] = out.version;
  everRepliedCount[slot]++;

  // Only worth the extra milliseconds once the chip is known to be talking.
  uint32_t before = 0;
  uint32_t after = 0;
  uint8_t ignored = 0;
  if (tmcRead(rxPin, txPin, REG_IFCNT, &before, &ignored)) {
    tmcWrite(rxPin, txPin, REG_TPOWERDOWN, 20);
    if (tmcRead(rxPin, txPin, REG_IFCNT, &after, &ignored)) {
      out.ifcntDelta = (int16_t)((uint8_t)after - (uint8_t)before);
    }
  }
  return out;
}

static LinkState classify(const Orientation& o) {
  // Any datagram that passed sync, address and CRC is the chip talking. Which chip it
  // is comes out of `version`, and is not a precondition for hearing it.
  if (o.replied) return STATE_TALKING;
  // The echo of our own sync byte is the sharpest signal we have, so it drives the
  // colour. It came out of the loopback fix and beats `link`/`idle` at their own job:
  //   0x05 -- the byte we sent, so the wire carries our bits faithfully
  //   0x00 -- the line never rose, i.e. it is pinned to ground
  //   0xFF -- the line never fell, i.e. nothing of ours reaches it
  if (o.echo == TMC_SYNC) return STATE_LINKED;
  if (o.echo == 0x00) return STATE_GROUNDED;
  return STATE_DEAD;
}

static void showState(uint8_t r, uint8_t g, uint8_t b, LinkState state) {
  digitalWrite(r, state == STATE_GROUNDED ? HIGH : LOW);
  digitalWrite(g, state == STATE_TALKING ? HIGH : LOW);
  digitalWrite(b, state == STATE_LINKED ? HIGH : LOW);
}

// Which LED is which is not obvious by looking, so the board says so once at boot:
// three green blinks on the first orientation's LED, then three on the second.
static void identifyLeds() {
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_A_G, HIGH);
    delay(150);
    digitalWrite(LED_A_G, LOW);
    delay(150);
  }
  delay(400);
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_B_G, HIGH);
    delay(150);
    digitalWrite(LED_B_G, LOW);
    delay(150);
  }
}

static void report(const char* tag, const Orientation& o, uint8_t slot) {
  Serial.print(tag);
  Serial.print(F("_link="));
  Serial.print(o.linked ? 1 : 0);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_idle="));
  Serial.print(o.idleHigh ? 1 : 0);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_echo="));
  Serial.print(o.echo);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_decay="));
  Serial.print(o.decayUs);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_raw="));
  Serial.print(o.raw);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_ver="));
  Serial.print(o.version);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_ifcnt="));
  Serial.print(o.ifcntDelta);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_state="));
  Serial.print((uint8_t)classify(o));
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_everver="));
  Serial.print(everRepliedVersion[slot]);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_evercount="));
  Serial.print(everRepliedCount[slot]);
  Serial.print(';');
}

void setup() {
  Serial.begin(115200);
  analogReference(INTERNAL);
  analogRead(POWER_SENSE_PIN);
  analogRead(POWER_SENSE_PIN);
  const uint8_t leds[] = {LED_A_R, LED_A_G, LED_A_B, LED_B_R, LED_B_G, LED_B_B, LED_ALIVE};
  for (uint8_t i = 0; i < sizeof(leds); i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }
  delay(50);
  Serial.println(F("ready"));
  identifyLeds();
}

static uint16_t supplyCentivolts() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += (uint32_t)analogRead(POWER_SENSE_PIN);
  const float counts = (float)sum / 8.0f;
  const float volts = counts * ADC_INTERNAL_VREF / 1023.0f * POWER_DIVIDER_RATIO;
  return (uint16_t)(volts * 100.0f + 0.5f);
}

void loop() {
  // Orientation A is what the main firmware assumes: pin 8 listens, pin 9 talks.
  const Orientation a = probe(PIN_A, PIN_B, 0);
  const Orientation b = probe(PIN_B, PIN_A, 1);

  showState(LED_A_R, LED_A_G, LED_A_B, everRepliedCount[0] ? STATE_TALKING : classify(a));
  showState(LED_B_R, LED_B_G, LED_B_B, everRepliedCount[1] ? STATE_TALKING : classify(b));
  // A pulse per round: the eye can tell a scanner that is running from one that hung.
  digitalWrite(LED_ALIVE, !digitalRead(LED_ALIVE));

  Serial.print(F("1;vm="));
  Serial.print(supplyCentivolts());
  Serial.print(';');
  report("a", a, 0);
  report("b", b, 1);
  Serial.println();

  // Everything is released between rounds, so a wire moved mid-round cannot leave a
  // pin driving into whatever it lands on next.
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  delay(150);
}
