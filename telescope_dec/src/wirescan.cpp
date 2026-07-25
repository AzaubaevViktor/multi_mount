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
  for (uint8_t i = 0; i < len; i++) bbWrite(txPin, datagram[i]);

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

struct Orientation {
  bool linked;
  // Level of the receive pin while nothing is being sent. A UART line idles HIGH, so
  // a low reading means something holds it down -- and then the bit-banged receiver
  // sees a start bit that never ends and fills its buffer with rubbish. Without this
  // field that rubbish reads as `raw=12` and looks exactly like a real reply.
  bool idleHigh;
  uint8_t raw;
  uint8_t version;
  int16_t ifcntDelta;
  bool replied;
};

static Orientation probe(uint8_t rxPin, uint8_t txPin) {
  Orientation out = {false, false, 0, 0, -1, false};
  out.linked = dcLinked(rxPin, txPin);

  idle(rxPin, txPin);
  delayMicroseconds(500);
  out.idleHigh = digitalRead(rxPin) == HIGH;

  uint32_t ioin = 0;
  out.replied = tmcRead(rxPin, txPin, REG_IOIN, &ioin, &out.raw);
  if (!out.replied) return out;

  out.version = (uint8_t)(ioin >> 24);

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

static void report(const char* tag, const Orientation& o) {
  Serial.print(tag);
  Serial.print(F("_link="));
  Serial.print(o.linked ? 1 : 0);
  Serial.print(';');
  Serial.print(tag);
  Serial.print(F("_idle="));
  Serial.print(o.idleHigh ? 1 : 0);
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
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("ready"));
}

void loop() {
  // Orientation A is what the main firmware assumes: pin 8 listens, pin 9 talks.
  const Orientation a = probe(PIN_A, PIN_B);
  const Orientation b = probe(PIN_B, PIN_A);

  Serial.print(F("1;"));
  report("a", a);
  report("b", b);
  Serial.println();

  // Everything is released between rounds, so a wire moved mid-round cannot leave a
  // pin driving into whatever it lands on next.
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  delay(150);
}
