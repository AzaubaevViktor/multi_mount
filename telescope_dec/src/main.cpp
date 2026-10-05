/*
  Nano v3 + TMC2209 + TMCStepper
  Wiring (typical 1-wire UART):
    
    GND common, VM+motor power as usual.

  STEP/DIR/EN to your driver module pins.
*/

#include <Arduino.h>
#include <AltSoftSerial.h>
#include <TMCStepper.h>
#include <limits.h>
#include <math.h>

#include "frame_v3.h"

// ---------- Pins ----------
static const uint8_t STEP_PIN = 7;
static const uint8_t DIR_PIN  = 4;
static const uint8_t EN_PIN   = 12;   // Enable pin to driver
static const bool    EN_ACTIVE_LOW = true;

static const uint8_t POWER_LED_PIN = 11;
static const uint8_t POWER_SENSE_PIN = A1;

static const uint8_t STEP_RGB_RED_PIN = 6;
static const uint8_t STEP_RGB_GREEN_PIN = 5;
static const uint8_t STEP_RGB_BLUE_PIN = 3;

static const uint8_t MODE_LED_RED_PIN = A3;
static const uint8_t MODE_LED_GREEN_PIN = A2;
static const uint8_t MODE_LED_BLUE_PIN = A5;

static const uint8_t TMC_RX_PIN = 8;
static const uint8_t TMC_TX_PIN = 9;

// ---------- TMC config ----------
static const uint32_t TMC_BAUD = 19200;
// Most SilentStepStick-like modules use 0.11 ohm; check your board to be correct.
static const float R_SENSE = 0.11f;
static const uint16_t DRIVER_RUN_CURRENT_MA = 600;
static const uint16_t DRIVER_DERATED_CURRENT_MA = 400;
static const uint32_t DRIVER_POLL_INTERVAL_MS = 1000;
static const uint8_t DRIVER_CLEAR_POLLS = 5;
static const uint32_t DRIVER_NORMAL_SPEED_LIMIT_SPS = 40000;
static const uint32_t DRIVER_DERATED_SPEED_LIMIT_SPS = 1000;
static const uint8_t DRIVER_FLAG_OTPW = 0x01;
static const uint8_t DRIVER_FLAG_OT = 0x02;
static const uint8_t DRIVER_FLAG_SHORT = 0x04;
static const uint8_t DRIVER_FLAG_OPEN_LOAD = 0x08;
static const uint8_t DRIVER_FLAG_STALL = 0x10;
static const uint8_t DRIVER_FLAG_UART = 0x80;
static const uint8_t DRIVER_STATUS_SHORT_MASK = 0x3C;
static const uint8_t DRIVER_STATUS_OPEN_LOAD_MASK = 0xC0;
static const uint8_t DRIVER_UART_FAILURE_LIMIT = 2;
static const uint32_t DRIVER_STALL_POLL_INTERVAL_MS = 200;
static const uint32_t DRIVER_STALL_ARM_TIME_MS = 800;
static const uint16_t DRIVER_STALL_MIN_SPEED_SPS = 500;
static const uint16_t DRIVER_STALL_STRONG_SG = 8;
static const uint16_t DRIVER_STALL_LOADED_SG = 20;
static const uint16_t DRIVER_STALL_CLEAR_SG = 40;
static const uint8_t DRIVER_STALL_SCORE_LIMIT = 6;
static const uint8_t FINE_APPROACH_FULL_STEPS = 32;
// Address is the MS1/MS2 strapping: ADDR = MS2<<1 | MS1.
//
// The original 9600-baud link was silent in both directions even with the driver
// powered. After the PDN_UART wiring was repaired, `tmc_wire` reported linked=1 and
// five consecutive scans at both 19200 and 38400 baud returned VERSION=0x21 and
// IFCNT+1. The same scans still failed at 9600, so this board uses address 0b00 and
// TMC_BAUD=19200.
//
// Normal reads now return stable nonzero registers and `tmc_scan` proves writes with
// the chip's own IFCNT counter. `get` and `set` therefore reach silicon at this baud.
static const uint8_t DRIVER_ADDRESS = 0b00;

static const float ADC_INTERNAL_VREF = 1.1f;
static const float POWER_DIVIDER_RATIO = 23.93555f;       // Vin = Vadc * ratio
static const float POWER_SOLID_THRESHOLD_V = 12.0f;
static const float POWER_EXP_BASE_V = 10.0f;
static const uint16_t POWER_BLINK_BASE_MS = 250;
static const uint32_t POWER_SAMPLE_INTERVAL_MS = 200;

static const uint16_t STEP_COLOR_LUT_SIZE = 128;
static const uint8_t STEP_COLOR_GREEN_SHIFT = 85;     // 120 degrees for 256-step LUT
static const uint8_t STEP_COLOR_BLUE_SHIFT = 170;     // 240 degrees for 256-step LUT
static const uint8_t MAX_STEP_LIGHT = 4;

AltSoftSerial TMCSerial(TMC_RX_PIN, TMC_TX_PIN);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Motion state (v2) ----------
struct RunnerV2 {
  bool enabled = false;
  bool dir = false;
  bool running = false;
  bool stopRequested = false;
  bool hasTarget = false;
  bool freeRideMode = false;
  long target = 0;
  float speedSps = 500.0f;
  float actualSpeedSps = 0.0f;
  float desiredSpeedSps = 0.0f;
  float accelStepsPerUs = 0.001f;
  uint32_t stepIntervalUs = 2000;
  uint32_t pulseWidthUs   = 3;
  uint32_t nextStepUs     = 0;
  bool stepHigh = false;
  uint32_t stepHighUntilUs = 0;
  uint32_t lastStepUs = 0;
  uint32_t lastUpdateUs = 0;
  uint32_t lastStepperUs = 0;
  float stepAcc = 0.0f;
} runV2;

static bool v2Initialized = false;

struct Profiler {
  float serial = 0.0;
  float serialLastProcessed = 0.0;

  float stepper = 0.0;
  float updateMotionState = 0.0;
  float stepperHigh = 0.0;
  float stepperLow = 0.0;
} profiler;

// ---------- Step position counter ----------
static const long STEP_POSITIVE = 1;
static const long STEP_NEGATIVE = -1;

static long stepPosition = 0;

static inline void setPosition(long value) {
  stepPosition = value;
}

static inline long getPosition() {
  return stepPosition;
}

static const char* const PHASE_IDLE_V2 = "idle";
static const char* const PHASE_HOLD_V2 = "hold";
static const char* const PHASE_ACCEL_V2 = "acceleration";
static const char* const PHASE_RUN_V2 = "running";
static const char* const PHASE_DECEL_V2 = "deceleration";
static const char* const MODE_TARGET_V2 = "target";
static const char* const MODE_FREE_RIDE_V2 = "free_ride";

enum MotionPhaseCodeV2 : uint8_t {
  MOTION_PHASE_IDLE_V2 = 0,
  MOTION_PHASE_HOLD_V2,
  MOTION_PHASE_ACCEL_V2,
  MOTION_PHASE_RUN_V2,
  MOTION_PHASE_DECEL_V2
};

enum SafetyStateCodeV2 : uint8_t {
  SAFETY_NORMAL_V2 = 0,
  SAFETY_DERATED_V2,
  SAFETY_STOPPING_V2,
  SAFETY_SHUTDOWN_V2
};

struct LedStateV2 {
  float supplyVoltageV = 0.0f;
  uint16_t powerBlinkHalfPeriodMs = 0;
  bool powerLedOn = true;
  uint32_t powerLastSampleMs = 0;
  uint32_t powerLastToggleMs = 0;
  long lastStepForColor = LONG_MIN;
  uint16_t microsteps = 16;
  uint32_t stepColorCycle = 200UL * 16UL;
} ledStateV2;

static uint8_t stepColorLutV2[STEP_COLOR_LUT_SIZE];

static SafetyStateCodeV2 safetyStateV2 = SAFETY_NORMAL_V2;
static uint32_t driverStatusV2 = 0;
static uint8_t driverFlagsV2 = DRIVER_FLAG_UART;
static uint16_t safetyEventsV2 = 0;
static uint32_t driverPolledAtMsV2 = 0;
static uint8_t driverClearPollsV2 = 0;
static uint8_t driverUartFailuresV2 = 0;
static uint32_t driverSgPolledAtMsV2 = 0;
static uint32_t driverStallEligibleAtMsV2 = 0;
static uint16_t driverSgResultV2 = 0;
static uint8_t driverStallScoreV2 = 0;
static bool driverStallLatchedV2 = false;
static uint16_t fineMicrostepsV2 = 16;

static MotionPhaseCodeV2 getPhaseCodeV2();
static inline uint32_t getSafetySpeedLimitV2();
static inline void setDriverMicrostepsV2(uint16_t microsteps);
static bool applyDriverMicrostepsV2(uint16_t microsteps);
static void buildStepColorLutV2();
static void runStartupLedSequenceV2();
static void serviceLedsV2();
static void samplePowerVoltageV2(uint32_t nowMs);

// ---------- V2 formatting ----------
static const uint8_t HEX_WIDTH = 8;

// ---------- Simple line parser ----------
static char lineBufV2[256];
static uint8_t lineLenV2 = 0;

// ---------- Fast TX ring buffer ----------
static char outBufV2[256];
static uint8_t outWriteV2 = 0;
static uint8_t outReadV2 = 0;

// One slot is always kept empty so that outWriteV2 == outReadV2 means "empty" and never
// "full"; usable capacity is therefore sizeof(outBufV2) - 1 == 255 bytes.
static const uint8_t OUT_CAPACITY_V2 = (uint8_t)(sizeof(outBufV2) - 1);

// Replacement emitted by respondEndV2() for a response that did not fit. Kept short so that
// the room it needs is almost always there.
static const char OUT_OVERFLOW_LINE_V2[] = "0;error=tx_overflow;\n";
static const uint8_t OUT_OVERFLOW_LINE_LEN_V2 = (uint8_t)(sizeof(OUT_OVERFLOW_LINE_V2) - 1);

// Bytes handed to the UART per loop() iteration. A non-blocking Serial.write() costs ~2 us,
// so the ceiling adds at most ~35 us to an iteration, and only for the four iterations it
// takes to fill the 64-byte hardware TX buffer right after a response is built; past that
// the wire (87 us per byte at 115200) is the limit and the batch stays empty. 35 us is well
// inside one step period even at the 6000 steps/s the host asks for, while a ~180-byte
// status response now clears the ring in ~12 iterations instead of ~180.
static const uint8_t TX_DRAIN_MAX_V2 = 16;

// Write index at the start of the response currently being built, used to roll a
// half-written response back instead of letting it reach the host truncated.
static uint8_t outLineStartV2 = 0;
static bool outLineFailedV2 = false;
static uint16_t txOverflowCountV2 = 0;

static inline bool outHasDataV2() {
  return outWriteV2 != outReadV2;
}

static inline uint8_t outFreeV2() {
  return (uint8_t)(OUT_CAPACITY_V2 - (uint8_t)(outWriteV2 - outReadV2));
}

static inline bool outAppendCharV2(char c) {
  // Once a response has failed, every further byte of it is dropped: partial output is
  // worse than none, it glues itself to the next response.
  if (outLineFailedV2) return false;
  uint8_t next = (uint8_t)(outWriteV2 + 1);
  if (next == outReadV2) {
    outLineFailedV2 = true;
    return false;
  }
  outBufV2[outWriteV2] = c;
  outWriteV2 = next;
  return true;
}

static inline bool outPopV2(char* c) {
  if (!outHasDataV2()) return false;
  *c = outBufV2[outReadV2];
  outReadV2 = (uint8_t)(outReadV2 + 1);
  return true;
}

static inline void outAppendStrV2(const char* s) {
  if (!s) return;
  while (*s) {
    if (!outAppendCharV2(*s++)) return;
  }
}

static inline void outAppendStrV2(const __FlashStringHelper* s) {
  if (!s) return;
  PGM_P p = reinterpret_cast<PGM_P>(s);
  char c;
  while ((c = (char)pgm_read_byte(p++)) != 0) {
    if (!outAppendCharV2(c)) return;
  }
}

static inline void outAppendNumU32V2(uint32_t v) {
  char tmp[10];
  uint8_t n = 0;

  do {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v && n < sizeof(tmp));

  while (n > 0) {
    outAppendCharV2(tmp[--n]);
  }
}

static inline void outAppendNumLongV2(long v) {
  uint32_t value;
  if (v < 0) {
    outAppendCharV2('-');
    value = (uint32_t)(-(v + 1L)) + 1U;
  } else {
    value = (uint32_t)v;
  }
  outAppendNumU32V2(value);
}

static inline void outAppendNumFloatV2(float v, uint8_t decimals) {
  // dtostrf is faster than repeated Serial.print on AVR
  char tmp[32];
  dtostrf(v, 0, decimals, tmp);
  // dtostrf may pad leading spaces; trim them
  char* p = tmp;
  while (*p == ' ') p++;
  outAppendStrV2(p);
}

static inline void outAppendHexU32V2(uint32_t v) {
  outAppendCharV2('0');
  outAppendCharV2('x');
  for (int8_t i = HEX_WIDTH - 1; i >= 0; i--) {
    const uint8_t nibble = (uint8_t)((v >> (i * 4)) & 0x0F);
    outAppendCharV2((char)(nibble < 10 ? ('0' + nibble) : ('A' + nibble - 10)));
  }
}

static inline void outFlushLineV2() {
  outAppendCharV2('\n');
}

static inline void clearStatusLedsV2() {
  digitalWrite(POWER_LED_PIN, LOW);
  analogWrite(STEP_RGB_RED_PIN, 0);
  analogWrite(STEP_RGB_GREEN_PIN, 0);
  analogWrite(STEP_RGB_BLUE_PIN, 0);
  digitalWrite(MODE_LED_RED_PIN, LOW);
  digitalWrite(MODE_LED_GREEN_PIN, LOW);
  digitalWrite(MODE_LED_BLUE_PIN, LOW);
}

static inline void writeStartupLedV2(uint8_t pin, bool on) {
  const uint8_t level = on ? 255 : 0;
  if (pin == STEP_RGB_RED_PIN || pin == STEP_RGB_GREEN_PIN || pin == STEP_RGB_BLUE_PIN) {
    analogWrite(pin, level);
    return;
  }
  digitalWrite(pin, on ? HIGH : LOW);
}

static void buildStepColorLutV2() {
  for (uint16_t i = 0; i < STEP_COLOR_LUT_SIZE; i++) {
    const float phase = (2.0f * PI * (float)i) / (float)STEP_COLOR_LUT_SIZE;
    const float normalized = 0.5f + (0.5f * sinf(phase));
    stepColorLutV2[i] = (uint8_t)(normalized * 255. + 0.5f);
  }
}

static void runStartupLedSequenceV2() {
  static const uint8_t LED_PINS[] = {
    POWER_LED_PIN,
    STEP_RGB_RED_PIN,  // red
    STEP_RGB_GREEN_PIN,  // green
    STEP_RGB_BLUE_PIN,  // blue
    MODE_LED_RED_PIN,  // red
    MODE_LED_GREEN_PIN,  // green
    MODE_LED_BLUE_PIN  // blue
  };

  clearStatusLedsV2();
  for (uint8_t i = 0; i < (sizeof(LED_PINS) / sizeof(LED_PINS[0])); i++) {
    writeStartupLedV2(LED_PINS[i], true);
    delay(10);
    // writeStartupLedV2(LED_PINS[i], false);
    // delay(5);
  }
}

static float readSupplyVoltageV2() {
  uint32_t sum = 0;
  static const uint8_t SAMPLE_COUNT = 4;
  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
    sum += (uint32_t)analogRead(POWER_SENSE_PIN);
  }
  const float raw = (float)sum / (float)SAMPLE_COUNT;
  const float adcV = (raw * ADC_INTERNAL_VREF) / 1023.0f;
  return adcV * POWER_DIVIDER_RATIO;
}

static uint16_t calcPowerBlinkHalfPeriodMsV2(float voltageV) {
  if (voltageV >= POWER_SOLID_THRESHOLD_V) return 0;

  const float exponent = voltageV - POWER_EXP_BASE_V;
  float period = (float)POWER_BLINK_BASE_MS * powf(2.0f, exponent);
  if (period < 35.0f) period = 35.0f;
  if (period > 2000.0f) period = 2000.0f;
  return (uint16_t)(period + 0.5f);
}

static void samplePowerVoltageV2(uint32_t nowMs) {
  if (ledStateV2.powerLastSampleMs != 0 &&
      (uint32_t)(nowMs - ledStateV2.powerLastSampleMs) < POWER_SAMPLE_INTERVAL_MS) {
    return;
  }

  ledStateV2.powerLastSampleMs = nowMs;
  ledStateV2.supplyVoltageV = readSupplyVoltageV2();

  const uint16_t nextPeriod = calcPowerBlinkHalfPeriodMsV2(ledStateV2.supplyVoltageV);
  if (nextPeriod != ledStateV2.powerBlinkHalfPeriodMs) {
    ledStateV2.powerBlinkHalfPeriodMs = nextPeriod;
    ledStateV2.powerLastToggleMs = nowMs;
    ledStateV2.powerLedOn = true;
  }
}

static void updatePowerLedV2(uint32_t nowMs) {
  if (ledStateV2.powerBlinkHalfPeriodMs == 0) {
    ledStateV2.powerLedOn = true;
    digitalWrite(POWER_LED_PIN, HIGH);
    return;
  }

  if ((uint32_t)(nowMs - ledStateV2.powerLastToggleMs) >= ledStateV2.powerBlinkHalfPeriodMs) {
    ledStateV2.powerLastToggleMs = nowMs;
    ledStateV2.powerLedOn = !ledStateV2.powerLedOn;
  }

  digitalWrite(POWER_LED_PIN, ledStateV2.powerLedOn ? HIGH : LOW);
}

static inline uint32_t positiveModuloV2(long value, uint32_t modulo) {
  if (modulo == 0U) return 0U;
  const long rem = value % (long)modulo;
  return (uint32_t)(rem < 0 ? rem + (long)modulo : rem);
}

static void updateStepColorLedsV2() {
  const long position = getPosition();
  if (position == ledStateV2.lastStepForColor) return;
  ledStateV2.lastStepForColor = position;

  if (ledStateV2.stepColorCycle == 0U) ledStateV2.stepColorCycle = 200U;
  const uint32_t wrapped = positiveModuloV2(position, ledStateV2.stepColorCycle);
  const uint8_t idx = (uint8_t)((wrapped * STEP_COLOR_LUT_SIZE) / ledStateV2.stepColorCycle);

  analogWrite(STEP_RGB_RED_PIN, stepColorLutV2[idx] / MAX_STEP_LIGHT);
  analogWrite(STEP_RGB_GREEN_PIN, stepColorLutV2[(uint8_t)(idx + STEP_COLOR_GREEN_SHIFT)] / MAX_STEP_LIGHT);
  analogWrite(STEP_RGB_BLUE_PIN, stepColorLutV2[(uint8_t)(idx + STEP_COLOR_BLUE_SHIFT)] / MAX_STEP_LIGHT);
}

static void updateModeLedsV2() {
  const MotionPhaseCodeV2 phase = getPhaseCodeV2();
  bool red = false;
  bool green = false;
  bool blue = false;

  switch (phase) {
    case MOTION_PHASE_IDLE_V2:
      blue = true;
      break;
    case MOTION_PHASE_HOLD_V2:
      red = true;
      break;
    case MOTION_PHASE_ACCEL_V2:
      red = true;
      green = true;
      break;
    case MOTION_PHASE_DECEL_V2:
      red = true;
      blue = true;
      break;
    case MOTION_PHASE_RUN_V2:
      green = true;
      if (runV2.freeRideMode) {
        blue = true;
      }
      break;
  }

  digitalWrite(MODE_LED_RED_PIN, red ? HIGH : LOW);
  digitalWrite(MODE_LED_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(MODE_LED_BLUE_PIN, blue ? HIGH : LOW);
}

static void serviceLedsV2() {
  const uint32_t nowMs = millis();
  samplePowerVoltageV2(nowMs);
  updatePowerLedV2(nowMs);
  updateStepColorLedsV2();
  updateModeLedsV2();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(POWER_LED_PIN, OUTPUT);
  pinMode(STEP_RGB_RED_PIN, OUTPUT);
  pinMode(STEP_RGB_GREEN_PIN, OUTPUT);
  pinMode(STEP_RGB_BLUE_PIN, OUTPUT);
  pinMode(MODE_LED_RED_PIN, OUTPUT);
  pinMode(MODE_LED_GREEN_PIN, OUTPUT);
  pinMode(MODE_LED_BLUE_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  runV2.dir = false;
  digitalWrite(DIR_PIN, LOW);
  runV2.enabled = false;
  digitalWrite(EN_PIN, EN_ACTIVE_LOW ? HIGH : LOW);
  clearStatusLedsV2();
  runV2.lastStepUs = micros();
  runV2.lastUpdateUs = micros();
  runV2.nextStepUs = 0;

  analogReference(INTERNAL);
  delay(5);
  analogRead(POWER_SENSE_PIN);
  analogRead(POWER_SENSE_PIN);

  buildStepColorLutV2();
  runStartupLedSequenceV2();

  TMCSerial.begin(TMC_BAUD);

  // while (!TMCSerial) {};

  // Basic driver init (safe-ish defaults; tune later)
  driver.begin();
  driver.pdn_disable(true);          // use UART
  driver.I_scale_analog(false);      // use internal current reference
  driver.mstep_reg_select(true);     // microsteps via registers (UART)
  driver.toff(4);                    // enable driver
  driver.blank_time(24);
  driver.rms_current(DRIVER_RUN_CURRENT_MA);
  setDriverMicrostepsV2(16);
  ledStateV2.microsteps = 16;
  fineMicrostepsV2 = 16;
  ledStateV2.stepColorCycle = 200UL * (uint32_t)ledStateV2.microsteps;
  driver.en_spreadCycle(false);      // stealth by default
  driver.pwm_autoscale(true);

  // Clear latched flags
  driver.GSTAT(0x7);
  driverStatusV2 = driver.DRV_STATUS();
  if (driverStatusV2 != 0 && driverStatusV2 != 0xFFFFFFFFUL) {
    driverFlagsV2 = 0;
    driverUartFailuresV2 = 0;
  }

  const uint32_t nowMs = millis();
  samplePowerVoltageV2(nowMs);
  ledStateV2.powerLastToggleMs = nowMs;
  serviceLedsV2();

  v2Initialized = true;
  Serial.println(F("ready"));
}

static inline void setEnableV2(bool on) {
  runV2.enabled = on;
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

static void acknowledgeStallV2() {
  if (!driverStallLatchedV2) return;

  driverStallLatchedV2 = false;
  driverStallScoreV2 = 0;
  driverStallEligibleAtMsV2 = 0;
  driverFlagsV2 &= (uint8_t)~DRIVER_FLAG_STALL;

  if (safetyStateV2 == SAFETY_SHUTDOWN_V2 &&
      !(driverFlagsV2 & (DRIVER_FLAG_OT | DRIVER_FLAG_SHORT | DRIVER_FLAG_UART))) {
    if (driverFlagsV2 & DRIVER_FLAG_OTPW) {
      driver.rms_current(DRIVER_DERATED_CURRENT_MA);
      safetyStateV2 = SAFETY_DERATED_V2;
    } else {
      driver.rms_current(DRIVER_RUN_CURRENT_MA);
      safetyStateV2 = SAFETY_NORMAL_V2;
    }
    driverClearPollsV2 = 0;
  }
}

static inline void setDirV2(bool dir) {
  runV2.dir = dir;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
}

static bool parseLongV2(const char* s, long* value) {
  if (!s || !*s) return false;
  char* end = nullptr;
  long parsed = strtol(s, &end, 10);
  if (end == s || *end != 0) return false;
  *value = parsed;
  return true;
}

static bool parseU32V2(const char* s, uint32_t* value) {
  if (!s || !*s) return false;
  char* end = nullptr;
  unsigned long parsed = strtoul(s, &end, 10);
  if (end == s || *end != 0) return false;
  *value = (uint32_t)parsed;
  return true;
}

static void respondStartV2(bool ok) {
  outLineStartV2 = outWriteV2;
  outLineFailedV2 = false;

  outAppendCharV2(ok ? '1' : '0');
  outAppendCharV2(';');
}

static void respondEndV2() {
  outFlushLineV2();
  if (!outLineFailedV2) return;

  // Roll the write index back to the start of this response. Bytes below outLineStartV2
  // belong to earlier, not yet drained responses and are untouched; bytes above it belong
  // to this response only, and draining never runs while a response is being built
  // (serviceSerialv2() drains after handleLineV2() returns), so nothing is lost.
  outWriteV2 = outLineStartV2;
  outLineFailedV2 = false;
  if (txOverflowCountV2 < 0xFFFF) txOverflowCountV2++;

  // The rollback gives back exactly the space this response had when it started, and
  // draining only ever frees more, so the marker fits whenever the response began with at
  // least OUT_OVERFLOW_LINE_LEN_V2 bytes free. Below that the host is more than 234 bytes
  // behind and nothing can be delivered anyway: stay silent and let the counter report it.
  if (outFreeV2() >= OUT_OVERFLOW_LINE_LEN_V2) {
    outAppendStrV2(OUT_OVERFLOW_LINE_V2);
  }
}

// ---------- Framed protocol v3 ----------
// The codec itself lives in include/frame_v3.h so that it can be compiled for the
// host and checked against the golden frames without a board (see that header).
// What stays here is what only the firmware can do: feed the TX ring, roll a
// half-written frame back, and act on the command.
//
// The line protocol above is kept alive next to it: the host probes with a HELLO
// frame and falls back to lines when the answer is `0;error=unknown_cmd;`, so a
// board flashed with this firmware serves both a new and an old host.

static void outSinkV3(char c) { outAppendCharV2(c); }

static FrameWriterV3 frameWriterV3 = {outSinkV3, 0xFFFF};
static uint8_t frameSeqV3 = 0;

// '#' + 2*(3 header + 1 payload + 2 crc) + '\n'
static const uint8_t ERROR_FRAME_LEN_V3 = 14;

static void frameStartV3(uint8_t op, uint8_t seq, uint8_t payloadLen) {
  outLineStartV2 = outWriteV2;
  outLineFailedV2 = false;
  frameWriterV3.begin(op, seq, payloadLen);
}

static void frameEndV3() {
  frameWriterV3.end();
  if (!outLineFailedV2) return;

  // Same rollback as respondEndV2: a half-written frame glued onto the next one is
  // exactly what the length and the CRC exist to prevent, so it must never leave.
  outWriteV2 = outLineStartV2;
  outLineFailedV2 = false;
  if (txOverflowCountV2 < 0xFFFF) txOverflowCountV2++;
  if (outFreeV2() >= ERROR_FRAME_LEN_V3) {
    frameWriterV3.begin(OP_ERROR_V3, frameSeqV3, 1);
    frameWriterV3.byte(ERR_TX_OVERFLOW_V3);
    frameWriterV3.end();
  }
}

static void respondFrameErrorV3(uint8_t seq, uint8_t code) {
  frameStartV3(OP_ERROR_V3, seq, 1);
  frameWriterV3.byte(code);
  frameEndV3();
}

static void respondKeyValueLongV2(const char* key, long value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumLongV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueLongV2(const __FlashStringHelper* key, long value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumLongV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueU32V2(const char* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumU32V2(value);
  outAppendCharV2(';');
}

static void respondKeyValueU32V2(const __FlashStringHelper* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumU32V2(value);
  outAppendCharV2(';');
}

static void respondKeyValueBoolV2(const char* key, bool value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendCharV2(value ? '1' : '0');
  outAppendCharV2(';');
}

static void respondKeyValueBoolV2(const __FlashStringHelper* key, bool value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendCharV2(value ? '1' : '0');
  outAppendCharV2(';');
}

static void respondKeyValueFloatV2(const char* key, float value, uint8_t decimals) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumFloatV2(value, decimals);
  outAppendCharV2(';');
}

static void respondKeyValueFloatV2(
    const __FlashStringHelper* key,
    float value,
    uint8_t decimals) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendNumFloatV2(value, decimals);
  outAppendCharV2(';');
}

static void respondKeyValueStrV2(const char* key, const char* value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendStrV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueStrV2(const __FlashStringHelper* key, const char* value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendStrV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueStrV2(
    const __FlashStringHelper* key,
    const __FlashStringHelper* value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendStrV2(value);
  outAppendCharV2(';');
}

static void respondKeyValueHexV2(const char* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendHexU32V2(value);
  outAppendCharV2(';');
}

static void respondKeyValueHexV2(const __FlashStringHelper* key, uint32_t value) {
  outAppendStrV2(key);
  outAppendCharV2('=');
  outAppendHexU32V2(value);
  outAppendCharV2(';');
}

static void respondErrorV2(const char* msg) {
  respondStartV2(false);
  if (msg && *msg) {
    respondKeyValueStrV2("error", msg);
  }
  respondEndV2();
}

static void respondErrorV2(const __FlashStringHelper* msg) {
  respondStartV2(false);
  respondKeyValueStrV2(F("error"), msg);
  respondEndV2();
}

static char* stripParamPrefixV2(char* token) {
  if (token && token[0] == ':') return token + 1;
  return token;
}

static bool parseNameValueTokenV2(char* token, char** name, char** value) {
  if (!token || !name || !value) return false;
  char* trimmed = stripParamPrefixV2(token);
  char* eq = strchr(trimmed, '=');
  if (!eq) return false;
  *eq = 0;
  *name = trimmed;
  *value = eq + 1;
  if (!**name || !**value) return false;
  return true;
}

static const uint16_t MICROSTEPS_ALLOWED_V2[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
static const uint32_t TPWMTHRS_MAX_V2 = 0xFFFFF;

static bool isMicrostepsAllowedV2(uint16_t value) {
  for (uint8_t i = 0; i < (sizeof(MICROSTEPS_ALLOWED_V2) / sizeof(MICROSTEPS_ALLOWED_V2[0])); i++) {
    if (MICROSTEPS_ALLOWED_V2[i] == value) return true;
  }
  return false;
}

// --- Microsteps/CHOPCONF helpers ---
static inline uint16_t microstepsFromChopconfV2(uint32_t chopconf) {
  // CHOPCONF.MRES bits [24..27], encoding: 0->256, 1->128, 2->64, ..., 8->1
  const uint8_t mres = (uint8_t)((chopconf >> 24) & 0x0F);
  if (mres >= 8) return 1;
  return (uint16_t)(256U >> mres);
}

static inline bool intpolFromChopconfV2(uint32_t chopconf) {
  // CHOPCONF.INTPOL bit 28
  return ((chopconf >> 28) & 0x01) != 0;
}

static inline void setDriverMicrostepsV2(uint16_t microsteps) {
  // TMCStepper 0.7.3 exposes full-step MRES=8 as the special argument 0, while
  // this firmware and both wire protocols use the conventional value 1.
  driver.microsteps(microsteps == 1 ? 0 : microsteps);
}

static bool applyDriverMicrostepsV2(uint16_t microsteps) {
  setDriverMicrostepsV2(microsteps);
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (microstepsFromChopconfV2(driver.CHOPCONF()) == microsteps) return true;
  }
  return false;
}

static uint8_t calcCurrentScaleFromMaV2(uint16_t mA, bool highSense) {
  const float vfs = highSense ? 0.180f : 0.325f;
  float cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / vfs - 1.0f;
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  return (uint8_t)(cs + 0.5f);
}

static uint8_t calcRunCurrentScaleFromMaV2(uint16_t mA) {
  float cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / 0.325f - 1.0f;
  if (cs < 16.0f) {
    driver.vsense(true);
    cs = 32.0f * 1.41421f * mA / 1000.0f * (R_SENSE + 0.02f) / 0.180f - 1.0f;
  } else {
    driver.vsense(false);
  }
  if (cs < 0.0f) cs = 0.0f;
  if (cs > 31.0f) cs = 31.0f;
  return (uint8_t)(cs + 0.5f);
}

static uint8_t calcHoldCurrentScaleFromMaV2(uint16_t mA) {
  return calcCurrentScaleFromMaV2(mA, driver.vsense());
}

static bool appendParamValueV2(const char* name, bool emit) {
  if (!strcmp(name, "ihold")) {
    if (emit) respondKeyValueU32V2("ihold", driver.cs2rms(driver.ihold()));
    return true;
  }
  if (!strcmp(name, "irun")) {
    if (emit) respondKeyValueU32V2("irun", driver.cs2rms(driver.irun()));
    return true;
  }
  if (!strcmp(name, "tpowerdown")) {
    if (emit) respondKeyValueU32V2("tpowerdown", driver.TPOWERDOWN());
    return true;
  }
  if (!strcmp(name, "tpwmthrs")) {
    if (emit) respondKeyValueU32V2("tpwmthrs", driver.TPWMTHRS());
    return true;
  }
  if (!strcmp(name, "sgthrs")) {
    if (emit) respondKeyValueU32V2("sgthrs", driver.SGTHRS());
    return true;
  }
  if (!strcmp(name, "microsteps")) {
    if (emit) respondKeyValueU32V2("microsteps", fineMicrostepsV2);
    return true;
  }
  if (!strcmp(name, "intpol")) {
    if (emit) {
      const uint32_t chop = driver.CHOPCONF();
      respondKeyValueBoolV2("intpol", intpolFromChopconfV2(chop));
    }
    return true;
  }
  if (!strcmp(name, "chopconf")) {
    if (emit) respondKeyValueHexV2("chopconf", driver.CHOPCONF());
    return true;
  }
  if (!strcmp(name, "stealth")) {
    if (emit) respondKeyValueBoolV2("stealth", !driver.en_spreadCycle());
    return true;
  }
  return false;
}

static bool applySetParamV2(const char* name, const char* value, const char** errorKey) {
  if (!name || !value) {
    if (errorKey) *errorKey = "bad_param";
    return false;
  }
  if (!strcmp(name, "ihold")) {
    long mA = 0;
    if (!parseLongV2(value, &mA)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (mA < 0 || mA > 2000) { if (errorKey) *errorKey = "range"; return false; }
    driver.ihold(calcHoldCurrentScaleFromMaV2((uint16_t)mA));
    return true;
  }
  if (!strcmp(name, "irun")) {
    long mA = 0;
    if (!parseLongV2(value, &mA)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (mA < 0 || mA > 2000) { if (errorKey) *errorKey = "range"; return false; }
    driver.irun(calcRunCurrentScaleFromMaV2((uint16_t)mA));
    return true;
  }
  if (!strcmp(name, "tpowerdown")) {
    long ticks = 0;
    if (!parseLongV2(value, &ticks)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (ticks < 0 || ticks > 255) { if (errorKey) *errorKey = "range"; return false; }
    driver.TPOWERDOWN((uint8_t)ticks);
    return true;
  }
  if (!strcmp(name, "tpwmthrs")) {
    uint32_t thrs = 0;
    if (!parseU32V2(value, &thrs)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (thrs > TPWMTHRS_MAX_V2) { if (errorKey) *errorKey = "range"; return false; }
    driver.TPWMTHRS(thrs);
    return true;
  }
  if (!strcmp(name, "sgthrs")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v < 0 || v > 255) { if (errorKey) *errorKey = "range"; return false; }
    driver.SGTHRS((uint8_t)v);
    return true;
  }
  if (!strcmp(name, "microsteps")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v < 1 || v > 256) { if (errorKey) *errorKey = "range"; return false; }
    if (!isMicrostepsAllowedV2((uint16_t)v)) { if (errorKey) *errorKey = "invalid_microsteps"; return false; }
    if (!applyDriverMicrostepsV2((uint16_t)v)) {
      if (errorKey) *errorKey = "driver_fault";
      return false;
    }
    fineMicrostepsV2 = (uint16_t)v;
    ledStateV2.microsteps = (uint16_t)v;
    ledStateV2.stepColorCycle = 200UL * (uint32_t)fineMicrostepsV2;
    ledStateV2.lastStepForColor = LONG_MIN;
    return true;
  }
  if (!strcmp(name, "intpol")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v != 0 && v != 1) { if (errorKey) *errorKey = "invalid_bool"; return false; }
    driver.intpol(v != 0);
    return true;
  }
  if (!strcmp(name, "stealth")) {
    long v = 0;
    if (!parseLongV2(value, &v)) { if (errorKey) *errorKey = "bad_value"; return false; }
    if (v != 0 && v != 1) { if (errorKey) *errorKey = "invalid_bool"; return false; }
    const bool stealth = (v != 0);
    driver.en_spreadCycle(!stealth);
    driver.pwm_autoscale(true);
    return true;
  }
  if (errorKey) *errorKey = "unknown_param";
  return false;
}

static bool setSpeedSpsV2(long sps, const char** errorKey) {
  if (sps < 0) { if (errorKey) *errorKey = "range"; return false; }
  if (sps > 40000) { if (errorKey) *errorKey = "range"; return false; }
  runV2.speedSps = (float)sps;
  return true;
}

static bool setAccelStepsPerUsV2(long accel, const char** errorKey) {
  if (accel < 0) { if (errorKey) *errorKey = "range"; return false; }
  if (accel > 100000) { if (errorKey) *errorKey = "range"; return false; }
  runV2.accelStepsPerUs = (float)accel / 1000000.0f;
  return true;
}

static void setDeltaV2(long delta) {
  runV2.target = stepPosition + delta;
  runV2.hasTarget = true;
}

static void completeTargetV2() {
  runV2.hasTarget = false;
  runV2.running = false;
  runV2.stopRequested = false;
  runV2.actualSpeedSps = 0.0f;
  runV2.desiredSpeedSps = 0.0f;
  runV2.nextStepUs = 0;
  runV2.lastStepperUs = 0;
  runV2.stepAcc = 0.0f;
}

static inline uint32_t getSafetySpeedLimitV2() {
  if (safetyStateV2 == SAFETY_DERATED_V2) return DRIVER_DERATED_SPEED_LIMIT_SPS;
  if (safetyStateV2 == SAFETY_STOPPING_V2 ||
      safetyStateV2 == SAFETY_SHUTDOWN_V2) return 0;
  return DRIVER_NORMAL_SPEED_LIMIT_SPS;
}

static MotionPhaseCodeV2 getPhaseCodeV2() {
  if (!runV2.enabled) return MOTION_PHASE_IDLE_V2;
  if (runV2.actualSpeedSps <= 0.0f) return MOTION_PHASE_HOLD_V2;
  if (runV2.desiredSpeedSps <= 0.0f) return MOTION_PHASE_DECEL_V2;
  if (runV2.actualSpeedSps < runV2.desiredSpeedSps) return MOTION_PHASE_ACCEL_V2;
  if (runV2.actualSpeedSps > runV2.desiredSpeedSps) return MOTION_PHASE_DECEL_V2;
  return MOTION_PHASE_RUN_V2;
}

static const char* getPhaseV2() {
  switch (getPhaseCodeV2()) {
    case MOTION_PHASE_IDLE_V2:
      return PHASE_IDLE_V2;
    case MOTION_PHASE_HOLD_V2:
      return PHASE_HOLD_V2;
    case MOTION_PHASE_ACCEL_V2:
      return PHASE_ACCEL_V2;
    case MOTION_PHASE_RUN_V2:
      return PHASE_RUN_V2;
    case MOTION_PHASE_DECEL_V2:
      return PHASE_DECEL_V2;
  }
  return PHASE_IDLE_V2;
}

static const char* getModeV2() {
  return runV2.freeRideMode ? MODE_FREE_RIDE_V2 : MODE_TARGET_V2;
}

static void updateMotionStateV2() {
  const uint32_t nowUs = micros();

  if (runV2.lastUpdateUs == 0) {
    runV2.lastUpdateUs = nowUs;
    return;
  }
  const uint32_t dtUs = nowUs - runV2.lastUpdateUs;
  if (dtUs == 0) return;
  runV2.lastUpdateUs = nowUs;

  if (!runV2.enabled) {
    runV2.actualSpeedSps = 0.0f;
    runV2.desiredSpeedSps = 0.0f;
    runV2.running = false;
    runV2.stopRequested = false;
    return;
  }

  if (runV2.running && runV2.hasTarget && !runV2.freeRideMode) {
    const long delta = runV2.target - getPosition();
    if (delta == 0) {
      completeTargetV2();
      return;
    }
    setDirV2(delta < 0);
  }

  float desired = 0.0f;
  if (runV2.stopRequested) {
    desired = 0.0f;
  } else if (runV2.running) {
    desired = runV2.speedSps;
  }

  const float safetySpeedLimit = (float)getSafetySpeedLimitV2();
  if (desired > safetySpeedLimit) desired = safetySpeedLimit;

  if (runV2.hasTarget && !runV2.freeRideMode && desired > 0.0f && runV2.accelStepsPerUs > 0.0f) {
    const long delta = runV2.target - getPosition();
    const float accelSps2 = runV2.accelStepsPerUs * 1000000.0f;
    const float stoppingDistance = (runV2.actualSpeedSps * runV2.actualSpeedSps) / (2.0f * accelSps2);
    if ((float)labs(delta) <= stoppingDistance) {
      desired = 0.0f;
    }
  }

  runV2.desiredSpeedSps = desired;
  if (runV2.accelStepsPerUs <= 0.0f) {
    runV2.actualSpeedSps = desired;
  } else {
    const float deltaSpeed = runV2.accelStepsPerUs * (float)dtUs;
    if (runV2.actualSpeedSps < desired) {
      runV2.actualSpeedSps += deltaSpeed;
      if (runV2.actualSpeedSps > desired) runV2.actualSpeedSps = desired;
    } else if (runV2.actualSpeedSps > desired) {
      runV2.actualSpeedSps -= deltaSpeed;
      if (runV2.actualSpeedSps < desired) runV2.actualSpeedSps = desired;
    }
  }

  if (runV2.stopRequested && runV2.actualSpeedSps <= 0.0f) {
    runV2.stopRequested = false;
    runV2.running = false;
    runV2.actualSpeedSps = 0.0f;
    runV2.desiredSpeedSps = 0.0f;
  }

  profiler.updateMotionState += micros() - nowUs;
  profiler.updateMotionState /= 2.;

}

// ---------- TMC2209 link diagnostics ----------
// This command separated the original open wire from a silent chip and remains useful
// as a baud/address diagnostic after the repair (DEC_PROTOCOL.md §13).
//
// Three numbers per baud rate, and it is the *first* that is new:
//
//   raw   bytes seen on the RX pin after a hand-built read datagram is sent. The link
//         is single-wire: TX reaches PDN_UART through a resistor and RX sits on the
//         same node, so the MCU always hears its own 4 bytes back if the two pins are
//         connected at all. 0 = RX hears nothing, ever. 4 = only the echo, i.e. the
//         wiring is fine and the *chip* is silent. 12 = echo plus an 8-byte reply.
//         No other probe here separates "dead wire" from "dead chip".
//   ver   TMCStepper's VERSION field, 0x21 on a healthy TMC2209.
//   ifcnt how much the chip's own write counter moved across one harmless write. It
//         is the only proof of a *write* that does not involve turning the shaft and
//         asking a human to count revolutions.
static const uint32_t TMC_SCAN_BAUDS[] PROGMEM = {4800, 9600, 19200, 38400, 57600, 115200};
static const uint8_t TMC_SCAN_COUNT = 6;

// CRC-8 of the TMC UART datagram (poly 0x07, LSB first) — the datasheet's
// swuart_calcCRC. Written out here because TMCStepper keeps its copy protected.
static uint8_t tmcCrc8V2(const uint8_t* data, uint8_t len) {
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

// Send one read request by hand and count whatever comes back, without letting
// TMCStepper consume it. Deliberately does not check the answer: the count alone
// is the wiring diagnosis.
static uint8_t tmcRawProbeV2(uint32_t baud) {
  TMCSerial.end();
  TMCSerial.begin(baud);
  delay(2);
  while (TMCSerial.available()) TMCSerial.read();

  uint8_t datagram[4] = {0x05, DRIVER_ADDRESS, 0x06 | 0x00, 0x00};  // 0x06 = IOIN, read
  datagram[3] = tmcCrc8V2(datagram, 3);
  for (uint8_t i = 0; i < 4; i++) TMCSerial.write(datagram[i]);

  // Long enough for 12 bytes even at 4800 baud (25 ms) plus the chip's turnaround.
  const uint32_t deadline = millis() + 40;
  uint8_t count = 0;
  while ((int32_t)(millis() - deadline) < 0) {
    if (TMCSerial.available()) {
      TMCSerial.read();
      if (count < 255) count++;
    }
  }
  return count;
}

static void tmcScanV2() {
  respondStartV2(true);
  for (uint8_t i = 0; i < TMC_SCAN_COUNT; i++) {
    const uint32_t baud = pgm_read_dword(&TMC_SCAN_BAUDS[i]);
    const uint8_t raw = tmcRawProbeV2(baud);

    // The first datagram after a speed change is the one the chip measures its
    // baud from, so its answer is not trusted; ask again for the reported value.
    driver.IOIN();
    const uint8_t ver = driver.version();
    const uint8_t before = driver.IFCNT();
    driver.TPOWERDOWN(20);
    const uint8_t delta = (uint8_t)(driver.IFCNT() - before);

    outAppendCharV2('b');
    outAppendNumU32V2(baud);
    outAppendCharV2('=');
    outAppendNumU32V2(raw);
    outAppendCharV2('/');
    outAppendNumU32V2(ver);
    outAppendCharV2('/');
    outAppendNumU32V2(delta);
    outAppendCharV2(';');
  }
  TMCSerial.end();
  TMCSerial.begin(TMC_BAUD);
  respondEndV2();
}

// Continuity between the two UART pins, measured as plain DC and without going
// through AltSoftSerial at all. It distinguishes an open wire from a protocol fault
// because it uses neither the library nor the chip.
//
// The two pads on the driver module join at its single PDN_UART node (one of them
// through the module's series resistor), so on a correctly wired board pin 8 must
// follow pin 8's neighbour: drive pin 9 low and pin 8 goes low, through 1k against a
// ~40k internal pull-up. If pin 8 stays high no matter what pin 9 does, the two pins
// share no copper and no amount of firmware will fix it.
static void tmcWireV2() {
  TMCSerial.end();

  pinMode(TMC_TX_PIN, OUTPUT);
  pinMode(TMC_RX_PIN, INPUT_PULLUP);
  digitalWrite(TMC_TX_PIN, HIGH);
  delay(2);
  const uint8_t rxAtTx1 = (uint8_t)digitalRead(TMC_RX_PIN);
  digitalWrite(TMC_TX_PIN, LOW);
  delay(2);
  const uint8_t rxAtTx0 = (uint8_t)digitalRead(TMC_RX_PIN);
  digitalWrite(TMC_TX_PIN, HIGH);
  delay(2);

  // The mirror direction, in case exactly one of the two wires is off.
  pinMode(TMC_RX_PIN, OUTPUT);
  pinMode(TMC_TX_PIN, INPUT_PULLUP);
  digitalWrite(TMC_RX_PIN, HIGH);
  delay(2);
  const uint8_t txAtRx1 = (uint8_t)digitalRead(TMC_TX_PIN);
  digitalWrite(TMC_RX_PIN, LOW);
  delay(2);
  const uint8_t txAtRx0 = (uint8_t)digitalRead(TMC_TX_PIN);

  // Both released: an idle UART line is held high by the chip, so a floating read
  // that comes back low says nothing is holding it at all.
  pinMode(TMC_RX_PIN, INPUT);
  pinMode(TMC_TX_PIN, INPUT);
  delay(5);
  const uint8_t rxHiZ = (uint8_t)digitalRead(TMC_RX_PIN);
  const uint8_t txHiZ = (uint8_t)digitalRead(TMC_TX_PIN);

  respondStartV2(true);
  respondKeyValueU32V2("rx_hi", rxAtTx1);
  respondKeyValueU32V2("rx_lo", rxAtTx0);
  respondKeyValueU32V2("tx_hi", txAtRx1);
  respondKeyValueU32V2("tx_lo", txAtRx0);
  respondKeyValueU32V2("rx_z", rxHiZ);
  respondKeyValueU32V2("tx_z", txHiZ);
  // The one line that reads as a verdict: connected means RX followed TX down.
  respondKeyValueBoolV2("linked", rxAtTx1 == 1 && rxAtTx0 == 0);
  respondEndV2();

  TMCSerial.begin(TMC_BAUD);
}

static void handleLineV2(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;

  char* cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "sensor")) {
    if (strtok(NULL, " \t")) { respondErrorV2("bad_value"); return; }
    // No physical sensor driver is configured yet. Never fabricate measurements.
    respondStartV2(true);
    respondKeyValueLongV2(F("sensor_flags"), 0);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "status")) {
    respondStartV2(true);
    respondKeyValueBoolV2(F("initialised"), v2Initialized);
    respondKeyValueBoolV2(F("enabled"), runV2.enabled);
    respondKeyValueStrV2(F("mode"), getModeV2());
    respondKeyValueLongV2(F("position"), getPosition());
    respondKeyValueStrV2(F("phase"), getPhaseV2());
    respondKeyValueLongV2(F("target"), runV2.target);
    respondKeyValueBoolV2(F("target_set"), runV2.hasTarget);
    respondKeyValueFloatV2(F("speed"), runV2.speedSps, 2);
    respondKeyValueFloatV2(F("actual_speed"), runV2.actualSpeedSps, 2);
    respondKeyValueFloatV2(F("accel_per_s"), runV2.accelStepsPerUs * 1000000., 2);
    respondKeyValueFloatV2(F("power_v"), ledStateV2.supplyVoltageV, 2);
    respondKeyValueU32V2(F("tx_overflow"), txOverflowCountV2);
    respondKeyValueU32V2(F("drv_flags"), driverFlagsV2);
    const __FlashStringHelper* safety = F("normal");
    if (safetyStateV2 == SAFETY_DERATED_V2) safety = F("derated");
    else if (safetyStateV2 == SAFETY_STOPPING_V2) safety = F("stopping");
    else if (safetyStateV2 == SAFETY_SHUTDOWN_V2) safety = F("shutdown");
    respondKeyValueStrV2(F("safety"), safety);
    respondKeyValueU32V2(F("mres"), ledStateV2.microsteps);
    respondKeyValueU32V2(F("fine_mres"), fineMicrostepsV2);
    respondKeyValueU32V2(F("limit"), getSafetySpeedLimitV2());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "tmc_scan")) {
    tmcScanV2();
    return;
  }

  if (!strcmp(cmd, "tmc_wire")) {
    tmcWireV2();
    return;
  }

  if (!strcmp(cmd, "full_status") || !strcmp(cmd, "driver_status")) {
    const uint32_t drvStatus = driver.DRV_STATUS();
    const uint8_t currentScale = (uint8_t)((drvStatus >> 16) & 0x1FUL);

    respondStartV2(true);
    respondKeyValueHexV2(F("ioin"), driver.IOIN());
    respondKeyValueU32V2(F("ifcnt"), driver.IFCNT());
    respondKeyValueHexV2(F("gconf"), driver.GCONF());
    respondKeyValueHexV2(F("chopconf"), driver.CHOPCONF());
    respondKeyValueHexV2(F("drv_status"), drvStatus);
    respondKeyValueU32V2(F("sg_result"), driver.SG_RESULT());
    respondKeyValueU32V2(F("cs_actual"), currentScale);
    respondKeyValueU32V2(F("current_ma_est"), driver.cs2rms(currentScale));
    respondKeyValueHexV2(F("tstep"), driver.TSTEP());
    respondKeyValueU32V2(F("mscnt"), driver.MSCNT());
    respondKeyValueHexV2(F("pwm_scale"), driver.PWM_SCALE());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "get")) {
    char* name = strtok(nullptr, " \t");
    if (!name) { respondErrorV2("missing_param"); return; }
    name = stripParamPrefixV2(name);
    if (!appendParamValueV2(name, false)) { respondErrorV2("unknown_param"); return; }
    respondStartV2(true);
    appendParamValueV2(name, true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "set")) {
    char* token = strtok(nullptr, " \t");
    char* name = nullptr;
    char* value = nullptr;
    while (token) {
      if (name) { respondErrorV2("single_param"); return; }
      if (!parseNameValueTokenV2(token, &name, &value)) { respondErrorV2("bad_param"); return; }
      token = strtok(nullptr, " \t");
    }
    if (!name) { respondErrorV2("missing_param"); return; }
    const char* errorKey = nullptr;
    if (!applySetParamV2(name, value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "set_failed"); return; }
    respondStartV2(true);
    appendParamValueV2(name, true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "position")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setPosition(value);
    respondStartV2(true);
    respondKeyValueLongV2("position", getPosition());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "enabled")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const bool enable = (value != 0);
    if (enable && (safetyStateV2 == SAFETY_STOPPING_V2 || safetyStateV2 == SAFETY_SHUTDOWN_V2)) {
      respondErrorV2(F("driver_fault"));
      return;
    }
    setEnableV2(enable);
    if (!enable) {
      acknowledgeStallV2();
      runV2.running = false;
      runV2.stopRequested = false;
      runV2.actualSpeedSps = 0.0f;
      runV2.desiredSpeedSps = 0.0f;
      runV2.nextStepUs = 0;
      runV2.lastStepperUs = 0;
      runV2.stepAcc = 0.0f;
    }
    respondStartV2(true);
    respondKeyValueBoolV2("enabled", runV2.enabled);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "direction")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setDirV2(value != 0);
    respondStartV2(true);
    respondKeyValueBoolV2("direction", runV2.dir);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "speed")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const char* errorKey = nullptr;
    if (!setSpeedSpsV2(value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "range"); return; }
    respondStartV2(true);
    respondKeyValueFloatV2("speed", runV2.speedSps, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "acceleration")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    const char* errorKey = nullptr;
    if (!setAccelStepsPerUsV2(value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "range"); return; }
    respondStartV2(true);
    respondKeyValueFloatV2("accel_per_s", runV2.accelStepsPerUs * 1000000.0f, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "delta")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setDeltaV2(value);
    // if (runV2.target == getPosition()) {
    //   completeTargetV2();
    // }
    respondStartV2(true);
    respondKeyValueLongV2("delta", value);
    respondKeyValueLongV2("target", runV2.target);
    respondKeyValueBoolV2("target_set", runV2.hasTarget);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "mode")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { respondErrorV2("bad_value"); return; }
    if (strtok(nullptr, " \t")) { respondErrorV2("single_param"); return; }
    if (!strcmp(a, MODE_FREE_RIDE_V2)) {
      runV2.freeRideMode = true;
    } else if (!strcmp(a, MODE_TARGET_V2)) {
      runV2.freeRideMode = false;
    } else {
      respondErrorV2("bad_value");
      return;
    }
    respondStartV2(true);
    respondKeyValueStrV2("mode", getModeV2());
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "run")) {
    if (safetyStateV2 == SAFETY_STOPPING_V2 || safetyStateV2 == SAFETY_SHUTDOWN_V2) {
      respondErrorV2(F("driver_fault"));
      return;
    }
    runV2.running = true;
    runV2.stopRequested = false;
    if (!runV2.enabled) setEnableV2(true);
    runV2.lastUpdateUs = micros();
    runV2.lastStepperUs = 0;
    runV2.stepAcc = 0.0f;
    respondStartV2(true);
    respondKeyValueBoolV2("running", true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "stop")) {
    runV2.stopRequested = true;
    runV2.running = true;
    runV2.lastUpdateUs = micros();
    // keep accumulated fractional steps; only reset time base
    runV2.lastStepperUs = 0;
    respondStartV2(true);
    respondKeyValueBoolV2("stopping", true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "profile")) {
    respondStartV2(true);
    respondKeyValueFloatV2("serial", profiler.serial, 2);
    respondKeyValueFloatV2("serialLastProcessed", profiler.serialLastProcessed, 2);
    respondKeyValueFloatV2("stepper", profiler.stepper, 2);
    respondKeyValueFloatV2("stepperHigh", profiler.stepperHigh, 2);
    respondKeyValueFloatV2("stepperLow", profiler.stepperLow, 2);
    respondKeyValueFloatV2("updateMotionState", profiler.updateMotionState, 2);
    respondEndV2();
    return;
  }

  respondErrorV2("unknown_cmd");
}

static void handleFrameV3(char* s, uint8_t len) {
  const FrameV3 frame = frameParseV3(s, len);
  frameSeqV3 = frame.seq;
  if (frame.error != FRAME_OK_V3) {
    respondFrameErrorV3(frame.seq, frame.error);
    return;
  }

  const uint8_t op = frame.op;
  const uint8_t seq = frame.seq;
  const int32_t argI32 = frame.len >= 4 ? framePayloadI32V3(frame.payload) : 0;

  switch (op) {
    case OP_SENSOR_V3: {
      if (frame.len != 0) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      // flags=0: absent device. Remaining reserved fields are not measurements.
      const RawSensorV3 sensor = {};
      outLineStartV2 = outWriteV2;
      outLineFailedV2 = false;
      frameWriteSensorV3(frameWriterV3, seq, sensor);
      frameEndV3();
      return;
    }

    case OP_HELLO_V3:
      frameStartV3(op, seq, 2);
      frameWriterV3.byte(FRAME_PROTOCOL_VERSION_V3);
      frameWriterV3.byte(FRAME_FIRMWARE_BUILD_V3);
      frameEndV3();
      return;

    case OP_STATUS_V3: {
      uint8_t flags = 0;
      if (v2Initialized) flags |= FRAME_FLAG_INITIALISED_V3;
      if (runV2.enabled) flags |= FRAME_FLAG_ENABLED_V3;
      if (runV2.freeRideMode) flags |= FRAME_FLAG_FREE_RIDE_V3;
      if (runV2.hasTarget) flags |= FRAME_FLAG_TARGET_SET_V3;
      outLineStartV2 = outWriteV2;
      outLineFailedV2 = false;
      frameWriteStatusV3(frameWriterV3, seq, flags, (uint8_t)getPhaseCodeV2(),
                         (int32_t)getPosition(), (int32_t)runV2.target,
                         runV2.speedSps, runV2.actualSpeedSps,
                         runV2.accelStepsPerUs * 1000000.0f,
                         ledStateV2.supplyVoltageV, txOverflowCountV2,
                         driverFlagsV2, (uint8_t)safetyStateV2,
                         ledStateV2.microsteps, fineMicrostepsV2,
                         getSafetySpeedLimitV2(),
                         safetyEventsV2);
      frameEndV3();
      return;
    }

    case OP_POSITION_V3:
      if (frame.len != 4) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      setPosition((long)argI32);
      frameStartV3(op, seq, 4);
      frameWriterV3.i32((int32_t)getPosition());
      frameEndV3();
      return;

    case OP_SPEED_V3: {
      if (frame.len != 4) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      const char* errorKey = nullptr;
      if (!setSpeedSpsV2((long)argI32, &errorKey)) { respondFrameErrorV3(seq, ERR_RANGE_V3); return; }
      frameStartV3(op, seq, 4);
      frameWriterV3.u32(frameCentiV3(runV2.speedSps));
      frameEndV3();
      return;
    }

    case OP_ACCELERATION_V3: {
      if (frame.len != 4) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      const char* errorKey = nullptr;
      if (!setAccelStepsPerUsV2((long)argI32, &errorKey)) { respondFrameErrorV3(seq, ERR_RANGE_V3); return; }
      frameStartV3(op, seq, 4);
      frameWriterV3.u32(frameCentiV3(runV2.accelStepsPerUs * 1000000.0f));
      frameEndV3();
      return;
    }

    case OP_DIRECTION_V3:
      if (frame.len != 1) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      setDirV2(frame.payload[0] != 0);
      frameStartV3(op, seq, 1);
      frameWriterV3.byte(runV2.dir ? 1 : 0);
      frameEndV3();
      return;

    case OP_ENABLED_V3: {
      if (frame.len != 1) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      const bool enable = frame.payload[0] != 0;
      if (enable && (safetyStateV2 == SAFETY_STOPPING_V2 || safetyStateV2 == SAFETY_SHUTDOWN_V2)) {
        respondFrameErrorV3(seq, ERR_DRIVER_FAULT_V3);
        return;
      }
      setEnableV2(enable);
      if (!enable) {
        acknowledgeStallV2();
        runV2.running = false;
        runV2.stopRequested = false;
        runV2.actualSpeedSps = 0.0f;
        runV2.desiredSpeedSps = 0.0f;
        runV2.nextStepUs = 0;
        runV2.lastStepperUs = 0;
        runV2.stepAcc = 0.0f;
      }
      frameStartV3(op, seq, 1);
      frameWriterV3.byte(runV2.enabled ? 1 : 0);
      frameEndV3();
      return;
    }

    case OP_DELTA_V3:
      if (frame.len != 4) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      setDeltaV2((long)argI32);
      frameStartV3(op, seq, 9);
      frameWriterV3.i32(argI32);
      frameWriterV3.i32((int32_t)runV2.target);
      frameWriterV3.byte(runV2.hasTarget ? 1 : 0);
      frameEndV3();
      return;

    case OP_MODE_V3:
      if (frame.len != 1 || frame.payload[0] > 1) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      runV2.freeRideMode = (frame.payload[0] == 1);
      frameStartV3(op, seq, 1);
      frameWriterV3.byte(runV2.freeRideMode ? 1 : 0);
      frameEndV3();
      return;

    case OP_MICROSTEPS_V3: {
      // No payload reads the parameter, two bytes write it.
      if (frame.len != 0 && frame.len != 2) { respondFrameErrorV3(seq, ERR_BAD_VALUE_V3); return; }
      if (frame.len == 2) {
        const uint16_t requested = framePayloadU16V3(frame.payload);
        if (requested < 1 || requested > 256) { respondFrameErrorV3(seq, ERR_RANGE_V3); return; }
        if (!isMicrostepsAllowedV2(requested)) { respondFrameErrorV3(seq, ERR_INVALID_MICROSTEPS_V3); return; }
        if (!applyDriverMicrostepsV2(requested)) {
          respondFrameErrorV3(seq, ERR_DRIVER_FAULT_V3);
          return;
        }
        fineMicrostepsV2 = requested;
        ledStateV2.microsteps = requested;
        ledStateV2.stepColorCycle = 200UL * (uint32_t)fineMicrostepsV2;
        ledStateV2.lastStepForColor = LONG_MIN;
      }
      frameStartV3(op, seq, 2);
      frameWriterV3.u16(fineMicrostepsV2);
      frameEndV3();
      return;
    }

    case OP_RUN_V3:
      if (safetyStateV2 == SAFETY_STOPPING_V2 || safetyStateV2 == SAFETY_SHUTDOWN_V2) {
        respondFrameErrorV3(seq, ERR_DRIVER_FAULT_V3);
        return;
      }
      runV2.running = true;
      runV2.stopRequested = false;
      if (!runV2.enabled) setEnableV2(true);
      runV2.lastUpdateUs = micros();
      runV2.lastStepperUs = 0;
      runV2.stepAcc = 0.0f;
      frameStartV3(op, seq, 1);
      frameWriterV3.byte(1);
      frameEndV3();
      return;

    case OP_STOP_V3:
      runV2.stopRequested = true;
      runV2.running = true;
      runV2.lastUpdateUs = micros();
      runV2.lastStepperUs = 0;
      frameStartV3(op, seq, 1);
      frameWriterV3.byte(1);
      frameEndV3();
      return;

    default:
      respondFrameErrorV3(seq, ERR_UNKNOWN_CMD_V3);
      return;
  }
}

void serviceSerialv2() {
  // Non-blocking block read + scan for '\n'. Avoids per-char Stream parsing overhead,
  // but still correctly detects line endings.
  static uint8_t tmp[32];

  while (Serial.available()) {
    const int avail = Serial.available();
    if (avail <= 0) break;

    size_t toRead = (size_t)avail;
    if (toRead > sizeof(tmp)) toRead = sizeof(tmp);

    size_t n = Serial.readBytes(tmp, toRead);
    for (size_t i = 0; i < n; i++) {
      char c = (char)tmp[i];
      if (c == '\r') continue;

      if (c == '\n') {
        const uint8_t lineLen = lineLenV2;
        lineBufV2[lineLen] = 0;
        const uint32_t start = micros();
        // A marker anywhere in the line means a frame: not only at position 0,
        // because the FIFO can leave a stump in front of it. No line command
        // contains '#', so the two dialects cannot be confused for one another.
        //
        // memchr, not strchr: the line is `lineLen` bytes of arbitrary wire data,
        // not a C string. A stray 0x00 anywhere in front of the marker used to end
        // the search early, so the frame behind it was handed to the line parser,
        // which saw an empty command and answered nothing at all. The v2 branch
        // below still takes a NUL-terminated buffer, and that is left alone: the
        // line protocol has no framing to salvage, and its NUL behaviour is
        // documented (DEC_PROTOCOL.md §5.8).
        if (memchr(lineBufV2, '#', lineLen) != nullptr) {
          handleFrameV3(lineBufV2, lineLen);
        } else {
          handleLineV2(lineBufV2);
        }
        profiler.serialLastProcessed = micros() - start;
        lineLenV2 = 0;
        continue;
      }

      if (lineLenV2 < sizeof(lineBufV2) - 1) {
        lineBufV2[lineLenV2++] = c;
      } else {
        // Overflow without newline: drop and resync.
        lineLenV2 = 0;
      }
    }

  }

  // Drain in a batch, but never past the free room in the hardware TX buffer, so that
  // Serial.write() cannot block. The extra ceiling keeps one loop() iteration short: the
  // step accumulator is clamped in serviceStepperv2(), so a stretched iteration means
  // steps that are lost, not steps that are merely late.
  int txBudget = Serial.availableForWrite();
  if (txBudget > TX_DRAIN_MAX_V2) txBudget = TX_DRAIN_MAX_V2;
  char c = 0;
  while (txBudget > 0 && outPopV2(&c)) {
    Serial.write((uint8_t)c);
    txBudget--;
  }
}

void serviceStepperv2() {
  const uint32_t nowUs = micros();

  // Finish pulse (STEP LOW)
  if (runV2.stepHigh) {
    if ((int32_t)(nowUs - runV2.stepHighUntilUs) >= 0) {
      digitalWrite(STEP_PIN, LOW);
      runV2.stepHigh = false;
    }
    return;
  }

  // Update motion profile (desired/actual speed)
  updateMotionStateV2();

  if (!runV2.enabled || runV2.actualSpeedSps <= 0.0f) {
    // When stopped/disabled, reset phase accumulator and time base.
    runV2.stepAcc = 0.0f;
    runV2.lastStepperUs = nowUs;
    return;
  }

  // Time base for accumulator
  if (runV2.lastStepperUs == 0) {
    runV2.lastStepperUs = nowUs;
    return;
  }

  const uint32_t dtUs = nowUs - runV2.lastStepperUs;
  runV2.lastStepperUs = nowUs;
  if (dtUs == 0) return;

  uint16_t logicalStepsPerPulse = fineMicrostepsV2 / ledStateV2.microsteps;
  if (logicalStepsPerPulse == 0) logicalStepsPerPulse = 1;

  // Speed and position stay in the configured fine-resolution units. In coarse
  // mode one physical STEP pulse advances several of those logical units.
  runV2.stepAcc +=
      runV2.actualSpeedSps * ((float)dtUs / 1000000.0f) / (float)logicalStepsPerPulse;

  // Clamp accumulator to avoid runaway after long stalls
  if (runV2.stepAcc > 8.0f) runV2.stepAcc = 8.0f;

  // Emit at most ONE step per call because we need a HIGH->LOW pulse across calls
  if (runV2.stepAcc < 1.0f) return;

  const uint32_t tUs = micros();

  digitalWrite(STEP_PIN, HIGH);
  runV2.stepHigh = true;
  runV2.stepHighUntilUs = tUs + runV2.pulseWidthUs;
  runV2.lastStepUs = tUs;

  // Consume one whole step from accumulator
  runV2.stepAcc -= 1.0f;

  // Update position
  stepPosition += runV2.dir
      ? STEP_NEGATIVE * (long)logicalStepsPerPulse
      : STEP_POSITIVE * (long)logicalStepsPerPulse;

  // Target completion check (same logic as before)
  if (runV2.running && runV2.hasTarget && !runV2.freeRideMode) {
    const long pos = getPosition();
    if ((runV2.dir && pos <= runV2.target) || (!runV2.dir && pos >= runV2.target)) {
      setPosition(runV2.target);
      completeTargetV2();
    }
  }
}

void loop() {
  long start = micros();
  serviceSerialv2();
  profiler.serial += micros() - start;
  profiler.serial /= 2.;

  if (!runV2.stepHigh) {
    uint16_t wantedMicrosteps = fineMicrostepsV2;
    if (runV2.enabled && runV2.running && runV2.hasTarget &&
        !runV2.freeRideMode && !runV2.stopRequested && fineMicrostepsV2 > 1) {
      const long delta = runV2.target - getPosition();
      const long remaining = labs(delta);
      if (delta != 0) setDirV2(delta < 0);

      float stoppingDistance = 0.0f;
      if (runV2.accelStepsPerUs > 0.0f) {
        const float accelSps2 = runV2.accelStepsPerUs * 1000000.0f;
        stoppingDistance =
            (runV2.actualSpeedSps * runV2.actualSpeedSps) / (2.0f * accelSps2);
      }
      const float fineApproach =
          stoppingDistance + (float)fineMicrostepsV2 * (float)FINE_APPROACH_FULL_STEPS;
      if (ledStateV2.microsteps == 1) {
        if ((float)remaining > fineApproach) wantedMicrosteps = 1;
      } else if (runV2.actualSpeedSps <= 0.0f && (float)remaining > fineApproach) {
        wantedMicrosteps = 1;
      }
    }

    if (wantedMicrosteps != ledStateV2.microsteps) {
      const uint16_t previousMicrosteps = ledStateV2.microsteps;
      const uint16_t before = driver.MSCNT();
      if (applyDriverMicrostepsV2(wantedMicrosteps)) {
        const uint16_t after = driver.MSCNT();
        int16_t phaseDelta = (int16_t)after - (int16_t)before;
        if (phaseDelta > 512) phaseDelta -= 1024;
        else if (phaseDelta < -512) phaseDelta += 1024;
        const long scaled = (long)phaseDelta * (long)fineMicrostepsV2;
        const long correction =
            scaled >= 0 ? (scaled + 128L) / 256L : (scaled - 128L) / 256L;
        stepPosition += correction;
        const uint16_t previousStride = fineMicrostepsV2 / previousMicrosteps;
        const uint16_t appliedStride = fineMicrostepsV2 / wantedMicrosteps;
        runV2.stepAcc *= (float)previousStride / (float)appliedStride;
        if (runV2.stepAcc > 8.0f) runV2.stepAcc = 8.0f;
        ledStateV2.microsteps = wantedMicrosteps;
        ledStateV2.lastStepForColor = LONG_MIN;
      } else {
        driverFlagsV2 |= DRIVER_FLAG_UART;
        if (safetyStateV2 != SAFETY_STOPPING_V2 && safetyStateV2 != SAFETY_SHUTDOWN_V2) {
          if (safetyEventsV2 < UINT16_MAX) safetyEventsV2++;
          safetyStateV2 = SAFETY_STOPPING_V2;
        }
        runV2.stopRequested = true;
        runV2.running = true;
      }
    }
  }

  start = micros();
  serviceStepperv2();
  profiler.stepper += micros() - start;
  profiler.stepper /= 2.;

  const uint32_t nowMs = millis();
  const bool stallEligible =
      runV2.enabled && runV2.running && !runV2.stopRequested &&
      (safetyStateV2 == SAFETY_NORMAL_V2 || safetyStateV2 == SAFETY_DERATED_V2) &&
      getPhaseCodeV2() == MOTION_PHASE_RUN_V2 &&
      runV2.actualSpeedSps >= (float)DRIVER_STALL_MIN_SPEED_SPS;
  if (!stallEligible) {
    driverStallEligibleAtMsV2 = 0;
    driverStallScoreV2 = 0;
  } else if (driverStallEligibleAtMsV2 == 0) {
    driverStallEligibleAtMsV2 = nowMs;
    driverStallScoreV2 = 0;
  } else if (!runV2.stepHigh && !driverStallLatchedV2 &&
             nowMs - driverStallEligibleAtMsV2 >= DRIVER_STALL_ARM_TIME_MS &&
             nowMs - driverSgPolledAtMsV2 >= DRIVER_STALL_POLL_INTERVAL_MS) {
    driverSgPolledAtMsV2 = nowMs;
    driverSgResultV2 = driver.SG_RESULT();

    if (driverSgResultV2 <= DRIVER_STALL_STRONG_SG) {
      const uint8_t room = (uint8_t)(DRIVER_STALL_SCORE_LIMIT - driverStallScoreV2);
      driverStallScoreV2 += room < 3 ? room : 3;
    } else if (driverSgResultV2 <= DRIVER_STALL_LOADED_SG) {
      if (driverStallScoreV2 < DRIVER_STALL_SCORE_LIMIT) driverStallScoreV2++;
    } else if (driverSgResultV2 >= DRIVER_STALL_CLEAR_SG && driverStallScoreV2 > 0) {
      driverStallScoreV2--;
    }

    if (driverStallScoreV2 >= DRIVER_STALL_SCORE_LIMIT) {
      const uint32_t confirmedStatus = driver.DRV_STATUS();
      if (confirmedStatus == 0 || confirmedStatus == 0xFFFFFFFFUL) {
        driverStallScoreV2 = 0;
      } else {
        driverStatusV2 = confirmedStatus;
        driverStallLatchedV2 = true;
        driverFlagsV2 |= DRIVER_FLAG_STALL;
        if (safetyEventsV2 < UINT16_MAX) safetyEventsV2++;
        safetyStateV2 = SAFETY_SHUTDOWN_V2;
        setEnableV2(false);
        runV2.running = false;
        runV2.stopRequested = false;
        runV2.actualSpeedSps = 0.0f;
        runV2.desiredSpeedSps = 0.0f;
        runV2.lastStepperUs = 0;
        runV2.stepAcc = 0.0f;
      }
    }
  }

  if (!runV2.stepHigh &&
      (runV2.enabled || safetyStateV2 != SAFETY_NORMAL_V2 ||
       (driverFlagsV2 & DRIVER_FLAG_UART)) &&
      nowMs - driverPolledAtMsV2 >= DRIVER_POLL_INTERVAL_MS) {
    driverPolledAtMsV2 = nowMs;
    const uint32_t observed = driver.DRV_STATUS();
    if (observed == 0 || observed == 0xFFFFFFFFUL) {
      if (driverUartFailuresV2 < UINT8_MAX) driverUartFailuresV2++;
      if (driverUartFailuresV2 >= DRIVER_UART_FAILURE_LIMIT) {
        driverFlagsV2 |= DRIVER_FLAG_UART;
        driverClearPollsV2 = 0;
        if (safetyStateV2 != SAFETY_STOPPING_V2 && safetyStateV2 != SAFETY_SHUTDOWN_V2) {
          if (safetyEventsV2 < UINT16_MAX) safetyEventsV2++;
          safetyStateV2 = SAFETY_STOPPING_V2;
        }
        runV2.stopRequested = true;
        runV2.running = true;
      }
    } else {
      driverStatusV2 = observed;
      driverUartFailuresV2 = 0;
      driverFlagsV2 = driverStallLatchedV2 ? DRIVER_FLAG_STALL : 0;
      if (observed & 0x01UL) driverFlagsV2 |= DRIVER_FLAG_OTPW;
      if (observed & 0x02UL) driverFlagsV2 |= DRIVER_FLAG_OT;
      if (observed & DRIVER_STATUS_SHORT_MASK) driverFlagsV2 |= DRIVER_FLAG_SHORT;
      if (observed & DRIVER_STATUS_OPEN_LOAD_MASK) driverFlagsV2 |= DRIVER_FLAG_OPEN_LOAD;

      const bool critical =
          (driverFlagsV2 & (DRIVER_FLAG_OT | DRIVER_FLAG_SHORT)) != 0;
      if (critical) {
        driverClearPollsV2 = 0;
        if (safetyStateV2 != SAFETY_SHUTDOWN_V2) {
          if (safetyEventsV2 < UINT16_MAX) safetyEventsV2++;
        }
        safetyStateV2 = SAFETY_SHUTDOWN_V2;
        setEnableV2(false);
        runV2.running = false;
        runV2.stopRequested = false;
        runV2.actualSpeedSps = 0.0f;
        runV2.desiredSpeedSps = 0.0f;
        runV2.lastStepperUs = 0;
        runV2.stepAcc = 0.0f;
      } else if (driverFlagsV2 & DRIVER_FLAG_OTPW) {
        driverClearPollsV2 = 0;
        if (safetyStateV2 == SAFETY_NORMAL_V2) {
          driver.rms_current(DRIVER_DERATED_CURRENT_MA);
          if (safetyEventsV2 < UINT16_MAX) safetyEventsV2++;
          safetyStateV2 = SAFETY_DERATED_V2;
        }
      } else if (safetyStateV2 == SAFETY_DERATED_V2 ||
                 (safetyStateV2 == SAFETY_SHUTDOWN_V2 && !driverStallLatchedV2)) {
        if (driverClearPollsV2 < DRIVER_CLEAR_POLLS) driverClearPollsV2++;
        if (driverClearPollsV2 >= DRIVER_CLEAR_POLLS) {
          driver.rms_current(DRIVER_RUN_CURRENT_MA);
          safetyStateV2 = SAFETY_NORMAL_V2;
          driverClearPollsV2 = 0;
        }
      } else {
        driverClearPollsV2 = 0;
      }
    }
  }

  if (!runV2.stepHigh && safetyStateV2 == SAFETY_STOPPING_V2 &&
      runV2.actualSpeedSps <= 0.0f) {
    setEnableV2(false);
    runV2.running = false;
    runV2.stopRequested = false;
    runV2.desiredSpeedSps = 0.0f;
    safetyStateV2 = SAFETY_SHUTDOWN_V2;
  }

  serviceLedsV2();
}
