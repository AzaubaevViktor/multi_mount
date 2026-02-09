/*
  Nano v3 + TMC2209 + TMCStepper
  Wiring (typical 1-wire UART):
    
    GND common, VM+motor power as usual.

  STEP/DIR/EN to your driver module pins.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include <stdio.h>

// ---------- Pins ----------
static const uint8_t STEP_PIN = 10;
static const uint8_t DIR_PIN  = 3;
static const uint8_t EN_PIN   = 4;   // Enable pin to driver
static const bool    EN_ACTIVE_LOW = true;

static const uint8_t TMC_RX_PIN = 8;
static const uint8_t TMC_TX_PIN = 9;

// ---------- TMC config ----------
static const uint32_t TMC_BAUD = 115200;
// Most SilentStepStick-like modules use 0.11 ohm; check your board to be correct.
static const float R_SENSE = 0.11f;
// Address depends on MS1/MS2 (CFG pins) strapping; often 0b00 if both low.
static const uint8_t DRIVER_ADDRESS = 0b00;

SoftwareSerial TMCSerial(TMC_RX_PIN, TMC_TX_PIN);
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// ---------- Motion state (v2) ----------
struct RunnerV2 {
  bool enabled = false;
  bool dir = false;
  bool running = false;
  bool stopRequested = false;
  bool hasTarget = false;
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

// ---------- V2 formatting ----------
static const uint8_t HEX_WIDTH = 8;
static const uint8_t HEX_PREFIX_LEN = 2;
static const uint8_t HEX_BUF_LEN = HEX_PREFIX_LEN + HEX_WIDTH + 1;

// ---------- Simple line parser ----------
static char lineBufV2[128];
static uint8_t lineLenV2 = 0;

// ---------- Fast TX response buffer ----------
static char outBufV2[256];
static uint16_t outLenV2 = 0;

static inline void outResetV2() { outLenV2 = 0; }

static inline void outAppendCharV2(char c) {
  if (outLenV2 < sizeof(outBufV2) - 1) outBufV2[outLenV2++] = c;
}

static inline void outAppendStrV2(const char* s) {
  if (!s) return;
  while (*s && outLenV2 < sizeof(outBufV2) - 1) outBufV2[outLenV2++] = *s++;
}

static inline void outAppendNumLongV2(long v) {
  char tmp[24];
  int n = snprintf(tmp, sizeof(tmp), "%ld", v);
  if (n > 0) {
    for (int i = 0; i < n && outLenV2 < sizeof(outBufV2) - 1; i++) outBufV2[outLenV2++] = tmp[i];
  }
}

static inline void outAppendNumU32V2(uint32_t v) {
  char tmp[24];
  int n = snprintf(tmp, sizeof(tmp), "%lu", (unsigned long)v);
  if (n > 0) {
    for (int i = 0; i < n && outLenV2 < sizeof(outBufV2) - 1; i++) outBufV2[outLenV2++] = tmp[i];
  }
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
  char tmp[HEX_BUF_LEN];
  snprintf(tmp, sizeof(tmp), "0x%0*lX", HEX_WIDTH, (unsigned long)v);
  outAppendStrV2(tmp);
}

static inline void outFlushLineV2() {
  outAppendCharV2('\n');
  outBufV2[outLenV2] = 0;
  Serial.write((const uint8_t*)outBufV2, outLenV2);
  outResetV2();
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(0);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  runV2.dir = false;
  digitalWrite(DIR_PIN, LOW);
  runV2.enabled = false;
  digitalWrite(EN_PIN, EN_ACTIVE_LOW ? HIGH : LOW);
  runV2.lastStepUs = micros();
  runV2.lastUpdateUs = micros();
  runV2.nextStepUs = 0;

  TMCSerial.begin(TMC_BAUD);

  // Basic driver init (safe-ish defaults; tune later)
  driver.begin();
  driver.pdn_disable(true);          // use UART
  driver.I_scale_analog(false);      // use internal current reference
  driver.mstep_reg_select(true);     // microsteps via registers (UART)
  driver.toff(4);                    // enable driver
  driver.blank_time(24);
  driver.rms_current(600);           // RMS mA; adjust for your motor
  driver.microsteps(16);
  driver.en_spreadCycle(false);      // stealth by default
  driver.pwm_autoscale(true);

  // Clear latched flags
  driver.GSTAT(0x7);

  v2Initialized = true;
  Serial.println(F("ready"));
}

static inline void setEnableV2(bool on) {
  runV2.enabled = on;
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
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
  outResetV2();
  outAppendCharV2(ok ? '1' : '0');
  outAppendCharV2(';');
}

static void respondEndV2() {
  outFlushLineV2();
}

static void respondKeyValueLongV2(const char* key, long value) {
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

static void respondKeyValueBoolV2(const char* key, bool value) {
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

static void respondKeyValueStrV2(const char* key, const char* value) {
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

static void respondErrorV2(const char* msg) {
  respondStartV2(false);
  if (msg && *msg) {
    respondKeyValueStrV2("error", msg);
  }
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
    if (emit) respondKeyValueU32V2("microsteps", driver.microsteps());
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
    // TODO: Microsteps keeps 256
    driver.microsteps((uint16_t)v);
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

static bool setAccelStepsPerMsV2(long accel, const char** errorKey) {
  if (accel < 0) { if (errorKey) *errorKey = "range"; return false; }
  if (accel > 100000) { if (errorKey) *errorKey = "range"; return false; }
  runV2.accelStepsPerUs = (float)accel / 1000.0f;
  return true;
}

static void setTargetV2(long target) {
  runV2.target = target;
  runV2.hasTarget = true;
}

static void completeTargetV2() {
  runV2.hasTarget = false;
  runV2.running = false;
  runV2.stopRequested = false;
  runV2.actualSpeedSps = 0.0f;
  runV2.desiredSpeedSps = 0.0f;
  runV2.nextStepUs = 0;
}

static const char* getPhaseV2() {
  if (!runV2.enabled) return PHASE_IDLE_V2;
  if (runV2.actualSpeedSps <= 0.0f) return PHASE_HOLD_V2;
  if (runV2.desiredSpeedSps <= 0.0f) return PHASE_DECEL_V2;
  if (runV2.actualSpeedSps < runV2.desiredSpeedSps) return PHASE_ACCEL_V2;
  if (runV2.actualSpeedSps > runV2.desiredSpeedSps) return PHASE_DECEL_V2;
  return PHASE_RUN_V2;
}

static uint32_t calcStepIntervalUsV2(float speedSps) {
  if (speedSps <= 0.0f) return 0;
  float interval = 1000000.0f / speedSps;
  if (interval < (float)(runV2.pulseWidthUs + 4)) {
    interval = (float)(runV2.pulseWidthUs + 4);
  }
  return (uint32_t)interval;
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

  if (runV2.hasTarget) {
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

  if (runV2.hasTarget && desired > 0.0f && runV2.accelStepsPerUs > 0.0f) {
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

static void handleLineV2(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;

  char* cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "status")) {
    respondStartV2(true);
    respondKeyValueBoolV2("initialised", v2Initialized);
    respondKeyValueBoolV2("enabled", runV2.enabled);
    respondKeyValueLongV2("position", getPosition());
    respondKeyValueStrV2("phase", getPhaseV2());
    respondKeyValueLongV2("target", runV2.target);
    respondKeyValueBoolV2("target_set", runV2.hasTarget);
    respondKeyValueFloatV2("speed", runV2.speedSps, 2);
    respondKeyValueFloatV2("actual_speed", runV2.actualSpeedSps, 2);
    respondKeyValueFloatV2("accel", runV2.accelStepsPerUs * 1000.0f, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "full_status") || !strcmp(cmd, "driver_status")) {
    respondStartV2(true);
    respondKeyValueHexV2("gconf", driver.GCONF());
    respondKeyValueHexV2("drv_status", driver.DRV_STATUS());
    respondKeyValueU32V2("sg_result", driver.SG_RESULT());
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
    setEnableV2(enable);
    if (!enable) {
      runV2.running = false;
      runV2.stopRequested = false;
      runV2.actualSpeedSps = 0.0f;
      runV2.desiredSpeedSps = 0.0f;
      runV2.nextStepUs = 0;
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
    if (!setAccelStepsPerMsV2(value, &errorKey)) { respondErrorV2(errorKey ? errorKey : "range"); return; }
    respondStartV2(true);
    respondKeyValueFloatV2("accel", runV2.accelStepsPerUs * 1000.0f, 2);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "target")) {
    char* a = strtok(nullptr, " \t");
    long value = 0;
    if (!a || !parseLongV2(a, &value)) { respondErrorV2("bad_value"); return; }
    setTargetV2(value);
    if (runV2.target == getPosition()) {
      completeTargetV2();
    }
    respondStartV2(true);
    respondKeyValueLongV2("target", runV2.target);
    respondKeyValueBoolV2("target_set", runV2.hasTarget);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "run")) {
    runV2.running = true;
    runV2.stopRequested = false;
    if (!runV2.enabled) setEnableV2(true);
    runV2.lastUpdateUs = micros();
    respondStartV2(true);
    respondKeyValueBoolV2("running", true);
    respondEndV2();
    return;
  }

  if (!strcmp(cmd, "stop")) {
    runV2.stopRequested = true;
    runV2.running = true;
    runV2.lastUpdateUs = micros();
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
        lineBufV2[lineLenV2] = 0;
        const uint32_t start = micros();
        handleLineV2(lineBufV2);
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
}

void serviceStepperv2() {
  const uint32_t nowUs = micros();

  if (runV2.stepHigh) {
    long start = micros();

    if ((int32_t)(nowUs - runV2.stepHighUntilUs) >= 0) {
      digitalWrite(STEP_PIN, LOW);
      runV2.stepHigh = false;
    }

    profiler.stepperLow += micros() - start;
    profiler.stepperLow /= 2.;
    return;
  }

  const float prevSpeed = runV2.actualSpeedSps;
  updateMotionStateV2();

  if (!runV2.enabled) return;
  if (runV2.actualSpeedSps <= 0.0f) return;

  if (prevSpeed <= 0.0f && runV2.actualSpeedSps > 0.0f) {
    runV2.nextStepUs = nowUs;
  }

  runV2.stepIntervalUs = calcStepIntervalUsV2(runV2.actualSpeedSps);
  if (runV2.stepIntervalUs == 0) return;

  if ((int32_t)(nowUs - runV2.nextStepUs) >= 0) {
    long start = micros();

    digitalWrite(STEP_PIN, HIGH);
    runV2.stepHigh = true;
    runV2.stepHighUntilUs = nowUs + runV2.pulseWidthUs;
    runV2.nextStepUs = nowUs + runV2.stepIntervalUs;
    runV2.lastStepUs = nowUs;

    stepPosition += runV2.dir ? STEP_NEGATIVE : STEP_POSITIVE;

    if (runV2.hasTarget) {
      const long pos = getPosition();
      if ((runV2.dir && pos <= runV2.target) || (!runV2.dir && pos >= runV2.target)) {
        setPosition(runV2.target);
        completeTargetV2();
      }
    }

    profiler.stepperHigh += micros() - start;
    profiler.stepperHigh /= 2.;
  }
}

void loop() {
  long start = micros();
  serviceSerialv2();
  profiler.serial += micros() - start;
  profiler.serial /= 2.;

  start = micros();
  serviceStepperv2();
  profiler.stepper += micros() - start;
  profiler.stepper /= 2.;
}
