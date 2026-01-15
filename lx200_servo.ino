/*
  dec_lx200_tmc2209.ino

  Arduino DEC axis controller for a "Franken mount":
  - Drives a stepper motor via TMC2209 (STEP/DIR + UART config)
  - Exposes a subset of LX200 over a serial port (to be used by franken_lx200_bridge.py)

  IMPORTANT ABOUT LOGS:
  - LX200 is strict: do NOT print logs into the same serial stream used for LX200 commands.
  - If you have only one Serial (UNO/Nano), keep logs disabled (ENABLE_LOGS 0).
  - If your board has Serial1/Serial2, route logs there.

  Supported LX200 subset (as used by the Python bridge):
    :GD#                  -> returns current DEC "+DD*MM:SS#"
    :Sd <dec>#            -> set target DEC, returns "1#"
    :Sr <ra>#             -> accepted (ignored), returns "1#"
    :MS#                  -> start goto to target DEC, returns "0#"
    :Q#                   -> stop NOW, returns "1#"
    :Qn#/:Qs#/:Qe#/:Qw#    -> stop NOW, returns "1#"
    :Mn#                  -> manual move North (positive DEC), returns "1#"
    :Ms#                  -> manual move South (negative DEC), returns "1#"
    :Me#/:Mw#             -> not applicable for DEC, returns "0#"
    :RG#/:RC#/:RM#/:RS#    -> set manual rate level, returns "1#"

  Non-standard extensions (optional, useful for tuning):
    :XAC+001.230#         -> set acceleration in deg/s^2, returns "1#"
    :XVM0004.000#         -> set vmax in deg/s, returns "1#"
    :XSC+DD*MM:SS#        -> set current DEC (sync current position), returns "1#"
    :X?#                  -> short status line, returns "<...>#"

  Coordinate model:
  - No encoders: DEC is derived from stepper position (steps -> degrees).
  - On boot, current position is 0. Use :XSC...# to sync if needed.

  Libraries:
    - AccelStepper (Stepper with acceleration)
    - TMCStepper (UART config for TMC2209)
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

// ------------------------
// Compile-time options
// ------------------------

// If you don't have a second serial for logs, keep 0
#define ENABLE_LOGS 0

// LX200 serial port (to Python bridge)
#define LX200_SERIAL Serial
#define LX200_BAUD   115200

// Optional logs port (only if your board supports it; e.g. Mega has Serial1)
#if ENABLE_LOGS
  #if defined(HAVE_HWSERIAL1)
    #define LOG_SERIAL Serial1
    #define LOG_BAUD   115200
  #else
    // fallback: logs disabled if no extra HW serial
    #undef ENABLE_LOGS
    #define ENABLE_LOGS 0
  #endif
#endif

// ------------------------
// Pins (edit to your wiring)
// ------------------------

// Stepper driver pins (TMC2209 STEP/DIR/EN)
static const uint8_t PIN_STEP = 2;
static const uint8_t PIN_DIR  = 3;
static const uint8_t PIN_EN   = 4;

// TMC2209 UART pins (single-wire via PDN_UART, or separate RX/TX depending on board wiring)
// Many setups use a single UART line; for SoftwareSerial we still need RX+TX pins.
static const uint8_t PIN_TMC_RX = 10;  // Arduino receives from TMC
static const uint8_t PIN_TMC_TX = 11;  // Arduino transmits to TMC

// ------------------------
// Motion configuration (edit to your mechanics)
// ------------------------

// Mechanical conversion: steps per full DEC degree.
//
// You should compute this from:
//   motor_steps_per_rev (usually 200 for 1.8°)
// * microsteps (e.g. 16)
// * gear_ratio (e.g. worm 144:1 => 144)
// / 360
//
// Example: 200*16*144 / 360 = 1280 steps/deg
static float STEPS_PER_DEG = 1280.0f;

// Limits (deg)
static const float DEC_MIN_DEG = -90.0f;
static const float DEC_MAX_DEG = +90.0f;

// Default motion tuning (can be overridden by :XAC and :XVM)
static float dec_vmax_deg_s   = 4.0f;  // max speed in deg/s
static float dec_accel_deg_s2 = 5.0f;  // accel in deg/s^2

// Manual rate levels (deg/s)
static float rate_s_deg_s = 0.02f;   // :RS slow
static float rate_m_deg_s = 0.2f;    // :RM medium
static float rate_c_deg_s = 1.0f;    // :RC center
static float rate_g_deg_s = 3.0f;    // :RG guide/faster manual

// ------------------------
// Driver + stepper objects
// ------------------------

SoftwareSerial TMCSerial(PIN_TMC_RX, PIN_TMC_TX);  // RX, TX

// For TMCStepper:
static const uint8_t TMC_ADDR = 0b00;  // set by MS1/MS2 pins on some boards; often 0
static const float R_SENSE = 0.11f;    // common value; adjust to your driver board

TMC2209Stepper driver(&TMCSerial, R_SENSE, TMC_ADDR);

// AccelStepper in DRIVER mode: step/dir
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// ------------------------
// State
// ------------------------

static float target_dec_deg = 0.0f;
static bool  target_dec_valid = false;

enum MoveMode : uint8_t { MODE_IDLE=0, MODE_GOTO=1, MODE_MANUAL=2 };
static MoveMode mode = MODE_IDLE;

// manual direction: +1 north (increase DEC), -1 south
static int8_t manual_dir = 0;

// Current manual speed (deg/s), set by :RG/:RC/:RM/:RS
static float manual_speed_deg_s = 0.2f;

// Parser buffer for LX200 frames (":" ... "#")
static char cmd_buf[96];
static uint8_t cmd_len = 0;
static bool in_frame = false;

// ------------------------
// Logging helpers
// ------------------------

static inline void logln(const __FlashStringHelper* s) {
#if ENABLE_LOGS
  LOG_SERIAL.println(s);
#endif
}
static inline void logf(const char* fmt, ...) {
#if ENABLE_LOGS
  char b[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  LOG_SERIAL.println(b);
#else
  (void)fmt;
#endif
}

// ------------------------
// Utility: numeric conversions
// ------------------------

static inline long degToSteps(float deg) {
  return lroundf(deg * STEPS_PER_DEG);
}
static inline float stepsToDeg(long steps) {
  return (float)steps / STEPS_PER_DEG;
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void applyMotionTuning() {
  // AccelStepper uses steps/s and steps/s^2
  const float vmax_steps_s   = dec_vmax_deg_s   * STEPS_PER_DEG;
  const float accel_steps_s2 = dec_accel_deg_s2 * STEPS_PER_DEG;

  stepper.setMaxSpeed(vmax_steps_s);
  stepper.setAcceleration(accel_steps_s2);
}

static void enableDriver(bool en) {
  // TMC EN is often active-low
  digitalWrite(PIN_EN, en ? LOW : HIGH);
  if (!en) {
    // also stop step pulses
    stepper.setSpeed(0);
  }
}

static void hardStopNow() {
  // Immediate stop: set target to current and zero speed
  long cur = stepper.currentPosition();
  stepper.setCurrentPosition(cur);
  stepper.moveTo(cur);
  stepper.setSpeed(0);
  mode = MODE_IDLE;
  manual_dir = 0;
}

// ------------------------
// Utility: DEC formatting/parsing
// LX200 DEC formats typically: "+DD*MM:SS" (we'll output that)
// ------------------------

static void formatDec(float dec_deg, char* out, size_t out_sz) {
  dec_deg = clampf(dec_deg, -90.0f, 90.0f);
  char sign = '+';
  if (dec_deg < 0) { sign = '-'; dec_deg = -dec_deg; }

  int d = (int)dec_deg;
  float rem = (dec_deg - (float)d) * 60.0f;
  int m = (int)rem;
  int s = (int)lroundf((rem - (float)m) * 60.0f);

  if (s == 60) { s = 0; m += 1; }
  if (m == 60) { m = 0; d += 1; }

  snprintf(out, out_sz, "%c%02d*%02d:%02d", sign, d, m, s);
}

static bool parseDec(const char* s, float* out_dec_deg) {
  // Accept: "+DD*MM:SS" or "+DD*MM" (SS optional)
  // Also accept degree symbol replaced with '*'
  if (!s || !out_dec_deg) return false;

  while (*s == ' ') s++;

  int sign = +1;
  if (*s == '-') { sign = -1; s++; }
  else if (*s == '+') { sign = +1; s++; }

  // parse DD
  char* endp = nullptr;
  long dd = strtol(s, &endp, 10);
  if (endp == s) return false;
  s = endp;

  if (*s != '*' && *s != 0xB0) return false; // '*' or '°'
  s++;

  long mm = strtol(s, &endp, 10);
  if (endp == s) return false;
  s = endp;

  long ss = 0;
  if (*s == ':') {
    s++;
    ss = strtol(s, &endp, 10);
    if (endp == s) return false;
    s = endp;
  }

  float deg = (float)dd + (float)mm/60.0f + (float)ss/3600.0f;
  deg *= (float)sign;
  deg = clampf(deg, -90.0f, 90.0f);
  *out_dec_deg = deg;
  return true;
}

// ------------------------
// LX200 responder
// Always end responses with '#'
// ------------------------

static void replyHash(const char* payload) {
  LX200_SERIAL.print(payload);
  LX200_SERIAL.print('#');
}

static void reply1() { replyHash("1"); }
static void reply0() { replyHash("0"); }

// ------------------------
// Command handlers
// ------------------------

static void handle_GD() {
  float dec = stepsToDeg(stepper.currentPosition());
  char buf[16];
  formatDec(dec, buf, sizeof(buf));
  replyHash(buf);
}

static void handle_Sd(const char* args) {
  float dec = 0.0f;
  if (!parseDec(args, &dec)) {
    reply0();
    return;
  }
  target_dec_deg = dec;
  target_dec_valid = true;
  reply1();
}

static void handle_Sr(const char* /*args*/) {
  // DEC axis does not need RA; accept for compatibility
  reply1();
}

static void startGoto() {
  if (!target_dec_valid) {
    reply0();
    return;
  }
  float dec = clampf(target_dec_deg, DEC_MIN_DEG, DEC_MAX_DEG);
  long tgt_steps = degToSteps(dec);

  // Enable driver and go
  enableDriver(true);
  applyMotionTuning();
  stepper.moveTo(tgt_steps);

  mode = MODE_GOTO;
  manual_dir = 0;

  replyHash("0"); // LX200 convention: "0" often means GOTO accepted/success
}

static void handle_MS() {
  startGoto();
}

static void handle_Q() {
  enableDriver(true);   // ensure we can brake / hold
  hardStopNow();
  reply1();
}

static void startManual(int8_t dir) {
  // Manual move with accel:
  // set a far away target so it keeps going while we keep calling run()
  enableDriver(true);
  applyMotionTuning();

  manual_dir = dir;
  mode = MODE_MANUAL;

  // Use current manual speed as vmax for manual mode (but keep global vmax as a ceiling)
  float v = clampf(manual_speed_deg_s, 0.001f, dec_vmax_deg_s);
  stepper.setMaxSpeed(v * STEPS_PER_DEG);

  long cur = stepper.currentPosition();
  const long FAR = 0x3FFFFFFF; // huge distance
  long tgt = (dir > 0) ? (cur + FAR) : (cur - FAR);
  stepper.moveTo(tgt);

  reply1();
}

static void handle_Mn() { startManual(+1); } // North -> +DEC
static void handle_Ms() { startManual(-1); } // South -> -DEC

static void handle_Me() { reply0(); } // not applicable
static void handle_Mw() { reply0(); } // not applicable

static void handle_rate(char which) {
  // :RS :RM :RC :RG
  switch (which) {
    case 'S': manual_speed_deg_s = rate_s_deg_s; break;
    case 'M': manual_speed_deg_s = rate_m_deg_s; break;
    case 'C': manual_speed_deg_s = rate_c_deg_s; break;
    case 'G': manual_speed_deg_s = rate_g_deg_s; break;
    default: break;
  }
  reply1();
}

// Non-standard extensions
static void handle_XAC(const char* args) {
  // :XAC+001.230
  // args may start immediately after XAC
  float a = atof(args);
  if (!(a > 0.0f)) { reply0(); return; }
  dec_accel_deg_s2 = a;
  applyMotionTuning();
  reply1();
}

static void handle_XVM(const char* args) {
  // :XVM0004.000
  float v = atof(args);
  if (!(v > 0.0f)) { reply0(); return; }
  dec_vmax_deg_s = v;
  applyMotionTuning();
  reply1();
}

static void handle_XSC(const char* args) {
  // :XSC+DD*MM:SS  -> set currentPosition to match given DEC
  float dec = 0.0f;
  if (!parseDec(args, &dec)) { reply0(); return; }
  dec = clampf(dec, DEC_MIN_DEG, DEC_MAX_DEG);
  long steps = degToSteps(dec);

  enableDriver(true);
  stepper.setCurrentPosition(steps);
  stepper.moveTo(steps);
  stepper.setSpeed(0);

  reply1();
}

static void handle_XQ() {
  // :X?# -> status
  // Example: "MODE=1 CUR=+12.345 TGT=+10.000 V=4.0 A=5.0"
  char curbuf[16], tgtbuf[16];
  float cur = stepsToDeg(stepper.currentPosition());
  formatDec(cur, curbuf, sizeof(curbuf));
  formatDec(target_dec_deg, tgtbuf, sizeof(tgtbuf));

  char out[80];
  snprintf(out, sizeof(out),
           "MODE=%u CUR=%s TGT=%s VM=%.3f AC=%.3f",
           (unsigned)mode, curbuf, tgtbuf, dec_vmax_deg_s, dec_accel_deg_s2);
  replyHash(out);
}

// ------------------------
// Parse and dispatch one command frame body (without ':' and '#')
// ------------------------

static void dispatch(const char* body) {
  // body example: "GD" or "Sd +12*34:56" or "XAC+001.23"
  // trim leading spaces
  while (*body == ' ') body++;

  // Basic two-letter commands
  if (strncmp(body, "GD", 2) == 0) { handle_GD(); return; }
  if (strncmp(body, "MS", 2) == 0) { handle_MS(); return; }
  if (strncmp(body, "Q", 1) == 0)  { handle_Q(); return; }

  // Stops by direction
  if (strncmp(body, "Qn", 2) == 0) { handle_Q(); return; }
  if (strncmp(body, "Qs", 2) == 0) { handle_Q(); return; }
  if (strncmp(body, "Qe", 2) == 0) { handle_Q(); return; }
  if (strncmp(body, "Qw", 2) == 0) { handle_Q(); return; }

  // Manual moves
  if (strncmp(body, "Mn", 2) == 0) { handle_Mn(); return; }
  if (strncmp(body, "Ms", 2) == 0) { handle_Ms(); return; }
  if (strncmp(body, "Me", 2) == 0) { handle_Me(); return; }
  if (strncmp(body, "Mw", 2) == 0) { handle_Mw(); return; }

  // Rate levels
  if (strncmp(body, "RS", 2) == 0) { handle_rate('S'); return; }
  if (strncmp(body, "RM", 2) == 0) { handle_rate('M'); return; }
  if (strncmp(body, "RC", 2) == 0) { handle_rate('C'); return; }
  if (strncmp(body, "RG", 2) == 0) { handle_rate('G'); return; }

  // Set target DEC
  if (strncmp(body, "Sd", 2) == 0) {
    const char* args = body + 2;
    handle_Sd(args);
    return;
  }

  // Set target RA (ignored)
  if (strncmp(body, "Sr", 2) == 0) {
    const char* args = body + 2;
    handle_Sr(args);
    return;
  }

  // Extensions
  if (strncmp(body, "XAC", 3) == 0) { handle_XAC(body + 3); return; }
  if (strncmp(body, "XVM", 3) == 0) { handle_XVM(body + 3); return; }
  if (strncmp(body, "XSC", 3) == 0) { handle_XSC(body + 3); return; }
  if (strncmp(body, "X?", 2) == 0)  { handle_XQ(); return; }

  // Unknown command -> "0#"
  reply0();
}

// ------------------------
// Serial frame reader
// ------------------------

static void pollLX200() {
  while (LX200_SERIAL.available() > 0) {
    char c = (char)LX200_SERIAL.read();

    if (!in_frame) {
      if (c == ':') {
        in_frame = true;
        cmd_len = 0;
      }
      continue;
    }

    if (c == '#') {
      // end of frame
      cmd_buf[cmd_len] = '\0';
      in_frame = false;
      dispatch(cmd_buf);
      continue;
    }

    // buffer
    if (cmd_len + 1 < sizeof(cmd_buf)) {
      cmd_buf[cmd_len++] = c;
    } else {
      // overflow: drop frame
      in_frame = false;
      cmd_len = 0;
      reply0();
    }
  }
}

// ------------------------
// TMC2209 setup
// ------------------------

static void setupTMC2209() {
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  enableDriver(false);

  TMCSerial.begin(115200);

  driver.begin();
  driver.pdn_disable(true);       // use UART
  driver.I_scale_analog(false);   // use internal current reference
  driver.toff(4);
  driver.blank_time(24);

  // Current (mA RMS) - tune for your motor/thermal
  driver.rms_current(900);

  // Microsteps - tune; also keep STEPS_PER_DEG consistent!
  driver.microsteps(16);
  driver.interpolate(true);

  // Motion mode
  driver.en_spreadCycle(false);   // false => StealthChop (quiet)
  driver.pwm_autoscale(true);

  // Optional: reduce idle current
  driver.TPOWERDOWN(10);

#if ENABLE_LOGS
  logf("TMC2209: microsteps=%u, current(mA)~%u", driver.microsteps(), 900);
#endif
}

// ------------------------
// Arduino setup/loop
// ------------------------

void setup() {
  LX200_SERIAL.begin(LX200_BAUD);

#if ENABLE_LOGS
  LOG_SERIAL.begin(LOG_BAUD);
  logln(F("DEC LX200 TMC2209 boot"));
#endif

  setupTMC2209();

  stepper.setEnablePin(PIN_EN);
  stepper.setPinsInverted(false, false, true); // enable pin inverted (active-low)
  stepper.disableOutputs(); // start disabled

  applyMotionTuning();

  // Start at 0 degrees
  stepper.setCurrentPosition(0);
  target_dec_deg = 0.0f;
  target_dec_valid = false;

#if ENABLE_LOGS
  logf("STEPS_PER_DEG=%.3f vmax=%.3f deg/s accel=%.3f deg/s^2", STEPS_PER_DEG, dec_vmax_deg_s, dec_accel_deg_s2);
#endif
}

void loop() {
  // 1) handle incoming LX200
  pollLX200();

  // 2) drive motion
  // Keep outputs enabled while moving or in manual mode; disable when idle (optional).
  if (mode == MODE_GOTO || mode == MODE_MANUAL) {
    stepper.enableOutputs();
    stepper.run();

    // Clamp DEC limits in software (no endstops):
    float dec = stepsToDeg(stepper.currentPosition());
    if (dec <= DEC_MIN_DEG || dec >= DEC_MAX_DEG) {
      hardStopNow();
#if ENABLE_LOGS
      logln(F("LIMIT HIT -> STOP"));
#endif
    }

    // If GOTO reached target
    if (mode == MODE_GOTO && stepper.distanceToGo() == 0) {
      mode = MODE_IDLE;
      manual_dir = 0;
      // Optionally keep holding torque:
      // stepper.disableOutputs();
#if ENABLE_LOGS
      logln(F("GOTO complete"));
#endif
    }
  } else {
    // idle
    // stepper.disableOutputs();  // uncomment if you want motor to relax when idle
  }
}