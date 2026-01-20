/*
  Nano v3 + TMC2209 + TMCStepper
  Commands in Serial Monitor (115200, newline):
    help
    info              - dump registers/status
    enable 0|1
    dir 0|1
    run <sps>         - continuous, steps per second; negative => reverse
    move <steps> <sps>- relative move; steps can be negative; sps optional (default last)
    stop
    current <mA>      - RMS current (library calc depends on R_SENSE)
    microsteps <n>    - 1/2/4/8/16/32/64/128/256
    stealth 0|1       - 1 = stealthChop, 0 = spreadCycle (en_spreadCycle = !stealth)
    sgthrs <0..255>

  Wiring (typical 1-wire UART):
    Nano D11 (TX) --[~1k]--> PDN_UART
    Nano D10 (RX) ----------> PDN_UART
    GND common, VM+motor power as usual.

  STEP/DIR/EN to your driver module pins.
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>

// ---------- Pins ----------
static const uint8_t STEP_PIN = 2;
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

// ---------- Simple step generator (non-blocking pulses) ----------
struct Runner {
  bool enabled = false;
  bool continuous = false;
  bool dir = false;
  long remaining = 0;              // steps left (for move)
  uint32_t stepIntervalUs = 2000;  // default 500 sps
  uint32_t pulseWidthUs   = 3;
  uint32_t nextStepUs     = 0;
  bool stepHigh = false;
  uint32_t stepHighUntilUs = 0;
  int lastSps = 500;
} run;

static inline void setEnable(bool on) {
  run.enabled = on;
  digitalWrite(EN_PIN, (EN_ACTIVE_LOW ? !on : on) ? HIGH : LOW);
}

static inline void setDir(bool dir) {
  run.dir = dir;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
}

static inline void setSpeedSps(int sps_abs) {
  if (sps_abs < 1) sps_abs = 1;
  if (sps_abs > 40000) sps_abs = 40000;                 // sanity cap
  run.lastSps = sps_abs;
  run.stepIntervalUs = (uint32_t)(1000000UL / (uint32_t)sps_abs);
  if (run.stepIntervalUs < (run.pulseWidthUs + 4)) run.stepIntervalUs = run.pulseWidthUs + 4;
}

static void serviceStepper() {
  const uint32_t now = micros();

  if (run.stepHigh) {
    if ((int32_t)(now - run.stepHighUntilUs) >= 0) {
      digitalWrite(STEP_PIN, LOW);
      run.stepHigh = false;
      if (!run.continuous && run.remaining == 0) {
        // finished move
      }
    }
    return;
  }

  const bool shouldStep = run.enabled && (run.continuous || run.remaining > 0);
  if (!shouldStep) return;

  if ((int32_t)(now - run.nextStepUs) >= 0) {
    digitalWrite(STEP_PIN, HIGH);
    run.stepHigh = true;
    run.stepHighUntilUs = now + run.pulseWidthUs;
    run.nextStepUs = now + run.stepIntervalUs;

    if (!run.continuous && run.remaining > 0) {
      run.remaining--;
    }
  }
}

// ---------- Printing helpers ----------
static void printHex32(const __FlashStringHelper* name, uint32_t v) {
  Serial.print(name);
  Serial.print(F(" = 0x"));
  if (v < 0x10000000UL) Serial.print('0');
  if (v < 0x01000000UL) Serial.print('0');
  if (v < 0x00100000UL) Serial.print('0');
  if (v < 0x00010000UL) Serial.print('0');
  if (v < 0x00001000UL) Serial.print('0');
  if (v < 0x00000100UL) Serial.print('0');
  if (v < 0x00000010UL) Serial.print('0');
  Serial.println(v, HEX);
}

static void dumpInfo() {
  Serial.println(F("\n=== TMC2209 dump ==="));

  // Communication health
  Serial.print(F("IFCNT (UART OK counter) = "));
  Serial.println(driver.IFCNT());

  // IOIN bits and version
  printHex32(F("IOIN"), driver.IOIN());
  Serial.print(F("IOIN.version = ")); Serial.println(driver.version(), HEX);
  Serial.print(F("IOIN.enn/ms1/ms2/diag/pdn_uart/step/dir/spread_en = "));
  Serial.print(driver.enn()); Serial.print('/');
  Serial.print(driver.ms1()); Serial.print('/');
  Serial.print(driver.ms2()); Serial.print('/');
  Serial.print(driver.diag()); Serial.print('/');
  Serial.print(driver.pdn_uart()); Serial.print('/');
  Serial.print(driver.step()); Serial.print('/');
  Serial.print(driver.dir()); Serial.print('/');
  Serial.println(driver.spread_en());

  // Core config registers
  printHex32(F("GCONF"), driver.GCONF());
  printHex32(F("GSTAT"), driver.GSTAT());
  printHex32(F("IHOLD_IRUN"), driver.IHOLD_IRUN());
  Serial.print(F("TPOWERDOWN = ")); Serial.println(driver.TPOWERDOWN());
  printHex32(F("TPWMTHRS"), driver.TPWMTHRS());
  printHex32(F("TCOOLTHRS"), driver.TCOOLTHRS());
  Serial.print(F("SGTHRS = ")); Serial.println(driver.SGTHRS());
  printHex32(F("CHOPCONF"), driver.CHOPCONF());
  printHex32(F("PWMCONF"), driver.PWMCONF());

  // Motion / status
  printHex32(F("VACTUAL"), driver.VACTUAL());
  Serial.print(F("TSTEP = ")); Serial.println(driver.TSTEP());
  Serial.print(F("MSCNT = ")); Serial.println(driver.MSCNT());
  printHex32(F("MSCURACT"), driver.MSCURACT());
  printHex32(F("DRV_STATUS"), driver.DRV_STATUS());
  Serial.print(F("SG_RESULT = ")); Serial.println(driver.SG_RESULT());

  // Decoded safety flags (common helpers in TMCStepper)
  Serial.print(F("Flags: ot="));   Serial.print(driver.ot());
  Serial.print(F(" otpw="));       Serial.print(driver.otpw());
  Serial.print(F(" s2ga="));       Serial.print(driver.s2ga());
  Serial.print(F(" s2gb="));       Serial.print(driver.s2gb());
  Serial.print(F(" ola="));        Serial.print(driver.ola());
  Serial.print(F(" olb="));        Serial.print(driver.olb());
  Serial.print(F(" t120="));       Serial.print(driver.t120());
  Serial.print(F(" t143="));       Serial.print(driver.t143());
  Serial.print(F(" t150="));       Serial.print(driver.t150());
  Serial.print(F(" t157="));       Serial.print(driver.t157());
  Serial.print(F(" stst="));       Serial.print(driver.stst());
  Serial.print(F(" stealth="));    Serial.print(driver.stealth());
  Serial.print(F(" cs_actual="));  Serial.println(driver.cs_actual());

  Serial.println(F("=== end ===\n"));
}

static void printHelp() {
  Serial.println(F(
    "help | info | enable 0|1 | dir 0|1 | run <sps> | move <steps> <sps> | stop\n"
    "current <mA> | microsteps <n> | stealth 0|1 | sgthrs <0..255>\n"
  ));
}

// ---------- Simple line parser ----------
static char lineBuf[96];
static uint8_t lineLen = 0;

static void handleLine(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return;

  char *cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcmp(cmd, "help")) { printHelp(); return; }
  if (!strcmp(cmd, "info") || !strcmp(cmd, "dump")) { dumpInfo(); return; }

  if (!strcmp(cmd, "enable")) {
    char *a = strtok(nullptr, " \t");
    setEnable(a && atoi(a) != 0);
    Serial.print(F("enable=")); Serial.println(run.enabled);
    return;
  }

  if (!strcmp(cmd, "dir")) {
    char *a = strtok(nullptr, " \t");
    setDir(a && atoi(a) != 0);
    Serial.print(F("dir=")); Serial.println(run.dir);
    return;
  }

  if (!strcmp(cmd, "stop")) {
    run.continuous = false;
    run.remaining = 0;
    Serial.println(F("stopped"));
    return;
  }

  if (!strcmp(cmd, "run")) {
    char *a = strtok(nullptr, " \t");
    int sps = a ? atoi(a) : run.lastSps;
    bool dir = (sps < 0);
    if (sps < 0) sps = -sps;
    setDir(dir);
    setSpeedSps(sps);
    run.continuous = true;
    run.remaining = 0;
    setEnable(true);
    Serial.print(F("run sps=")); Serial.print(sps);
    Serial.print(F(" dir=")); Serial.println(dir);
    return;
  }

  if (!strcmp(cmd, "move")) {
    char *a = strtok(nullptr, " \t");
    char *b = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("move needs <steps>")); return; }
    long steps = atol(a);
    int sps = b ? atoi(b) : run.lastSps;
    bool dir = (steps < 0);
    if (steps < 0) steps = -steps;
    if (sps < 0) sps = -sps;

    setDir(dir);
    setSpeedSps(sps);
    run.continuous = false;
    run.remaining = steps;
    setEnable(true);

    Serial.print(F("move steps=")); Serial.print(steps);
    Serial.print(F(" sps=")); Serial.print(sps);
    Serial.print(F(" dir=")); Serial.println(dir);
    return;
  }

  if (!strcmp(cmd, "current")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("current needs <mA>")); return; }
    uint16_t mA = (uint16_t)constrain(atoi(a), 50, 2000);
    driver.rms_current(mA);
    Serial.print(F("rms_current(mA)=")); Serial.println(mA);
    return;
  }

  if (!strcmp(cmd, "microsteps")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("microsteps needs <n>")); return; }
    uint16_t ms = (uint16_t)atoi(a);
    driver.microsteps(ms);
    Serial.print(F("microsteps=")); Serial.println(ms);
    return;
  }

  if (!strcmp(cmd, "stealth")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("stealth needs 0|1")); return; }
    bool stealth = atoi(a) != 0;
    driver.en_spreadCycle(!stealth);
    driver.pwm_autoscale(true);
    Serial.print(F("stealth=")); Serial.println(stealth);
    return;
  }

  if (!strcmp(cmd, "sgthrs")) {
    char *a = strtok(nullptr, " \t");
    if (!a) { Serial.println(F("sgthrs needs 0..255")); return; }
    uint8_t v = (uint8_t)constrain(atoi(a), 0, 255);
    driver.SGTHRS(v);
    Serial.print(F("SGTHRS=")); Serial.println(v);
    return;
  }

  Serial.print(F("unknown: ")); Serial.println(cmd);
}

static void serviceSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = 0;
      handleLine(lineBuf);
      lineLen = 0;
      continue;
    }
    if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  setDir(false);
  setEnable(false);

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

  Serial.println(F("Ready. Type 'help'."));
  dumpInfo();
}

void loop() {
  serviceSerial();
  serviceStepper();

  // Optional: notify when a move finished (edge)
  static long lastRemaining = -1;
  if (!run.continuous && lastRemaining != 0 && run.remaining == 0) {
    Serial.println(F("move done"));
  }
  lastRemaining = run.remaining;
}