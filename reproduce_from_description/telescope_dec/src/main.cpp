#include <Arduino.h>

struct MotionState {
  bool initialised = true;
  bool enabled = true;
  bool direction_positive = true;
  bool target_set = false;
  String mode = "free_ride";
  String phase = "idle";
  long position = 0;
  long target = 0;
  long speed = 0;
  long actual_speed = 0;
  long accel_per_s = 1000;
  long microsteps = 16;
  unsigned long last_tick_ms = 0;
};

MotionState state;
String input_line;

void respond_ok(const String &payload) {
  Serial.print("1;");
  Serial.print(payload);
  Serial.println(";");
}

void respond_error(const String &payload) {
  Serial.print("0;");
  Serial.print(payload);
  Serial.println(";");
}

String status_payload() {
  return "initialised=1;enabled=" + String(state.enabled ? 1 : 0) +
         ";mode=" + state.mode +
         ";position=" + String(state.position) +
         ";phase=" + state.phase +
         ";target=" + String(state.target) +
         ";target_set=" + String(state.target_set ? 1 : 0) +
         ";speed=" + String(state.speed) +
         ";actual_speed=" + String(state.actual_speed) +
         ";accel_per_s=" + String(state.accel_per_s) +
         ";direction=" + String(state.direction_positive ? 1 : 0);
}

void service_motion() {
  unsigned long now = millis();
  unsigned long elapsed = now - state.last_tick_ms;
  if (elapsed == 0) {
    return;
  }
  state.last_tick_ms = now;

  if (state.phase == "idle") {
    state.actual_speed = 0;
    return;
  }

  if (state.mode == "free_ride") {
    state.actual_speed = state.speed;
    long delta = (state.speed * static_cast<long>(elapsed)) / 1000L;
    state.position += state.direction_positive ? delta : -delta;
    state.phase = state.speed == 0 ? "idle" : "cruise";
    return;
  }

  long signed_speed = state.direction_positive ? state.speed : -state.speed;
  state.actual_speed = abs(signed_speed);
  long step_delta = (abs(signed_speed) * static_cast<long>(elapsed)) / 1000L;
  if (step_delta == 0) {
    state.phase = "cruise";
    return;
  }

  long next_position = state.position + (signed_speed >= 0 ? step_delta : -step_delta);
  bool reached = (signed_speed >= 0 && next_position >= state.target) || (signed_speed < 0 && next_position <= state.target);
  if (reached) {
    state.position = state.target;
    state.phase = "target_reached";
    state.target_set = false;
    state.actual_speed = 0;
    return;
  }

  state.position = next_position;
  state.phase = "cruise";
}

void handle_set(String payload) {
  int separator = payload.indexOf('=');
  if (separator < 0) {
    respond_error("reason=bad_set");
    return;
  }

  String key = payload.substring(0, separator);
  String value = payload.substring(separator + 1);
  if (key == "microsteps") {
    state.microsteps = value.toInt();
    respond_ok("microsteps=" + String(state.microsteps));
    return;
  }

  respond_error("reason=unknown_key");
}

void handle_command(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "status" || line == "full_status") {
    respond_ok(status_payload());
    return;
  }

  if (line == "driver_status" || line == "profile") {
    respond_ok("phase=" + state.phase + ";speed=" + String(state.speed));
    return;
  }

  if (line == "run") {
    state.phase = state.mode == "target" && state.target_set ? "accel" : (state.speed == 0 ? "idle" : "cruise");
    respond_ok("phase=" + state.phase);
    return;
  }

  if (line == "stop") {
    state.phase = "idle";
    state.actual_speed = 0;
    state.target_set = false;
    respond_ok("phase=idle");
    return;
  }

  if (line.startsWith("position ")) {
    state.position = line.substring(9).toInt();
    respond_ok("position=" + String(state.position));
    return;
  }

  if (line.startsWith("speed ")) {
    state.speed = max(0L, line.substring(6).toInt());
    respond_ok("speed=" + String(state.speed));
    return;
  }

  if (line.startsWith("acceleration ")) {
    state.accel_per_s = max(0L, line.substring(13).toInt());
    respond_ok("accel_per_s=" + String(state.accel_per_s));
    return;
  }

  if (line.startsWith("direction ")) {
    state.direction_positive = line.substring(10).toInt() == 1;
    respond_ok("direction=" + String(state.direction_positive ? 1 : 0));
    return;
  }

  if (line.startsWith("delta ")) {
    long delta = line.substring(6).toInt();
    state.target = state.position + (state.direction_positive ? delta : -delta);
    state.target_set = true;
    respond_ok("target=" + String(state.target) + ";target_set=1");
    return;
  }

  if (line.startsWith("mode ")) {
    String mode = line.substring(5);
    if (mode != "target" && mode != "free_ride") {
      respond_error("reason=bad_mode");
      return;
    }
    state.mode = mode;
    state.phase = "idle";
    respond_ok("mode=" + state.mode);
    return;
  }

  if (line.startsWith("set ")) {
    handle_set(line.substring(4));
    return;
  }

  if (line.startsWith("get ")) {
    String key = line.substring(4);
    if (key == "microsteps") {
      respond_ok("microsteps=" + String(state.microsteps));
      return;
    }
    respond_error("reason=unknown_key");
    return;
  }

  respond_error("reason=unknown_command");
}

void setup() {
  Serial.begin(115200);
  state.last_tick_ms = millis();
}

void loop() {
  service_motion();
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      handle_command(input_line);
      input_line = "";
      continue;
    }
    if (c != '\r') {
      input_line += c;
    }
  }
}
