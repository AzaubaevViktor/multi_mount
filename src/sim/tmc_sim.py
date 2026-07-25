"""TMC2209/Arduino DEC-controller endpoint simulator.

Ground truth is the firmware ``telescope_dec/src/main.cpp`` (the Python driver
``src/tmc2209/motor.py`` is its client): line-based protocol terminated by
``\\n`` (``\\r`` ignored), replies ``1;key=value;...\\n`` / ``0;error=...;\\n``
(``outFlushLineV2`` appends a bare LF), floats formatted with 2 decimals, and a
single ``ready\\r\\n`` line printed by ``Serial.println(F("ready"))`` at the end
of ``setup()`` after the board resets (host DTR toggle).

Motion model mirrors ``updateMotionStateV2``/``serviceStepperv2``:

- ``desired`` speed is 0 when stopping/not running, else ``speed``;
- in target mode the direction is forced towards the target and ``desired``
  drops to 0 once within the stopping distance ``actual^2 / (2*accel)``;
- ``actual`` ramps towards ``desired`` at ``accel`` steps/s^2 (jumps instantly
  when accel is 0), position integrates ``actual`` over the virtual clock, and
  crossing the target snaps to it (``completeTargetV2``).

Phases replicate ``getPhaseCodeV2``: idle (disabled), hold (actual<=0),
deceleration (desired<=0 or actual>desired), acceleration (actual<desired),
running otherwise. The virtual :class:`sim.clock.Clock` is the only time
source; integration substeps are pure computation.

Boot delay: after a DTR rising edge the firmware takes a couple of real
seconds to boot, so the host's ``read_all_data()`` right after
``SerialLine.reset()`` sees nothing and ``ready`` arrives on the following
poll. The sim models that as "skip one drain after reset" instead of real time.
"""

from sim.clock import Clock
from sim.faults import FaultScript

MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}

_MAX_SPEED_SPS = 40000
_MAX_ACCEL_SPS2 = 100000

_INTEGRATION_SUBSTEP_S = 0.01


class TMC2209Sim:
    """Simulated DEC controller behind a :class:`sim.fake_serial.FakeSerial`."""

    def __init__(self, clock: Clock, *, power_v: float = 12.0, ready_on_reset: bool = True) -> None:
        self._clock = clock
        self.faults = FaultScript()
        self.power_v = power_v
        self.ready_on_reset = ready_on_reset

        self._rx = bytearray()
        self._tx = bytearray()
        self._skip_drains = 0
        self._boot()

    def _boot(self) -> None:
        self.enabled = False
        self.dir_negative = False
        self.running = False
        self.stop_requested = False
        self.has_target = False
        self.free_ride = False
        self.target = 0
        self.speed_sps = 500.0
        self.desired_sps = 0.0
        self.actual_sps = 0.0
        self.accel_sps2 = 1000.0
        self.position = 0.0
        self.microsteps = 16

        self._integrated_at = self._clock.now
        if self.ready_on_reset:
            # `Serial.println(F("ready"))` (main.cpp:452) — Arduino's println
            # appends CR LF, unlike the plain LF of `outFlushLineV2` used by
            # every command reply.
            self._tx.extend(b"ready\r\n")

    def feed(self, data: bytes) -> None:
        for byte in data:
            if byte == ord("\r"):
                continue
            if byte == ord("\n"):
                line = self._rx.decode("ascii", errors="replace")
                self._rx.clear()
                self._handle(line)
                continue
            self._rx.append(byte)

    def drain(self) -> bytes:
        if self._skip_drains > 0:
            self._skip_drains -= 1
            return b""
        data = bytes(self._tx)
        self._tx.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        if level:
            # Rising edge = board reboot; one skipped drain models boot time.
            self._rx.clear()
            self._tx.clear()
            self._boot()
            self._skip_drains = 1

    def _integrate(self) -> None:
        now = self._clock.now
        dt = now - self._integrated_at
        self._integrated_at = now
        while dt > 1e-9:
            step = min(_INTEGRATION_SUBSTEP_S, dt)
            dt -= step
            self._step_motion(step)

    def _step_motion(self, dt: float) -> None:
        if not self.enabled:
            self.actual_sps = 0.0
            self.desired_sps = 0.0
            self.running = False
            self.stop_requested = False
            return

        if self.running and self.has_target and not self.free_ride:
            delta = self.target - int(round(self.position))
            if delta == 0:
                self._complete_target()
                return
            self.dir_negative = delta < 0

        desired = 0.0
        if not self.stop_requested and self.running:
            desired = self.speed_sps

        if self.has_target and not self.free_ride and desired > 0.0 and self.accel_sps2 > 0.0:
            remaining = self.target - self.position
            stopping_distance = (self.actual_sps * self.actual_sps) / (2.0 * self.accel_sps2)
            if abs(remaining) <= stopping_distance:
                desired = 0.0

        self.desired_sps = desired
        if self.accel_sps2 <= 0.0:
            self.actual_sps = desired
        else:
            delta_speed = self.accel_sps2 * dt
            if self.actual_sps < desired:
                self.actual_sps = min(self.actual_sps + delta_speed, desired)
            elif self.actual_sps > desired:
                self.actual_sps = max(self.actual_sps - delta_speed, desired)

        if self.stop_requested and self.actual_sps <= 0.0:
            self.stop_requested = False
            self.running = False
            self.actual_sps = 0.0
            self.desired_sps = 0.0

        if self.actual_sps > 0.0:
            travel = self.actual_sps * dt
            self.position += -travel if self.dir_negative else travel
            if self.running and self.has_target and not self.free_ride:
                crossed = self.position <= self.target if self.dir_negative else self.position >= self.target
                if crossed:
                    self.position = float(self.target)
                    self._complete_target()

    def _complete_target(self) -> None:
        self.has_target = False
        self.running = False
        self.stop_requested = False
        self.actual_sps = 0.0
        self.desired_sps = 0.0

    def _phase(self) -> str:
        if not self.enabled:
            return "idle"
        if self.actual_sps <= 0.0:
            return "hold"
        if self.desired_sps <= 0.0:
            return "deceleration"
        if self.actual_sps < self.desired_sps:
            return "acceleration"
        if self.actual_sps > self.desired_sps:
            return "deceleration"
        return "running"

    def _reply(self, command: str, ok: bool, pairs: list[tuple[str, str]]) -> None:
        body = "".join(f"{key}={value};" for key, value in pairs)
        line = f"{'1' if ok else '0'};{body}\n".encode("ascii")
        self._tx.extend(self.faults.apply(line, b"\n", command))

    def _error(self, command: str, message: str) -> None:
        self._reply(command, False, [("error", message)])

    def _handle(self, line: str) -> None:
        self._integrate()

        tokens = line.split()
        if not tokens:
            return
        cmd = tokens[0]
        args = tokens[1:]

        if cmd == "status":
            self._reply(cmd, True, [
                ("initialised", "1"),
                ("enabled", "1" if self.enabled else "0"),
                ("mode", "free_ride" if self.free_ride else "target"),
                ("position", str(int(round(self.position)))),
                ("phase", self._phase()),
                ("target", str(self.target)),
                ("target_set", "1" if self.has_target else "0"),
                ("speed", f"{self.speed_sps:.2f}"),
                ("actual_speed", f"{self.actual_sps:.2f}"),
                ("accel_per_s", f"{self.accel_sps2:.2f}"),
                ("power_v", f"{self.power_v:.2f}"),
            ])
        elif cmd == "position":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            self.position = float(value)
            self._reply(cmd, True, [("position", str(int(round(self.position))))])
        elif cmd == "enabled":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            self.enabled = value != 0
            if not self.enabled:
                self.running = False
                self.stop_requested = False
                self.actual_sps = 0.0
                self.desired_sps = 0.0
            self._reply(cmd, True, [("enabled", "1" if self.enabled else "0")])
        elif cmd == "direction":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            self.dir_negative = value != 0
            self._reply(cmd, True, [("direction", "1" if self.dir_negative else "0")])
        elif cmd == "speed":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            if value < 0 or value > _MAX_SPEED_SPS:
                self._error(cmd, "range")
                return
            self.speed_sps = float(value)
            self._reply(cmd, True, [("speed", f"{self.speed_sps:.2f}")])
        elif cmd == "acceleration":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            if value < 0 or value > _MAX_ACCEL_SPS2:
                self._error(cmd, "range")
                return
            self.accel_sps2 = float(value)
            self._reply(cmd, True, [("accel_per_s", f"{self.accel_sps2:.2f}")])
        elif cmd == "delta":
            value = self._parse_long(args)
            if value is None:
                self._error(cmd, "bad_value")
                return
            self.target = int(round(self.position)) + value
            self.has_target = True
            self._reply(cmd, True, [
                ("delta", str(value)),
                ("target", str(self.target)),
                ("target_set", "1"),
            ])
        elif cmd == "mode":
            if not args:
                self._error(cmd, "bad_value")
                return
            if len(args) > 1:
                self._error(cmd, "single_param")
                return
            if args[0] == "free_ride":
                self.free_ride = True
            elif args[0] == "target":
                self.free_ride = False
            else:
                self._error(cmd, "bad_value")
                return
            self._reply(cmd, True, [("mode", "free_ride" if self.free_ride else "target")])
        elif cmd == "run":
            self.running = True
            self.stop_requested = False
            if not self.enabled:
                self.enabled = True
            self._reply(cmd, True, [("running", "1")])
        elif cmd == "stop":
            self.stop_requested = True
            self.running = True
            self._reply(cmd, True, [("stopping", "1")])
        elif cmd in ("set", "get"):
            if not args:
                self._error(cmd, "missing_param")
                return
            if len(args) > 1:
                self._error(cmd, "single_param")
                return
            token = args[0].removeprefix(":")
            # `set` carries a textual value, `get` carries none — unlike every
            # other branch above, where `value` is the parsed long.
            param_value: str | None
            if cmd == "get":
                name, param_value = token, None
            else:
                name, separator, param_value = token.partition("=")
                if not separator or not name or not param_value:
                    self._error(cmd, "bad_param")
                    return
            if name != "microsteps":
                self._error(cmd, "unknown_param")
                return
            if param_value is not None:
                try:
                    requested = int(param_value)
                except ValueError:
                    self._error(cmd, "bad_value")
                    return
                if requested < 1 or requested > 256:
                    self._error(cmd, "range")
                    return
                if requested not in MICROSTEPS_ALLOWED:
                    self._error(cmd, "invalid_microsteps")
                    return
                self.microsteps = requested
            self._reply(cmd, True, [("microsteps", str(self.microsteps))])
        else:
            self._error(cmd, "unknown_cmd")

    @staticmethod
    def _parse_long(args: list[str]) -> int | None:
        if len(args) != 1:
            return None
        try:
            return int(args[0], 10)
        except ValueError:
            return None
