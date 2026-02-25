from dataclasses import dataclass
import logging
import time

from serial_wrapper.wrapper import SerialLine

from .base import Device, Motion, MoveMode, RawStatus


COMMAND_TERMINATOR = "\n"
RESPONSE_DELIMITER = ";"
KEY_VALUE_SEPARATOR = "="
READY_RESPONSE = "ready"
BOOL_VALUES = {"0", "1"}

STEPS_PER_REV = 200
GEAR_RATIO_1 = 44 / 26
GEAR_RATIO_2 = 117.4145
DEGREES_PER_REV = 360
ARCSECONDS_PER_DEGREE = 60 * 60
ARCSECONDS_PER_REV = DEGREES_PER_REV * ARCSECONDS_PER_DEGREE
MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}


class TMC2209DeviceError(Exception):
    pass


class TMC2209ConfigError(TMC2209DeviceError):
    pass


class TMC2209ProtocolError(TMC2209DeviceError):
    pass


class TMC2209CommandError(TMC2209DeviceError):
    pass


@dataclass(frozen=True)
class _Response:
    ok: bool
    values: dict[str, str]
    error: str | None
    raw: str

    @classmethod
    def from_line(cls, line: str) -> "_Response":
        cleaned = line.strip()
        if not cleaned:
            raise TMC2209ProtocolError("empty response")

        tokens = [token for token in cleaned.split(RESPONSE_DELIMITER) if token]
        if not tokens:
            raise TMC2209ProtocolError(f"invalid response: {line!r}")

        ok_prefix = tokens[0]
        if ok_prefix not in BOOL_VALUES:
            raise TMC2209ProtocolError(f"invalid response prefix: {ok_prefix!r}")

        values: dict[str, str] = {}
        for token in tokens[1:]:
            if KEY_VALUE_SEPARATOR not in token:
                raise TMC2209ProtocolError(f"invalid key-value token: {token!r}")
            key, value = token.split(KEY_VALUE_SEPARATOR, 1)
            if not key:
                raise TMC2209ProtocolError(f"empty key in response token: {token!r}")
            values[key] = value

        ok = ok_prefix == "1"
        error = values.get("error") if not ok else None
        return cls(ok=ok, values=values, error=error, raw=cleaned)


PHASE_TO_MOTION = {
    "idle": Motion.IDLE,
    "hold": Motion.HOLD,
    "acceleration": Motion.ACCELERATION,
    "running": Motion.RUNNING,
    "deceleration": Motion.DECELERATION,
}


class TMC2209Device(Device):
    _READY_TIMEOUT_S = 10.0
    _READY_RETRIES = 3

    def __init__(self, serial: SerialLine, microsteps: int = 16) -> None:
        super().__init__(serial)
        self.logger = logging.getLogger("hw.tmc2209")
        if microsteps not in MICROSTEPS_ALLOWED:
            raise TMC2209ConfigError(f"microsteps not allowed: {microsteps!r}")

        self._microsteps = microsteps
        self._current_speed_steps_per_sec: float = 0.0
        self._is_ready = False

        terminator = getattr(self.serial, "terminator", None)
        if not terminator or b"\n" not in terminator:
            raise TMC2209ConfigError("serial terminator must include \\n")

    @property
    def steps_per_rev(self) -> float:
        return STEPS_PER_REV * self._microsteps * GEAR_RATIO_1 * GEAR_RATIO_2

    @property
    def steps_per_arcsecond(self) -> float:
        return self.steps_per_rev / ARCSECONDS_PER_REV

    def connect(self) -> None:
        retries = self._READY_RETRIES
        while retries:
            super().connect()
            self.serial.reset()
            self.serial.read_all_data()

            start = time.monotonic()
            ready = ""
            while not ready:
                ready = self.serial.query(None, timeout=int(self._READY_TIMEOUT_S))
                if time.monotonic() - start > self._READY_TIMEOUT_S:
                    break

            if ready.strip() == READY_RESPONSE:
                self._is_ready = True
                return

            retries -= 1
            self.serial.close()

        raise TMC2209ConfigError("device is not ready")

    def disconnect(self) -> None:
        self._is_ready = False
        super().disconnect()

    def reset(self) -> None:
        self.stop_motor()
        self.set_position(0)
        self._current_speed_steps_per_sec = 0.0

    def to_steps(self, value: float) -> float:
        return value * self.steps_per_arcsecond

    def from_steps(self, steps: float) -> float:
        return steps / self.steps_per_arcsecond

    def current_speed(self) -> float:
        return self._current_speed_steps_per_sec

    def need_stop(self, desired_steps_per_sec: float) -> bool:
        return desired_steps_per_sec == 0

    def change_speed(self, speed_steps_per_sec: float) -> None:
        self._ensure_ready()
        self._current_speed_steps_per_sec = speed_steps_per_sec

        speed_abs = int(round(abs(speed_steps_per_sec)))
        direction = "1" if speed_steps_per_sec < 0 else "0"
        self._transact("mode", ["free_ride"])
        self._transact("direction", [direction])
        self._transact("speed", [str(speed_abs)])

    def move_to_target(self, delta_steps: float) -> None:
        self._ensure_ready()
        delta = int(round(delta_steps))
        if delta == 0:
            return

        direction = "1" if delta < 0 else "0"
        self._transact("mode", ["target"])
        self._transact("direction", [direction])
        self._transact("delta", [str(delta)])

    def start_motor(self) -> None:
        self._ensure_ready()
        if self._current_speed_steps_per_sec == 0:
            return
        self._transact("run")

    def stop_motor(self) -> None:
        self._ensure_ready()
        self._transact("stop")

    def set_position(self, position_steps: float) -> None:
        self._ensure_ready()
        self._transact("position", [str(int(round(position_steps)))])

    def status(self) -> RawStatus:
        self._ensure_ready()
        response = self._transact("status")
        position = int(_require_value(response.values, "position"))
        phase = _require_value(response.values, "phase")
        motion = PHASE_TO_MOTION.get(phase)
        if motion is None:
            raise TMC2209ProtocolError(f"unexpected phase: {phase!r}")
        mode_raw = response.values.get("mode", "free_ride")
        mode = MoveMode.MOVING
        if mode_raw == "target":
            mode = MoveMode.TARGET
        elif motion in {Motion.IDLE, Motion.HOLD}:
            mode = MoveMode.STOP
        return RawStatus(motor_steps=float(position), motion=motion, mode=mode)

    def _ensure_ready(self) -> None:
        if not self._is_ready:
            raise TMC2209ConfigError("device is not ready, call connect() first")

    def _transact(self, command: str, args: list[str] | None = None) -> _Response:
        payload = command
        if args:
            payload = f"{payload} {' '.join(args)}"
        payload = f"{payload}{COMMAND_TERMINATOR}"

        raw = self.serial.query(payload)
        response = _Response.from_line(raw)
        if not response.ok:
            raise TMC2209CommandError(response.error or "tmc2209 command error")
        return response


def _require_value(values: dict[str, str], key: str) -> str:
    if key not in values:
        raise TMC2209ProtocolError(f"missing key in response: {key!r}")
    return values[key]
