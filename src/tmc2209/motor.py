import dataclasses
import logging
import time
from enum import StrEnum

from serial_wrapper.wrapper import SerialLine
from sky.motor import MotionMode, Motor, MotorDirection, MotorStateError, MotorStatus, MotorStopRequire
from sky.physics import Dec, DecPerSecond

COMMAND_TERMINATOR = "\n"
RESPONSE_DELIMITER = ";"
KEY_VALUE_SEPARATOR = "="
MICROSTEPS_ALLOWED = {1, 2, 4, 8, 16, 32, 64, 128, 256}
DEGREES_PER_REV = 360.0
STEPS_PER_REV = 200
GEAR_RATIO_1 = 44 / 26
GEAR_RATIO_2 = 117.4145


class TMC2209MotorError(Exception):
    pass


class TMC2209MotorProtocolError(TMC2209MotorError):
    pass


class TMC2209MotorCommandError(TMC2209MotorError):
    pass


class _Phase(StrEnum):
    IDLE = "idle"
    HOLD = "hold"
    ACCELERATION = "acceleration"
    RUNNING = "running"
    DECELERATION = "deceleration"


class _Mode(StrEnum):
    TARGET = "target"
    FREE_RIDE = "free_ride"


@dataclasses.dataclass(frozen=True)
class _Response:
    ok: bool
    values: dict[str, str]
    error: str | None

    @classmethod
    def from_line(cls, line: str) -> "_Response":
        cleaned = line.strip()
        tokens = [token for token in cleaned.split(RESPONSE_DELIMITER) if token]
        if not tokens or tokens[0] not in {"0", "1"}:
            raise TMC2209MotorProtocolError(f"invalid response: {line!r}")
        values: dict[str, str] = {}
        for token in tokens[1:]:
            if KEY_VALUE_SEPARATOR not in token:
                raise TMC2209MotorProtocolError(f"missing separator in token: {token!r}")
            key, value = token.split(KEY_VALUE_SEPARATOR, 1)
            if not key or value == "":
                raise TMC2209MotorProtocolError(f"invalid key-value token: {token!r}")
            values[key] = value
        ok = tokens[0] == "1"
        return cls(ok=ok, values=values, error=values.get("error"))


@dataclasses.dataclass(frozen=True)
class _Status:
    initialised: bool
    enabled: bool
    mode: _Mode
    position: int
    phase: _Phase
    target: int
    target_set: bool
    speed_sps: float
    actual_speed_sps: float
    accel_steps_per_s: float

    @classmethod
    def from_response(cls, response: _Response) -> "_Status":
        return cls(
            initialised=response.values["initialised"] == "1",
            enabled=response.values["enabled"] == "1",
            mode=_Mode(response.values.get("mode", _Mode.TARGET.value)),
            position=int(response.values["position"]),
            phase=_Phase(response.values["phase"]),
            target=int(response.values["target"]),
            target_set=response.values["target_set"] == "1",
            speed_sps=float(response.values["speed"]),
            actual_speed_sps=float(response.values["actual_speed"]),
            accel_steps_per_s=float(response.values["accel_per_s"]),
        )


class TMC2209Motor(Motor[Dec, DecPerSecond]):
    FORWARD_POSITION_SIGN = 1
    _READY_RETRIES = 3
    _READY_TIMEOUT_S = 10.0

    def __init__(self, serial: SerialLine) -> None:
        self._serial = serial
        self._logger = logging.getLogger(type(self).__name__)
        self._microsteps = 16
        self._is_connected = False
        self._direction = MotorDirection.STOP

    def connect(self):
        ready = ""
        for attempt in range(self._READY_RETRIES):
            self._serial.connect()
            self._serial.reset()
            self._serial.read_all_data()

            deadline = time.monotonic() + self._READY_TIMEOUT_S
            while time.monotonic() < deadline:
                ready = self._serial.query(None, timeout=1)
                if ready and ready.strip() == "ready":
                    self._is_connected = True
                    return

            self._serial.close()
            if attempt < self._READY_RETRIES - 1:
                time.sleep(0.5)

        raise TMC2209MotorProtocolError(f"device not ready: {ready!r}")

    def disconnect(self) -> bool:
        self._is_connected = False
        self._serial.close()
        return True

    def status(self) -> MotorStatus:
        status = self._status()
        if status.phase == _Phase.IDLE:
            motion_mode = MotionMode.IDLE
        elif status.mode == _Mode.TARGET and status.phase in (_Phase.ACCELERATION, _Phase.RUNNING, _Phase.DECELERATION):
            motion_mode = MotionMode.TARGET
        elif status.phase == _Phase.ACCELERATION:
            motion_mode = MotionMode.ACCELERATION
        elif status.phase == _Phase.DECELERATION:
            motion_mode = MotionMode.DECELERATION
        else:
            motion_mode = MotionMode.RUN

        if status.phase in (_Phase.IDLE, _Phase.HOLD):
            direction = MotorDirection.STOP
        else:
            direction = self._direction

        return MotorStatus(
            is_connected=self._is_connected,
            steps=status.position,
            motion_mode=motion_mode,
            speed_sps=int(round(abs(status.speed_sps))),
            accel_sps=int(round(status.accel_steps_per_s)),
            direction=direction,
            target=status.target if status.target_set else None,
            microsteps=self._microsteps,
        )

    def set_steps(self, steps: int) -> bool:
        self._ensure_not_goto(self._status(), "cannot change steps while GOTO is in progress")
        self._transact("position", [str(steps)])
        return True

    def set_speed(self, steps_per_second: int) -> int:
        self._ensure_not_goto(self._status(), "cannot change speed while GOTO is in progress")
        if steps_per_second < 0:
            raise ValueError(f"steps_per_second must be non-negative, got {steps_per_second}")
        speed = int(round(steps_per_second))
        self._transact("speed", [str(speed)])
        return speed

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        self._ensure_not_goto(self._status(), "cannot change acceleration while GOTO is in progress")
        acceleration = int(round(steps_per_second_square))
        if acceleration < 0:
            raise ValueError(f"steps_per_second_square must be non-negative, got {steps_per_second_square}")
        self._transact("acceleration", [str(acceleration)])
        return True

    def set_direction(self, direction: MotorDirection) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change direction while GOTO is in progress")
        if status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire("cannot change direction while motor is moving")
        if direction == MotorDirection.STOP:
            return True
        self._transact("direction", ["1" if direction == MotorDirection.BACKWARD else "0"])
        self._direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        self._ensure_not_goto(self._status(), "cannot change target while GOTO is in progress")
        self._transact("delta", [str(delta_steps)])
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return min(max(abs(delta_steps), 1), 6000)

    def get_speed_by_speed_sps(self, speed_sps: int) -> DecPerSecond:
        if speed_sps < 0:
            raise ValueError(f"speed_sps must be non-negative, got {speed_sps}")
        return self.convert_steps_to_speed(speed_sps)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change motion mode while GOTO is in progress")
        if motion_mode in (MotionMode.IDLE, MotionMode.RUN):
            self._transact("mode", [_Mode.FREE_RIDE.value])
            return True
        if motion_mode == MotionMode.TARGET:
            self._transact("mode", [_Mode.TARGET.value])
            return True
        raise MotorStateError(f"unsupported motion mode: {motion_mode}")

    def set_microsteps(self, microsteps: int) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot change microsteps while GOTO is in progress")
        if status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire("cannot change microsteps while motor is moving")
        if microsteps not in MICROSTEPS_ALLOWED:
            raise ValueError(f"microsteps not allowed: {microsteps}")
        self._transact("set", [f"microsteps={microsteps}"])
        self._microsteps = microsteps
        return True

    def convert_position_to_steps(self, position: Dec) -> int:
        return int(round(float(position) * self._steps_per_arcsecond()))

    def convert_steps_to_position(self, steps: int) -> Dec:
        return Dec(float(steps) / self._steps_per_arcsecond())

    def convert_speed_to_steps_per_second(self, speed: DecPerSecond) -> int:
        return int(round(abs(float(speed)) * self._steps_per_arcsecond()))

    def run(self) -> bool:
        status = self._status()
        self._ensure_not_goto(status, "cannot run while GOTO is in progress")
        if status.target_set and status.mode != _Mode.TARGET:
            raise MotorStateError("cannot run before motor motion mode is switched")
        self._transact("run")
        return True

    def stop(self) -> bool:
        self._transact("stop")
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        if do_stop:
            self.stop()
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while True:
            status = self._status()
            if status.phase in (_Phase.IDLE, _Phase.HOLD):
                return
            if deadline is not None and time.monotonic() > deadline:
                raise TimeoutError(f"motor did not stop within {timeout_s}s")
            time.sleep(0.05)

    def reset(self) -> None:
        self.wait_till_stop(do_stop=True)

    def convert_steps_to_speed(self, speed_sps: int | float) -> DecPerSecond:
        return DecPerSecond(float(speed_sps) / self._steps_per_arcsecond())

    def _steps_per_arcsecond(self) -> float:
        return STEPS_PER_REV * self._microsteps * GEAR_RATIO_1 * GEAR_RATIO_2 / (DEGREES_PER_REV * 60 * 60)

    def _ensure_not_goto(self, status: _Status, message: str) -> None:
        if status.mode == _Mode.TARGET and status.phase not in (_Phase.IDLE, _Phase.HOLD):
            raise MotorStopRequire(message)

    def _status(self) -> _Status:
        return _Status.from_response(self._transact("status"))

    def _transact(self, command: str, args: list[str] | None = None) -> _Response:
        payload = command if not args else f"{command} {' '.join(args)}"
        count = 3
        response = None
        while count > 0:
            try:
                response = _Response.from_line(self._serial.query(f"{payload}{COMMAND_TERMINATOR}"))
                if not response.ok:
                    raise TMC2209MotorCommandError(response.error or "tmc2209 error")
                return response
            except TMC2209MotorCommandError:
                count -= 1
                if count == 0:
                    raise
                self._logger.exception("TMC2209 WHILE TRANSACTING: %s(%s) `%s` -> `%s`, %d last", command, args, payload, response, count)
                self._serial.drop_buffers()
                data = self._serial.read_all_data(timeout=.5)
                if data is not None:
                    self._logger.info("Received data: %s", data)
                time.sleep(0.1)
