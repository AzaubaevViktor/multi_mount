import dataclasses
import logging
import time
from enum import IntEnum, StrEnum

from serial_wrapper.wrapper import SerialLine
from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.motor import MotionMode, Motor, MotorDirection, MotorStateError, MotorStatus, MotorStopRequire
from sky.physics import Ha, HaPerSecond
from skywatcher.protocol import Protocol


class SkyWatcherMotorError(Exception):
    def __init__(self, message: str) -> None:
        self.message = message
        super().__init__(message)


class SkyWatcherMotorProtocolError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorCommandError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorTimeoutError(SkyWatcherMotorError, TimeoutError):
    pass


class _Command(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    SET_STEP_PERIOD = "I"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INITIALIZE = "F"


class _Axis(StrEnum):
    RA = "1"


class _Direction(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class _SlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class _SpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass(frozen=True)
class _Status:
    raw: int
    running: bool
    initialized: bool
    slew_mode: _SlewMode
    direction: _Direction
    speed_mode: _SpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "_Status":
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0
        raw = b2 | (b1 << 8) | (b3 << 16)
        return cls(
            raw=raw,
            running=bool(b2 & 0x01),
            initialized=bool(b3 & 0x01),
            slew_mode=_SlewMode.SLEW if (b1 & 0x01) else _SlewMode.GOTO,
            direction=_Direction.BACKWARD if (b1 & 0x02) else _Direction.FORWARD,
            speed_mode=_SpeedMode.HIGHSPEED if (b1 & 0x04) else _SpeedMode.LOWSPEED,
        )


@dataclasses.dataclass(frozen=True)
class _MotionStatus:
    slew_mode: _SlewMode
    direction: _Direction
    speed_mode: _SpeedMode

    def to_command(self) -> str:
        if self.slew_mode == _SlewMode.SLEW:
            motion_mode = "1" if self.speed_mode == _SpeedMode.LOWSPEED else "3"
        else:
            motion_mode = "2" if self.speed_mode == _SpeedMode.LOWSPEED else "0"
        direction_mode = "0" if self.direction == _Direction.FORWARD else "1"
        return f"{motion_mode}{direction_mode}"


class _Revu24:
    @staticmethod
    def from_mount(data: str) -> int:
        if len(data) == 2:
            data = f"{data}0000"
        if len(data) < 6:
            raise SkyWatcherMotorProtocolError(f"expected at least 6 hex chars, got {len(data)}")
        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherMotorProtocolError(f"invalid hex data: {data!r}") from exc

    @staticmethod
    def from_int(value: int) -> str:
        if value < 0 or value > 0xFFFFFF:
            raise SkyWatcherMotorProtocolError(f"expected value in range 0..{0xFFFFFF}, got {value}")
        return value.to_bytes(3, "little").hex().upper()


class SkyWatcherMotor(Motor[Ha, HaPerSecond]):
    FORWARD_POSITION_SIGN = -1

    _POSITION_OFFSET = 0x800000
    _LOWSPEED_MARGIN = Ha(10 * 60)
    _LOWSPEED_SPEED = STELLAR_SPEED * 128
    _HIGHSPEED_SPEED = STELLAR_SPEED * 800

    def __init__(self, serial: SerialLine) -> None:
        self._serial = serial
        self._logger = logging.getLogger(type(self).__name__)
        self._is_connected = False
        self._steps_360 = 0
        self._steps_worm = 0
        self._highspeed_ratio = 0
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target: int | None = None
        self._zero_target_pending = False
        self._mount_position_cache = Ha(0)
        self._mount_position_cache_updated = 0.0

    def connect(self):
        if self._serial.terminator != Protocol.ANSWER_END_BYTE:
            raise SkyWatcherMotorProtocolError(
                f"invalid SerialLine terminator: expected {Protocol.ANSWER_END_BYTE!r}, got {self._serial.terminator!r}"
            )
        self._serial.connect()
        self._transact(_Command.INITIALIZE)
        self._steps_360 = _Revu24.from_mount(self._transact(_Command.INQUIRE_CPR))
        self._steps_worm = _Revu24.from_mount(self._transact(_Command.INQUIRE_TIMER_FREQ))
        self._highspeed_ratio = _Revu24.from_mount(self._transact(_Command.INQUIRE_HIGHSPEED_RATIO))
        if self._steps_360 <= 0 or self._steps_worm <= 0 or self._highspeed_ratio <= 0:
            raise SkyWatcherMotorProtocolError(
                f"invalid mount config: steps_360={self._steps_360} steps_worm={self._steps_worm} highspeed_ratio={self._highspeed_ratio}"
            )
        self._is_connected = True

    def disconnect(self) -> bool:
        self._is_connected = False
        self._serial.close()
        return True

    def status(self) -> MotorStatus:
        status = self._get_status()
        if status.running and status.slew_mode == _SlewMode.GOTO:
            motion_mode = MotionMode.TARGET
        elif status.running:
            motion_mode = MotionMode.RUN
        else:
            motion_mode = MotionMode.IDLE
        if not status.running:
            direction = MotorDirection.STOP
        elif status.direction == _Direction.FORWARD:
            direction = MotorDirection.FORWARD
        else:
            direction = MotorDirection.BACKWARD
        return MotorStatus(
            is_connected=self._is_connected,
            steps=self.convert_position_to_steps(self._get_position()),
            motion_mode=motion_mode,
            speed_sps=self._last_speed_sps,
            accel_sps=None,
            direction=direction,
            target=self._last_target if status.slew_mode == _SlewMode.GOTO else None,
            microsteps=None,
        )

    def set_steps(self, steps: int) -> bool:
        self._ensure_not_goto(self._get_status(), "cannot change steps while GOTO is in progress")
        self._transact(_Command.SET_AXIS_POSITION, _Revu24.from_int((steps + self._POSITION_OFFSET) % self._steps_360))
        self._mount_position_cache = self.convert_steps_to_position(steps)
        self._mount_position_cache_updated = time.monotonic()
        return True

    def set_speed(self, steps_per_second: int) -> int:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change speed while GOTO is in progress")
        if steps_per_second <= 0:
            raise ValueError(f"steps_per_second must be positive, got {steps_per_second}")
        target_speed_mode = self._get_speed_mode_for_speed_sps(steps_per_second)
        self._set_motion(
            _MotionStatus(
                slew_mode=status.slew_mode,
                direction=status.direction,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        period = self._period_from_speed_sps(steps_per_second)
        self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(period))
        self._last_speed_sps = self._speed_sps_from_period(period, target_speed_mode)
        return self._last_speed_sps

    def set_acceleration(self, steps_per_second_square: float) -> bool:
        self._ensure_not_goto(self._get_status(), "cannot change acceleration while GOTO is in progress")
        return False

    def set_direction(self, direction: MotorDirection) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change direction while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change direction while motor is moving")
        if direction == MotorDirection.STOP:
            self._last_direction = direction
            return True
        self._set_motion(
            _MotionStatus(
                slew_mode=_SlewMode.SLEW,
                direction=_Direction.FORWARD if direction == MotorDirection.FORWARD else _Direction.BACKWARD,
                speed_mode=status.speed_mode,
            ),
            status,
        )
        self._last_direction = direction
        return True

    def set_delta(self, delta_steps: int) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change target while GOTO is in progress")
        delta = self.convert_steps_to_position(delta_steps).moving_wrap()
        if delta == Ha(0):
            self._last_target = None
            self._zero_target_pending = True
            return True
        self._zero_target_pending = False
        speed = self._get_goto_speed(delta)
        target_speed_mode = self._get_speed_mode_for_speed_sps(self.convert_speed_to_steps_per_second(abs(speed)))
        self._set_motion(
            _MotionStatus(
                slew_mode=_SlewMode.GOTO,
                direction=_Direction.FORWARD if speed > HaPerSecond(0) else _Direction.BACKWARD,
                speed_mode=target_speed_mode,
            ),
            status,
        )
        self._last_speed_sps = self.convert_speed_to_steps_per_second(abs(speed))
        self._transact(_Command.SET_STEP_PERIOD, _Revu24.from_int(self._period_from_speed_sps(self._last_speed_sps)))
        self._last_target = abs(delta_steps) % self._steps_360
        self._transact(_Command.SET_GOTO_TARGET_INCREMENT, _Revu24.from_int(self._last_target))
        self._transact(_Command.SET_BREAK_POINT_INCREMENT, _Revu24.from_int(min(200, self._last_target)))
        self._last_direction = MotorDirection.FORWARD if speed > HaPerSecond(0) else MotorDirection.BACKWARD
        return True

    def get_speed_sps_by_delta(self, delta_steps: int) -> int:
        return self.convert_speed_to_steps_per_second(self._get_goto_speed(self.convert_steps_to_position(delta_steps).moving_wrap()))

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond:
        if speed_sps < 0:
            raise ValueError(f"speed_sps must be non-negative, got {speed_sps}")
        return HaPerSecond(float(speed_sps) * 24 * 60 * 60 / self._steps_360)

    def set_motion_mode(self, motion_mode: MotionMode) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change motion mode while GOTO is in progress")
        if motion_mode == MotionMode.RUN:
            self._set_motion(
                _MotionStatus(
                    slew_mode=_SlewMode.SLEW,
                    direction=_Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else _Direction.BACKWARD,
                    speed_mode=status.speed_mode,
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.TARGET:
            self._set_motion(
                _MotionStatus(
                    slew_mode=_SlewMode.GOTO,
                    direction=_Direction.FORWARD if self._last_direction != MotorDirection.BACKWARD else _Direction.BACKWARD,
                    speed_mode=status.speed_mode,
                ),
                status,
            )
            return True
        if motion_mode == MotionMode.IDLE:
            return True
        raise MotorStateError(f"unsupported motion mode: {motion_mode}")

    def set_microsteps(self, microsteps: int) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot change microsteps while GOTO is in progress")
        if status.running:
            raise MotorStopRequire("cannot change microsteps while motor is moving")
        return False

    def convert_position_to_steps(self, position: Ha) -> int:
        self._ensure_geometry_ready()
        return int(round(float(position) / (24 * 60 * 60) * self._steps_360))

    def convert_steps_to_position(self, steps: int) -> Ha:
        self._ensure_geometry_ready()
        return Ha(steps / self._steps_360 * 24 * 60 * 60)

    def convert_speed_to_steps_per_second(self, speed: HaPerSecond) -> int:
        self._ensure_geometry_ready()
        return int(round(abs(float(speed)) * self._steps_360 / (24 * 60 * 60)))

    def run(self) -> bool:
        status = self._get_status()
        self._ensure_not_goto(status, "cannot run while GOTO is in progress")
        if self._zero_target_pending:
            self._zero_target_pending = False
            return True
        if self._last_target is not None and status.slew_mode != _SlewMode.GOTO:
            raise MotorStateError("cannot run before motor motion mode is switched")
        self._transact(_Command.START_MOTION)
        return True

    def stop(self) -> bool:
        self._transact(_Command.STOP_MOTION)
        self._last_target = None
        self._zero_target_pending = False
        return True

    def wait_till_stop(self, do_stop: bool = True, timeout_s: float | None = None) -> None:
        if do_stop:
            self.stop()
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        while self._get_status().running:
            if deadline is not None and time.monotonic() > deadline:
                raise SkyWatcherMotorTimeoutError(f"motor did not stop within {timeout_s}s")
            time.sleep(0.2)

    def reset(self) -> None:
        self.stop()
        self._last_speed_sps = 0
        self._last_direction = MotorDirection.STOP
        self._last_target = None
        self._zero_target_pending = False

    def _get_status(self) -> _Status:
        return _Status.from_bytes(self._transact(_Command.INQUIRE_STATUS).encode("ascii"))

    def _get_position(self) -> Ha:
        self._ensure_geometry_ready()
        if time.monotonic() - self._mount_position_cache_updated <= 0.25:
            return self._mount_position_cache
        ticks = (_Revu24.from_mount(self._transact(_Command.INQUIRE_POSITION)) - self._POSITION_OFFSET) % self._steps_360
        self._mount_position_cache = self.convert_steps_to_position(ticks).wrap()
        self._mount_position_cache_updated = time.monotonic()
        return self._mount_position_cache

    def _set_motion(self, target: _MotionStatus, current: _Status) -> None:
        if current.running and (
            current.slew_mode != target.slew_mode
            or current.direction != target.direction
            or current.speed_mode != target.speed_mode
        ):
            raise MotorStopRequire("cannot change motion mode while motor is moving")
        self._transact(_Command.SET_MOTION_MODE, target.to_command())

    def _get_goto_speed(self, delta: Ha) -> HaPerSecond:
        speed = self._HIGHSPEED_SPEED if abs(delta) > self._LOWSPEED_MARGIN else self._LOWSPEED_SPEED
        return speed if delta >= Ha(0) else -speed

    def _get_speed_mode_for_speed_sps(self, speed_sps: int) -> _SpeedMode:
        return _SpeedMode.HIGHSPEED if speed_sps > self.convert_speed_to_steps_per_second(self._LOWSPEED_SPEED) else _SpeedMode.LOWSPEED

    def _period_from_speed_sps(self, speed_sps: int) -> int:
        self._ensure_geometry_ready()
        if speed_sps <= 0:
            raise MotorStateError("speed must be positive")
        rate = speed_sps * (24 * 60 * 60) / self._steps_360 / float(STELLAR_SPEED)
        if rate > self._highspeed_ratio:
            rate /= self._highspeed_ratio
        return int(STELLAR_DAY * self._steps_worm / self._steps_360 / rate)

    def _speed_sps_from_period(self, period: int, speed_mode: _SpeedMode) -> int:
        self._ensure_geometry_ready()
        if period <= 0:
            raise MotorStateError("period must be positive")
        rate = float(STELLAR_DAY) * self._steps_worm / self._steps_360 / period
        if speed_mode == _SpeedMode.HIGHSPEED:
            rate *= self._highspeed_ratio
        return int(round(rate * float(STELLAR_SPEED) * self._steps_360 / (24 * 60 * 60)))

    def _ensure_not_goto(self, status: _Status, message: str) -> None:
        if status.running and status.slew_mode == _SlewMode.GOTO:
            raise MotorStopRequire(message)

    def _ensure_geometry_ready(self) -> None:
        if self._steps_360 <= 0:
            raise SkyWatcherMotorProtocolError("motor geometry is not initialized")

    REPEATS = 3
    def _transact(self, command: _Command, arg: str | None = None) -> str:
        payload = f"{Protocol.COMMAND_PREFIX}{command.value}{_Axis.RA}{arg or ''}{Protocol.COMMAND_TERMINATOR}"
        count = self.REPEATS
        response = None
        while count > 0:
            try:
                response = self._serial.query(
                    payload,
                    response_prefixes=(Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE),
                )
                if not response:
                    raise SkyWatcherMotorProtocolError(f"empty response: {response!r}")
                if not response.endswith(Protocol.ANSWER_END):
                    raise SkyWatcherMotorProtocolError(f"unterminated response: {response!r}")
                if response[0] == Protocol.COMMAND_ERROR_PREFIX:
                    raise SkyWatcherMotorCommandError(f"command error: {response!r}")
                if response[0] != Protocol.RESPONSE_PREFIX:
                    raise SkyWatcherMotorProtocolError(f"invalid response: {response!r}")

                return response[1:-len(Protocol.ANSWER_END)]

            except SkyWatcherMotorProtocolError:
                self._logger.exception("While quering %s(%s) `%s` -> `%s`, %d last", command.name, arg, payload, response, count)
                self._serial.drop_buffers()
                data = self._serial.read_all_data(timeout=.5)
                if data is not None:
                    self._logger.info("Received data: %s", data)
                count -= 1
                if count == 0:
                    raise
                time.sleep(0.1)
        raise SkyWatcherMotorProtocolError(f"failed to execute command {command} with payload {payload}")
