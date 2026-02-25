from dataclasses import dataclass
from enum import IntEnum, StrEnum
import logging
import time

from serial_wrapper.wrapper import SerialLine

from .base import Device, Motion, MoveMode, RawStatus


class SkyWatcherDeviceError(Exception):
    pass


class SkyWatcherProtocolError(SkyWatcherDeviceError):
    pass


class SkyWatcherCommandError(SkyWatcherDeviceError):
    pass


class SkyWatcherTimeoutError(SkyWatcherDeviceError):
    pass


class SkyWatcherCommand(StrEnum):
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


class _Direction(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class _SpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


class _SlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


@dataclass(frozen=True)
class _Status:
    running: bool
    initialized: bool
    slew_mode: _SlewMode
    direction: _Direction
    speed_mode: _SpeedMode


class Revu24:
    @staticmethod
    def from_mount(data: str) -> int:
        if len(data) == 2:
            data = f"{data}0000"
        if len(data) < 6:
            raise SkyWatcherProtocolError(f"expected at least 6 hex chars, got {len(data)}")

        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherProtocolError(f"invalid hex data: {data!r}") from exc

    @staticmethod
    def from_int(value: int) -> str:
        if value < 0 or value > 0xFFFFFF:
            raise SkyWatcherProtocolError(f"value out of range 0..{0xFFFFFF}: {value!r}")
        return value.to_bytes(3, "little").hex().upper()


class SkyWatcherDevice(Device):
    _LEADING = ":"
    _TRAILING = "\r"
    _RESPONSE_PREFIX = "="
    _ERROR_PREFIX = "!"

    _TIMEOUT_S = 5.0
    _POLL_INTERVAL_S = 0.2
    _MAX_RETRIES = 3
    _RETRY_SLEEP_S = 0.1

    _POSITION_OFFSET = 0x800000
    _SECONDS_PER_CIRCLE = 24 * 60 * 60
    _HALF_CIRCLE_SECONDS = _SECONDS_PER_CIRCLE / 2
    _STELLAR_DAY = 86164.098903691
    _LOWSPEED_RATE = 128.0
    _LOWSPEED_MARGIN_S = 10 * 60
    _HIGHSPEED_RATE = 800.0
    MIN_RATE = 0.05
    MAX_RATE = 800.0

    def __init__(self, serial: SerialLine) -> None:
        super().__init__(serial)
        self.logger = logging.getLogger("hw.skywatcher")
        self.ra_steps_360: int = 0
        self.ra_steps_worm: int = 0
        self.ra_highspeed_ratio: int = 0
        self._current_speed_steps_per_sec: float = 0.0

        expected_terminator = self._TRAILING.encode(self.serial.encoding)
        if self.serial.terminator != expected_terminator:
            raise SkyWatcherProtocolError(
                f"serial terminator must be {expected_terminator!r}, got {self.serial.terminator!r}"
            )

    def _ensure_ready(self) -> None:
        if self.ra_steps_360 <= 0 or self.ra_steps_worm <= 0 or self.ra_highspeed_ratio <= 0:
            raise SkyWatcherProtocolError("device is not initialized, call connect() first")

    def _transact(self, command: SkyWatcherCommand, arg: str | None = None) -> str:
        payload = f"{self._LEADING}{command.value}1"
        if arg is not None:
            payload = f"{payload}{arg}"
        payload = f"{payload}{self._TRAILING}"

        retries = self._MAX_RETRIES
        while True:
            response = self.serial.query(payload)
            if not response.endswith(self._TRAILING):
                if retries:
                    retries -= 1
                    time.sleep(self._RETRY_SLEEP_S)
                    continue
                raise SkyWatcherProtocolError(f"response does not end with terminator: {response!r}")
            if not response:
                raise SkyWatcherProtocolError("empty response")
            if response[0] == self._ERROR_PREFIX:
                raise SkyWatcherCommandError(response)
            if response[0] != self._RESPONSE_PREFIX:
                if retries:
                    retries -= 1
                    time.sleep(self._RETRY_SLEEP_S)
                    continue
                raise SkyWatcherProtocolError(f"unexpected response prefix: {response!r}")

            cleaned = response.removesuffix(self._TRAILING).removeprefix(self._RESPONSE_PREFIX)
            return cleaned

    @staticmethod
    def _build_motion_mode(
        direction: _Direction,
        speed_mode: _SpeedMode,
        slew_mode: _SlewMode,
    ) -> str:
        if slew_mode == _SlewMode.SLEW:
            motion_mode = "1" if speed_mode == _SpeedMode.LOWSPEED else "3"
        else:
            motion_mode = "2" if speed_mode == _SpeedMode.LOWSPEED else "0"
        direction_mode = "0" if direction == _Direction.FORWARD else "1"
        return f"{motion_mode}{direction_mode}"

    def _get_status(self) -> _Status:
        raw = self._transact(SkyWatcherCommand.INQUIRE_STATUS)
        data = raw.encode("ascii")
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0

        running = bool(b2 & 0x01)
        initialized = bool(b3 & 0x01)
        slew_mode = _SlewMode.SLEW if (b1 & 0x01) else _SlewMode.GOTO
        direction = _Direction.BACKWARD if (b1 & 0x02) else _Direction.FORWARD
        speed_mode = _SpeedMode.HIGHSPEED if (b1 & 0x04) else _SpeedMode.LOWSPEED
        return _Status(
            running=running,
            initialized=initialized,
            slew_mode=slew_mode,
            direction=direction,
            speed_mode=speed_mode,
        )

    def _is_highspeed(self, status: _Status | None = None) -> bool:
        if status is None:
            status = self._get_status()
        return status.speed_mode == _SpeedMode.HIGHSPEED

    def _is_goto(self, status: _Status | None = None) -> bool:
        if status is None:
            status = self._get_status()
        return status.slew_mode == _SlewMode.GOTO

    def _tracking_rate_from_ha_speed(self, speed_ha_seconds_per_sec: float) -> float:
        return speed_ha_seconds_per_sec * self._STELLAR_DAY / self._SECONDS_PER_CIRCLE

    def _ha_speed_from_tracking_rate(self, tracking_rate: float) -> float:
        return tracking_rate * self._SECONDS_PER_CIRCLE / self._STELLAR_DAY

    def _apply_rate(
        self,
        tracking_rate: float,
        slew_mode: _SlewMode,
    ) -> None:
        direction = _Direction.FORWARD if tracking_rate > 0 else _Direction.BACKWARD
        speed_mode = _SpeedMode.LOWSPEED
        effective_rate = tracking_rate
        if abs(tracking_rate) > self._LOWSPEED_RATE:
            effective_rate = tracking_rate / self.ra_highspeed_ratio
            speed_mode = _SpeedMode.HIGHSPEED

        self._transact(
            SkyWatcherCommand.SET_MOTION_MODE,
            self._build_motion_mode(direction, speed_mode, slew_mode),
        )

        period = (
            self._STELLAR_DAY
            * self.ra_steps_worm
            / self.ra_steps_360
            / abs(effective_rate)
        )
        self._transact(SkyWatcherCommand.SET_STEP_PERIOD, Revu24.from_int(int(period)))

    def _initialize(self) -> None:
        status = self._get_status()
        if not status.initialized:
            self._transact(SkyWatcherCommand.INITIALIZE)
            start = time.monotonic()
            while True:
                status = self._get_status()
                if status.initialized:
                    break
                if time.monotonic() - start > self._TIMEOUT_S:
                    raise SkyWatcherTimeoutError(f"mount initialization timeout after {self._TIMEOUT_S}s")
                time.sleep(self._POLL_INTERVAL_S)

        self.ra_steps_360 = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_CPR))
        self.ra_steps_worm = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_TIMER_FREQ))
        self.ra_highspeed_ratio = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_HIGHSPEED_RATIO))

    def to_steps(self, value: float) -> float:
        self._ensure_ready()
        return value / self._SECONDS_PER_CIRCLE * self.ra_steps_360

    def from_steps(self, steps: float) -> float:
        self._ensure_ready()
        return steps / self.ra_steps_360 * self._SECONDS_PER_CIRCLE

    def current_speed(self) -> float:
        return self._current_speed_steps_per_sec

    def stop_motor(self) -> None:
        self._ensure_ready()
        self._transact(SkyWatcherCommand.STOP_MOTION)

    def start_motor(self) -> None:
        self._ensure_ready()
        if self._current_speed_steps_per_sec == 0:
            return
        self._transact(SkyWatcherCommand.START_MOTION)

    def set_position(self, position_steps: float) -> None:
        self._ensure_ready()
        ticks = int(round(position_steps)) % self.ra_steps_360
        ticks = (ticks + self._POSITION_OFFSET) % self.ra_steps_360
        self._transact(SkyWatcherCommand.SET_AXIS_POSITION, Revu24.from_int(ticks))

    def status(self) -> RawStatus:
        self._ensure_ready()
        position_raw = self._transact(SkyWatcherCommand.INQUIRE_POSITION)
        position_ticks = (Revu24.from_mount(position_raw) - self._POSITION_OFFSET) % self.ra_steps_360
        sw_status = self._get_status()
        motion = Motion.RUNNING if sw_status.running else Motion.HOLD
        mode = MoveMode.MOVING
        if sw_status.slew_mode == _SlewMode.GOTO:
            mode = MoveMode.TARGET
        elif not sw_status.running:
            mode = MoveMode.STOP
        return RawStatus(motor_steps=float(position_ticks), motion=motion, mode=mode)

    def change_speed(self, speed_steps_per_sec: float) -> None:
        self._ensure_ready()
        self._current_speed_steps_per_sec = speed_steps_per_sec
        if speed_steps_per_sec == 0:
            self.stop_motor()
            return

        speed_ha_seconds_per_sec = (
            speed_steps_per_sec / self.ra_steps_360 * self._SECONDS_PER_CIRCLE
        )
        tracking_rate = self._tracking_rate_from_ha_speed(speed_ha_seconds_per_sec)
        if not (self.MIN_RATE < abs(tracking_rate) < self.MAX_RATE):
            self.logger.warning(
                "Speed rate out of limits: rate=%.3f min=%.3f max=%.3f",
                tracking_rate,
                self.MIN_RATE,
                self.MAX_RATE,
            )

        self._apply_rate(tracking_rate, _SlewMode.SLEW)

    def move_to_target(self, delta_steps: float) -> None:
        self._ensure_ready()
        delta_ha_seconds = self.from_steps(delta_steps)

        if delta_ha_seconds > self._HALF_CIRCLE_SECONDS:
            delta_ha_seconds -= self._SECONDS_PER_CIRCLE
        elif delta_ha_seconds < -self._HALF_CIRCLE_SECONDS:
            delta_ha_seconds += self._SECONDS_PER_CIRCLE

        delta_abs_seconds = abs(delta_ha_seconds)
        if delta_abs_seconds == 0:
            self._current_speed_steps_per_sec = 0.0
            return

        goto_tracking_rate = self._HIGHSPEED_RATE
        if delta_abs_seconds <= self._LOWSPEED_MARGIN_S:
            goto_tracking_rate = self._LOWSPEED_RATE

        if delta_ha_seconds < 0:
            goto_tracking_rate *= -1

        goto_speed_ha_seconds_per_sec = self._ha_speed_from_tracking_rate(goto_tracking_rate)
        self._current_speed_steps_per_sec = self.to_steps(goto_speed_ha_seconds_per_sec)
        self._apply_rate(goto_tracking_rate, _SlewMode.GOTO)

        delta_ticks = int(round(self.to_steps(delta_abs_seconds))) % self.ra_steps_360
        self._transact(
            SkyWatcherCommand.SET_GOTO_TARGET_INCREMENT,
            Revu24.from_int(delta_ticks),
        )
        self._transact(
            SkyWatcherCommand.SET_BREAK_POINT_INCREMENT,
            Revu24.from_int(min(200, delta_ticks)),
        )

    def need_stop(self, desired_steps_per_sec: float) -> bool:
        if desired_steps_per_sec == 0:
            return True

        self._ensure_ready()
        status = self._get_status()
        return self._is_highspeed(status) or self._is_goto(status)

    def reset(self) -> None:
        self.stop_motor()
        self._current_speed_steps_per_sec = 0.0
        self._transact(SkyWatcherCommand.INITIALIZE)
        self._initialize()

    def connect(self) -> None:
        super().connect()
        self._initialize()
