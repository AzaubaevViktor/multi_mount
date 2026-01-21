from __future__ import annotations
import dataclasses
import logging
import time
from enum import IntEnum, StrEnum
from typing import Optional

from coords import clamp
from serial_prims import SerialLineDevice

LOGGER = logging.getLogger("skywatcher")


@dataclasses.dataclass
class SkyWatcherAxisInfo:
    cpr: int = 0
    timer_freq: int = 0
    last_pos: int = 0
    updated_monotonic: float = 0.0
    last_status: Optional["SkyWatcherStatus"] = None


class SkyWatcherAxis(IntEnum):
    RA = 0
    DEC = 1

    @classmethod
    def from_channel(cls, value: int | str) -> "SkyWatcherAxis":
        # LOGGER.info("axis_channel value=%r", value)
        if isinstance(value, str):
            value = value.strip()
        if value in (1, "1"):
            return cls.RA
        if value in (2, "2"):
            return cls.DEC
        raise ValueError(f"invalid axis channel {value!r}, expected 1 or 2")

    def to_bytes(self) -> bytes:
        # LOGGER.info("axis_bytes axis=%s", self)
        if self.value not in (0, 1):
            raise ValueError(f"invalid axis {self.value!r}, expected 0 or 1")
        return str(self.value + 1).encode("ascii")

class SkyWatcherDirection(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class SkyWatcherSlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class SkyWatcherSpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


class SkyWatcherCommand(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    SET_STEP_PERIOD = "I"
    SET_GOTO_TARGET = "S"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INSTANT_STOP = "L"
    INITIALIZE = "F"


class SkyWatcherConstants:
    ZERO_RATE = 0.0
    MIN_RATE = 0.05
    MAX_RATE = 800.0
    LOWSPEED_RATE = 128.0
    SIDEREAL_DAY_S = 86164.0905
    DEGREES_PER_REV = 360.0
    ARCSEC_PER_DEG = 3600.0
    SIDEREAL_RATE_DEG_S = DEGREES_PER_REV / SIDEREAL_DAY_S
    SIDEREAL_SPEED_ARCSEC_S = (DEGREES_PER_REV * ARCSEC_PER_DEG) / SIDEREAL_DAY_S
    SIDEREAL_RATE_MULT = 1.0
    MIN_STEP_PERIOD = 1
    MAX_STEP_PERIOD = (1 << 24) - 1


class SkyWatcherRateError(Exception):
    pass


class SkyWatcherTrackingError(Exception):
    pass


class SkyWatcherRevu24Error(Exception):
    pass


class SkyWatcherRevu24Constants:
    HEX_LENGTH = 6
    HEX_BASE = 16
    NIBBLE_BITS = 4
    MIN_VALUE = 0
    MAX_VALUE = (1 << 24) - 1
    DECIMAL_OFFSET = 10
    ENCODING = "ascii"
    ENCODE_ORDER = (4, 5, 2, 3, 0, 1)
    ASCII_0 = ord("0")
    ASCII_9 = ord("9")
    ASCII_A = ord("A")
    ASCII_F = ord("F")
    ASCII_a = ord("a")
    ASCII_f = ord("f")


@dataclasses.dataclass
class SkyWatcherRevu24:
    _raw: Optional[bytes] = None
    _value: Optional[int] = None

    def __post_init__(self) -> None:
        if self._raw is None and self._value is None:
            raise SkyWatcherRevu24Error("revu24 requires raw data or value.")
        if self._raw is not None and self._value is not None:
            raise SkyWatcherRevu24Error("revu24 raw data and value are mutually exclusive.")
        if self._raw is not None:
            if len(self._raw) < SkyWatcherRevu24Constants.HEX_LENGTH:
                raise SkyWatcherRevu24Error(f"revu24 data too short: {self._raw!r}")
            self._raw = self._raw[: SkyWatcherRevu24Constants.HEX_LENGTH]
        if self._value is not None:
            if self._value < SkyWatcherRevu24Constants.MIN_VALUE or self._value > SkyWatcherRevu24Constants.MAX_VALUE:
                raise SkyWatcherRevu24Error(f"revu24 value out of range: {self._value!r}")

    @classmethod
    def from_bytes(cls, data: bytes) -> "SkyWatcherRevu24":
        return cls(_raw=data)

    @classmethod
    def from_int(cls, value: int) -> "SkyWatcherRevu24":
        return cls(_value=int(value))

    @property
    def raw(self) -> bytes:
        if self._raw is None:
            self._raw = self._encode_value(self._value)
        return self._raw

    @property
    def value(self) -> int:
        if self._value is None:
            self._value = self._decode_raw(self._raw)
        return self._value

    def to_ascii(self) -> str:
        return self.raw.decode(SkyWatcherRevu24Constants.ENCODING)

    @staticmethod
    def _decode_raw(data: Optional[bytes]) -> int:
        if data is None:
            raise SkyWatcherRevu24Error("revu24 raw data is required for decode.")
        if len(data) < SkyWatcherRevu24Constants.HEX_LENGTH:
            raise SkyWatcherRevu24Error(f"revu24 data too short: {data!r}")
        res = 0
        for idx in SkyWatcherRevu24Constants.ENCODE_ORDER:
            res = (res << SkyWatcherRevu24Constants.NIBBLE_BITS) | SkyWatcherRevu24._hex_val(data[idx])
        return res

    @staticmethod
    def _encode_value(value: Optional[int]) -> bytes:
        if value is None:
            raise SkyWatcherRevu24Error("revu24 value is required for encode.")
        if value < SkyWatcherRevu24Constants.MIN_VALUE or value > SkyWatcherRevu24Constants.MAX_VALUE:
            raise SkyWatcherRevu24Error(f"revu24 value out of range: {value!r}")
        hex_value = f"{value:0{SkyWatcherRevu24Constants.HEX_LENGTH}X}"
        ordered = [hex_value[idx] for idx in SkyWatcherRevu24Constants.ENCODE_ORDER]
        return "".join(ordered).encode(SkyWatcherRevu24Constants.ENCODING)

    @staticmethod
    def _hex_val(b: int) -> int:
        if SkyWatcherRevu24Constants.ASCII_0 <= b <= SkyWatcherRevu24Constants.ASCII_9:
            return b - SkyWatcherRevu24Constants.ASCII_0
        if SkyWatcherRevu24Constants.ASCII_A <= b <= SkyWatcherRevu24Constants.ASCII_F:
            return (
                b
                - SkyWatcherRevu24Constants.ASCII_A
                + (SkyWatcherRevu24Constants.HEX_BASE - SkyWatcherRevu24Constants.DECIMAL_OFFSET)
            )
        if SkyWatcherRevu24Constants.ASCII_a <= b <= SkyWatcherRevu24Constants.ASCII_f:
            return (
                b
                - SkyWatcherRevu24Constants.ASCII_a
                + (SkyWatcherRevu24Constants.HEX_BASE - SkyWatcherRevu24Constants.DECIMAL_OFFSET)
            )
        raise SkyWatcherRevu24Error(f"invalid hex digit: {b!r}")


@dataclasses.dataclass(frozen=True)
class SkyWatcherStatus:
    raw: int
    running: bool
    initialized: bool
    slew_mode: SkyWatcherSlewMode
    direction: SkyWatcherDirection
    speed_mode: SkyWatcherSpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "SkyWatcherStatus":
        # LOGGER.info("status_data data=%r", data)
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0
        raw = b2 | (b1 << 8) | (b3 << 16)
        running = bool(b2 & 0x01)
        initialized = bool(b3 & 0x01)
        slew_mode = SkyWatcherSlewMode.SLEW if (b1 & 0x01) else SkyWatcherSlewMode.GOTO
        direction = SkyWatcherDirection.BACKWARD if (b1 & 0x02) else SkyWatcherDirection.FORWARD
        speed_mode = SkyWatcherSpeedMode.HIGHSPEED if (b1 & 0x04) else SkyWatcherSpeedMode.LOWSPEED
        return cls(
            raw=raw,
            running=running,
            initialized=initialized,
            slew_mode=slew_mode,
            direction=direction,
            speed_mode=speed_mode,
        )


@dataclasses.dataclass(frozen=True)
class SkyWatcherMotionMode:
    slew_mode: SkyWatcherSlewMode
    direction: SkyWatcherDirection
    speed_mode: SkyWatcherSpeedMode

    def to_command(self) -> str:
        LOGGER.info("motion_mode mode=%s", self)
        if self.slew_mode == SkyWatcherSlewMode.SLEW and self.speed_mode == SkyWatcherSpeedMode.LOWSPEED:
            mode = "1"
        elif self.slew_mode == SkyWatcherSlewMode.SLEW and self.speed_mode == SkyWatcherSpeedMode.HIGHSPEED:
            mode = "3"
        elif self.slew_mode == SkyWatcherSlewMode.GOTO and self.speed_mode == SkyWatcherSpeedMode.LOWSPEED:
            mode = "2"
        else:
            mode = "0"
        direction = "1" if self.direction == SkyWatcherDirection.BACKWARD else "0"
        return f"{mode}{direction}"


class SkyWatcherMC:
    """SkyWatcher motor controller protocol wrapper."""

    _LEADING = b":"
    _TRAILING = b"\r"

    def __init__(self, dev: SerialLineDevice, logger: Optional[logging.Logger] = None) -> None:
        LOGGER.info("init dev=%r logger=%r", dev, logger)
        self.dev = dev
        self.log = logger or logging.getLogger("skywatcher.mc")

    def inquire_timer_freq(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        self.log.info("timer_freq axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_TIMER_FREQ, axis)
        return SkyWatcherRevu24.from_bytes(data).value

    def inquire_cpr(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        """
        Inquire counts per revolution (CPR) for the given axis.
        """
        self.log.info("cpr axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_CPR, axis)
        return SkyWatcherRevu24.from_bytes(data).value

    def inquire_position(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        self.log.info("inquire position axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_POSITION, axis)
        return SkyWatcherRevu24.from_bytes(data).value

    def inquire_status(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> SkyWatcherStatus:
        # self.log.info("inquire status axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_STATUS, axis)
        return SkyWatcherStatus.from_bytes(data)

    def inquire_highspeed_ratio(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        self.log.info("highspeed_ratio axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_HIGHSPEED_RATIO, axis)
        return SkyWatcherRevu24.from_bytes(data).value

    def set_step_period(self, axis: SkyWatcherAxis, period: int) -> None:
        self.log.info("step_period axis=%s period=%s", axis, period)
        arg = SkyWatcherRevu24.from_int(period).to_ascii()
        self._transact(SkyWatcherCommand.SET_STEP_PERIOD, axis, arg)

    def set_goto_target(self, axis: SkyWatcherAxis, target: int) -> None:
        self.log.info("goto_target axis=%s target=%s", axis, target)
        arg = SkyWatcherRevu24.from_int(target).to_ascii()
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET, axis, arg)

    def set_goto_target_increment(self, axis: SkyWatcherAxis, increment: int) -> None:
        self.log.info("goto_target_increment axis=%s increment=%s", axis, increment)
        arg = SkyWatcherRevu24.from_int(increment).to_ascii()
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET_INCREMENT, axis, arg)

    def set_target_breaks(self, axis: SkyWatcherAxis, increment: int) -> None:
        self.log.info("target_breaks axis=%s increment=%s", axis, increment)
        arg = SkyWatcherRevu24.from_int(increment).to_ascii()
        self._transact(SkyWatcherCommand.SET_BREAK_POINT_INCREMENT, axis, arg)

    def set_axis_position(self, axis: SkyWatcherAxis, position: int) -> None:
        self.log.info("set_axis_position axis=%s position=%s", axis, position)
        arg = SkyWatcherRevu24.from_int(position).to_ascii()
        self._transact(SkyWatcherCommand.SET_AXIS_POSITION, axis, arg)

    def set_motion_mode(self, axis: SkyWatcherAxis, mode: SkyWatcherMotionMode) -> None:
        self.log.info("motion_mode axis=%s mode=%s", axis, mode)
        self._transact(SkyWatcherCommand.SET_MOTION_MODE, axis, mode.to_command())

    def start_motion(self, axis: SkyWatcherAxis) -> None:
        self.log.info("start axis=%s", axis)
        self._transact(SkyWatcherCommand.START_MOTION, axis)

    def stop_motion(self, axis: SkyWatcherAxis) -> None:
        self.log.info("stop axis=%s", axis)
        self._transact(SkyWatcherCommand.STOP_MOTION, axis)

    def instant_stop(self, axis: SkyWatcherAxis) -> None:
        self.log.info("emergency_stop axis=%s", axis)
        self._transact(SkyWatcherCommand.INSTANT_STOP, axis)

    def set_ra_rate(self, rate: float, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> None:
        abs_rate = abs(rate)
        if abs_rate < SkyWatcherConstants.MIN_RATE or abs_rate > SkyWatcherConstants.MAX_RATE:
            raise SkyWatcherRateError(
                "Speed rate out of limits: %.2fx Sidereal (min=%.2f, max=%.2f)"
                % (abs_rate, SkyWatcherConstants.MIN_RATE, SkyWatcherConstants.MAX_RATE)
            )
        if abs_rate == SkyWatcherConstants.ZERO_RATE:
            raise SkyWatcherRateError("Speed rate must be non-zero.")
        use_highspeed = abs_rate > SkyWatcherConstants.LOWSPEED_RATE
        if use_highspeed:
            ratio = self.inquire_highspeed_ratio(axis)
            if ratio <= SkyWatcherConstants.ZERO_RATE:
                raise SkyWatcherRateError("Invalid highspeed ratio: %s" % ratio)
            abs_rate = abs_rate / ratio
        rate_deg_s = abs_rate * SkyWatcherConstants.SIDEREAL_RATE_DEG_S
        period = self._compute_step_period(axis, rate_deg_s)
        status = self.inquire_status(axis)
        direction = SkyWatcherDirection.FORWARD if rate >= SkyWatcherConstants.ZERO_RATE else SkyWatcherDirection.BACKWARD
        speed_mode = SkyWatcherSpeedMode.HIGHSPEED if use_highspeed else SkyWatcherSpeedMode.LOWSPEED
        mode = SkyWatcherMotionMode(
            slew_mode=SkyWatcherSlewMode.SLEW,
            direction=direction,
            speed_mode=speed_mode,
        )
        if status.running:
            if status.speed_mode != speed_mode:
                raise SkyWatcherRateError("Can not change rate while motor is running (speed mode differs).")
            if status.direction != direction:
                raise SkyWatcherRateError("Can not change rate while motor is running (direction differs).")
        self.set_motion_mode(axis, mode)
        self.set_step_period(axis, period)

    def start_ra_tracking(self, trackspeed_arcsec_s: float, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> None:
        if trackspeed_arcsec_s == SkyWatcherConstants.ZERO_RATE:
            self.stop_motion(axis)
            return
        rate = trackspeed_arcsec_s / SkyWatcherConstants.SIDEREAL_SPEED_ARCSEC_S
        try:
            self.set_ra_rate(rate, axis=axis)
        except SkyWatcherRateError as exc:
            raise SkyWatcherTrackingError(str(exc)) from exc
        self.start_motion(axis)

    def SetRARate(self, rate: float, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> None:
        self.set_ra_rate(rate, axis=axis)

    def StartRATracking(self, trackspeed_arcsec_s: float, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> None:
        self.start_ra_tracking(trackspeed_arcsec_s, axis=axis)

    def do_initialize(
        self,
        axis: SkyWatcherAxis,
        *,
        timeout_s: float,
        poll_interval_s: float,
    ) -> None:
        self.log.info(
            "initialize axis=%s timeout_s=%s poll_interval_s=%s",
            axis,
            timeout_s,
            poll_interval_s,
        )
        status = self.inquire_status(axis)
        if status.initialized:
            return
        self._transact(SkyWatcherCommand.INITIALIZE, axis)
        start = time.monotonic()
        while True:
            status = self.inquire_status(axis)
            if status.initialized:
                return
            if (time.monotonic() - start) >= timeout_s:
                raise TimeoutError
            time.sleep(poll_interval_s)

    def _transact(
        self,
        cmd: SkyWatcherCommand,
        axis: SkyWatcherAxis,
        arg: Optional[str] = None,
    ) -> bytes:
        # self.log.info("command cmd=%s axis=%s arg=%r", cmd, axis, arg)
        axis_char = self._normalize_axis(axis)
        if arg is None:
            payload = self._LEADING + cmd.encode("ascii") + axis_char + self._TRAILING
        else:
            payload = self._LEADING + cmd.encode("ascii") + axis_char + arg.encode("ascii") + self._TRAILING
        self.log.debug(
            "tx cmd=%s axis=%s arg=%r raw=%r hex=%s",
            cmd,
            axis,
            arg,
            payload,
            payload.hex(),
        )
        resp = self.dev.transact(payload, terminator=self._TRAILING)
        self.log.debug(
            "rx cmd=%s axis=%s arg=%r raw=%r hex=%s",
            cmd,
            axis,
            arg,
            resp,
            resp.hex(),
        )
        if not resp:
            raise RuntimeError(f"empty response for cmd={cmd} axis={axis}")
        if resp.endswith(self._TRAILING):
            resp = resp[:-1]
        if not resp:
            raise RuntimeError(f"empty response for cmd={cmd} axis={axis}")
        if resp[:1] == b"!":
            raise RuntimeError(f"skywatcher command error: {resp!r}")
        if resp[:1] != b"=":
            raise RuntimeError(f"invalid response: {resp!r}")
        return resp[1:]

    def _normalize_axis(self, axis: SkyWatcherAxis) -> bytes:
        # self.log.info("axis_normalize axis=%s", axis)
        if not isinstance(axis, SkyWatcherAxis):
            raise TypeError(f"axis must be SkyWatcherAxis, got {type(axis)!r}")
        return axis.to_bytes()

    def _compute_step_period(self, axis: SkyWatcherAxis, rate_deg_s: float) -> int:
        if rate_deg_s <= SkyWatcherConstants.ZERO_RATE:
            raise SkyWatcherRateError("Rate must be positive.")
        cpr = self.inquire_cpr(axis)
        timer_freq = self.inquire_timer_freq(axis)
        if cpr <= SkyWatcherConstants.ZERO_RATE or timer_freq <= SkyWatcherConstants.ZERO_RATE:
            raise SkyWatcherRateError("Invalid CPR or timer frequency for rate calculation.")
        counts_per_s = rate_deg_s * cpr / SkyWatcherConstants.DEGREES_PER_REV
        if counts_per_s <= SkyWatcherConstants.ZERO_RATE:
            raise SkyWatcherRateError("Invalid counts-per-second computed for rate.")
        preset = int(round(timer_freq / counts_per_s))
        return int(clamp(preset, SkyWatcherConstants.MIN_STEP_PERIOD, SkyWatcherConstants.MAX_STEP_PERIOD))
