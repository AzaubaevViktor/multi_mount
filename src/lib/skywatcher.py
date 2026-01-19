from __future__ import annotations
import dataclasses
import logging
import time
from enum import IntEnum, StrEnum
from typing import Optional

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
        return self._revu24_to_int(data)

    def inquire_cpr(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        """
        Inquire counts per revolution (CPR) for the given axis.
        """
        self.log.info("cpr axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_CPR, axis)
        return self._revu24_to_int(data)

    def inquire_position(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> int:
        self.log.info("inquire position axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_POSITION, axis)
        return self._revu24_to_int(data)

    def inquire_status(self, axis: SkyWatcherAxis = SkyWatcherAxis.RA) -> SkyWatcherStatus:
        # self.log.info("inquire status axis=%s", axis)
        data = self._transact(SkyWatcherCommand.INQUIRE_STATUS, axis)
        return SkyWatcherStatus.from_bytes(data)

    def set_step_period(self, axis: SkyWatcherAxis, period: int) -> None:
        self.log.info("step_period axis=%s period=%s", axis, period)
        arg = self._int_to_revu24(period)
        self._transact(SkyWatcherCommand.SET_STEP_PERIOD, axis, arg)

    def set_goto_target(self, axis: SkyWatcherAxis, target: int) -> None:
        self.log.info("goto_target axis=%s target=%s", axis, target)
        arg = self._int_to_revu24(target)
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET, axis, arg)

    def set_goto_target_increment(self, axis: SkyWatcherAxis, increment: int) -> None:
        self.log.info("goto_target_increment axis=%s increment=%s", axis, increment)
        arg = self._int_to_revu24(increment)
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET_INCREMENT, axis, arg)

    def set_target_breaks(self, axis: SkyWatcherAxis, increment: int) -> None:
        self.log.info("target_breaks axis=%s increment=%s", axis, increment)
        arg = self._int_to_revu24(increment)
        self._transact(SkyWatcherCommand.SET_BREAK_POINT_INCREMENT, axis, arg)

    def set_axis_position(self, axis: SkyWatcherAxis, position: int) -> None:
        self.log.info("set_axis_position axis=%s position=%s", axis, position)
        arg = self._int_to_revu24(position)
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

    def _revu24_to_int(self, data: bytes) -> int:
        # self.log.info("revu24_data data=%r", data)
        if len(data) < 6:
            raise ValueError(f"revu24 data too short: {data!r}")
        def hex_val(b: int) -> int:
            # self.log.info("hex_digit b=%s", b)
            if 48 <= b <= 57:
                return b - 48
            if 65 <= b <= 70:
                return b - 55
            if 97 <= b <= 102:
                return b - 87
            raise ValueError(f"invalid hex digit: {b!r}")
        res = hex_val(data[4])
        res = (res << 4) | hex_val(data[5])
        res = (res << 4) | hex_val(data[2])
        res = (res << 4) | hex_val(data[3])
        res = (res << 4) | hex_val(data[0])
        res = (res << 4) | hex_val(data[1])
        return res

    def _int_to_revu24(self, value: int) -> str:
        self.log.info("revu24_encode value=%s", value)
        n = int(value) & 0xFFFFFF
        hexa = "0123456789ABCDEF"
        return "".join(
            [
                hexa[(n & 0xF0) >> 4],
                hexa[(n & 0x0F)],
                hexa[(n & 0xF000) >> 12],
                hexa[(n & 0x0F00) >> 8],
                hexa[(n & 0xF00000) >> 20],
                hexa[(n & 0x0F0000) >> 16],
            ]
        )
