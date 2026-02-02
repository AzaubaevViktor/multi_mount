from __future__ import annotations

import dataclasses

from lx200.models import LX200Date, LX200Dec, LX200Ra, LX200Time, LX200UtcOffset
from lx200.protocol import LX200Constants, LX200SlewRate
from lib.skywatcher import SkyWatcherAxis, SkyWatcherConstants, SkyWatcherRevu24Constants, SkyWatcherSpeedMode


class SkyWatcherBackendConstants:
    LOGGER_NAME = "lx200.skywatcher"
    SERIAL_LOGGER_NAME = "lx200.skywatcher.serial"
    DEFAULT_SITE_NAME = "SkyWatcher"
    DEFAULT_DISTANCE = LX200Constants.DEFAULT_DISTANCE
    DEFAULT_TRACKING_RATE = LX200Constants.DEFAULT_TRACKING_RATE
    DEFAULT_OBJECT_SIZE = LX200Constants.DEFAULT_DISTANCE
    RA_DEG_PER_HOUR = SkyWatcherConstants.DEGREES_PER_REV / LX200Constants.HOURS_PER_DAY
    REVU24_MOD = SkyWatcherRevu24Constants.MAX_VALUE + 1
    REVU24_HALF = REVU24_MOD // 2
    DEFAULT_INIT_TIMEOUT_S = 5.0
    DEFAULT_INIT_POLL_INTERVAL_S = 0.2
    DEFAULT_GOTO_STEP_PERIOD = 18
    DEFAULT_GOTO_BREAK_MAX = 200
    DEFAULT_GOTO_TOLERANCE_TICKS = 1
    DEFAULT_GUIDE_RATE_MULT = 0.5
    DEFAULT_CENTER_RATE_MULT = 1.0
    DEFAULT_FIND_RATE_MULT = 4.0
    DEFAULT_SLEW_RATE_MULT = 16.0
    DEFAULT_LOCAL_TIME = LX200Time(
        hour=LX200Constants.MIN_HOUR,
        minute=LX200Constants.MIN_MINUTE,
        second=LX200Constants.DEFAULT_SECOND,
    )
    DEFAULT_DATE = LX200Date(
        month=LX200Constants.MIN_MONTH,
        day=LX200Constants.MIN_DAY,
        year=LX200Constants.YEAR_BASE,
    )
    DEFAULT_UTC_OFFSET = LX200UtcOffset(hours=0.0)
    DEFAULT_INITIAL_RA = LX200Ra(hours=0.0)
    DEFAULT_INITIAL_DEC = LX200Dec(degrees=0.0)


class SkyWatcherBackendError(Exception):
    pass


class SkyWatcherConfigError(SkyWatcherBackendError):
    pass


class SkyWatcherInitializationError(SkyWatcherBackendError):
    pass


class SkyWatcherAxisStateError(SkyWatcherBackendError):
    pass


class SkyWatcherOperationError(SkyWatcherBackendError):
    pass


@dataclasses.dataclass(frozen=True)
class SkyWatcherSerialConfig:
    port: str
    baud: int
    timeout_s: float
    device_name: str = SkyWatcherBackendConstants.SERIAL_LOGGER_NAME

    def __post_init__(self) -> None:
        if not self.port:
            raise SkyWatcherConfigError("serial port is required")
        if self.baud <= 0:
            raise SkyWatcherConfigError("baud rate must be positive")
        if self.timeout_s <= 0.0:
            raise SkyWatcherConfigError("timeout must be positive")


@dataclasses.dataclass(frozen=True)
class SkyWatcherInitConfig:
    timeout_s: float = SkyWatcherBackendConstants.DEFAULT_INIT_TIMEOUT_S
    poll_interval_s: float = SkyWatcherBackendConstants.DEFAULT_INIT_POLL_INTERVAL_S

    def __post_init__(self) -> None:
        if self.timeout_s <= 0.0:
            raise SkyWatcherConfigError("init timeout must be positive")
        if self.poll_interval_s <= 0.0:
            raise SkyWatcherConfigError("init poll interval must be positive")


@dataclasses.dataclass(frozen=True)
class SkyWatcherGotoConfig:
    step_period: int = SkyWatcherBackendConstants.DEFAULT_GOTO_STEP_PERIOD
    break_max: int = SkyWatcherBackendConstants.DEFAULT_GOTO_BREAK_MAX
    tolerance_ticks: int = SkyWatcherBackendConstants.DEFAULT_GOTO_TOLERANCE_TICKS
    speed_mode: SkyWatcherSpeedMode = SkyWatcherSpeedMode.LOWSPEED

    def __post_init__(self) -> None:
        if self.step_period <= 0:
            raise SkyWatcherConfigError("goto step period must be positive")
        if self.break_max <= 0:
            raise SkyWatcherConfigError("goto break max must be positive")
        if self.tolerance_ticks < 0:
            raise SkyWatcherConfigError("goto tolerance must be non-negative")


@dataclasses.dataclass(frozen=True)
class SkyWatcherSlewRateConfig:
    guide_rate_mult: float = SkyWatcherBackendConstants.DEFAULT_GUIDE_RATE_MULT
    center_rate_mult: float = SkyWatcherBackendConstants.DEFAULT_CENTER_RATE_MULT
    find_rate_mult: float = SkyWatcherBackendConstants.DEFAULT_FIND_RATE_MULT
    slew_rate_mult: float = SkyWatcherBackendConstants.DEFAULT_SLEW_RATE_MULT

    def __post_init__(self) -> None:
        for value in (
            self.guide_rate_mult,
            self.center_rate_mult,
            self.find_rate_mult,
            self.slew_rate_mult,
        ):
            if value <= 0.0:
                raise SkyWatcherConfigError("slew rate multipliers must be positive")

    def multiplier_for(self, rate: LX200SlewRate) -> float:
        if rate == LX200SlewRate.GUIDE:
            return self.guide_rate_mult
        if rate == LX200SlewRate.CENTER:
            return self.center_rate_mult
        if rate == LX200SlewRate.FIND:
            return self.find_rate_mult
        return self.slew_rate_mult


@dataclasses.dataclass(frozen=True)
class SkyWatcherAxisMapping:
    ra_axis: SkyWatcherAxis = SkyWatcherAxis.RA
    dec_axis: SkyWatcherAxis = SkyWatcherAxis.DEC
    ra_forward_is_east: bool = True
    dec_forward_is_north: bool = True

    def __post_init__(self) -> None:
        if self.ra_axis == self.dec_axis:
            raise SkyWatcherConfigError("RA and DEC axis must differ")


@dataclasses.dataclass
class SkyWatcherAxisState:
    axis: SkyWatcherAxis
    cpr: int
    zero_ticks: int

    def __post_init__(self) -> None:
        if self.cpr <= 0:
            raise SkyWatcherAxisStateError("axis CPR must be positive")
        if not isinstance(self.axis, SkyWatcherAxis):
            raise SkyWatcherAxisStateError("axis must be SkyWatcherAxis")
        if self.zero_ticks < 0:
            raise SkyWatcherAxisStateError("zero ticks must be non-negative")
        if self.zero_ticks > SkyWatcherRevu24Constants.MAX_VALUE:
            raise SkyWatcherAxisStateError("zero ticks exceed revu24 range")

    def ticks_per_deg(self) -> float:
        return self.cpr / SkyWatcherConstants.DEGREES_PER_REV

    def ticks_from_degrees(self, degrees: float) -> int:
        return int(round(degrees * self.ticks_per_deg()))

    def degrees_from_ticks(self, ticks: int) -> float:
        return ticks / self.ticks_per_deg()


@dataclasses.dataclass(frozen=True)
class SkyWatcherMountConfig:
    axis_mapping: SkyWatcherAxisMapping = dataclasses.field(default_factory=SkyWatcherAxisMapping)
    init_config: SkyWatcherInitConfig = dataclasses.field(default_factory=SkyWatcherInitConfig)
    goto_config: SkyWatcherGotoConfig = dataclasses.field(default_factory=SkyWatcherGotoConfig)
    slew_rate_config: SkyWatcherSlewRateConfig = dataclasses.field(default_factory=SkyWatcherSlewRateConfig)
    initial_ra: LX200Ra = SkyWatcherBackendConstants.DEFAULT_INITIAL_RA
    initial_dec: LX200Dec = SkyWatcherBackendConstants.DEFAULT_INITIAL_DEC
    auto_initialize: bool = True
