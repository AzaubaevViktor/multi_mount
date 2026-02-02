from __future__ import annotations

import dataclasses
from enum import IntEnum, StrEnum

from lx200.models import LX200Dec, LX200Ra
from lx200.protocol import LX200Constants
from tmc2209.proxy import ProtocolConstants


class TMC2209LX200Constants:
    LOGGER_NAME = "lx200.tmc2209"
    DEGREES_PER_REV = 360.0
    HALF_DEGREES_PER_REV = DEGREES_PER_REV / 2.0
    RA_DEG_PER_HOUR = DEGREES_PER_REV / LX200Constants.HOURS_PER_DAY
    DEFAULT_GUIDE_SPS = ProtocolConstants.MIN_SPS
    DEFAULT_CENTER_SPS = ProtocolConstants.MIN_SPS
    DEFAULT_FIND_SPS = ProtocolConstants.MIN_SPS
    DEFAULT_SLEW_SPS = ProtocolConstants.MIN_SPS
    DEFAULT_GOTO_SPS = ProtocolConstants.MIN_SPS
    DEFAULT_TOLERANCE_STEPS = 1
    DEFAULT_INITIAL_RA = LX200Ra(hours=0.0)
    DEFAULT_INITIAL_DEC = LX200Dec(degrees=0.0)


class TMC2209LX200Error(Exception):
    pass


class TMC2209ConfigError(TMC2209LX200Error):
    pass


class TMC2209AxisStateError(TMC2209LX200Error):
    pass


class TMC2209OperationError(TMC2209LX200Error):
    pass


class TMC2209Axis(StrEnum):
    RA = "ra"
    DEC = "dec"


class TMC2209DirectionSign(IntEnum):
    POSITIVE = 1
    NEGATIVE = -1


@dataclasses.dataclass(frozen=True)
class TMC2209AxisMapping:
    ra_forward_is_east: bool = True
    dec_forward_is_north: bool = True


@dataclasses.dataclass(frozen=True)
class TMC2209AxisConfig:
    steps_per_degree: float
    guide_sps: int = TMC2209LX200Constants.DEFAULT_GUIDE_SPS
    center_sps: int = TMC2209LX200Constants.DEFAULT_CENTER_SPS
    find_sps: int = TMC2209LX200Constants.DEFAULT_FIND_SPS
    slew_sps: int = TMC2209LX200Constants.DEFAULT_SLEW_SPS
    goto_sps: int = TMC2209LX200Constants.DEFAULT_GOTO_SPS
    tolerance_steps: int = TMC2209LX200Constants.DEFAULT_TOLERANCE_STEPS
    auto_enable: bool = True

    def __post_init__(self) -> None:
        if self.steps_per_degree <= 0.0:
            raise TMC2209ConfigError("steps per degree must be positive")
        self._validate_sps(self.guide_sps, "guide")
        self._validate_sps(self.center_sps, "center")
        self._validate_sps(self.find_sps, "find")
        self._validate_sps(self.slew_sps, "slew")
        self._validate_sps(self.goto_sps, "goto")
        if self.tolerance_steps < 0:
            raise TMC2209ConfigError("tolerance steps must be non-negative")

    @staticmethod
    def _validate_sps(value: int, label: str) -> None:
        if value < ProtocolConstants.MIN_SPS or value > ProtocolConstants.MAX_SPS:
            raise TMC2209ConfigError(f"{label} steps per second out of range")


@dataclasses.dataclass
class TMC2209AxisState:
    axis: TMC2209Axis
    steps_per_degree: float
    direction_sign: TMC2209DirectionSign

    def __post_init__(self) -> None:
        if self.steps_per_degree <= 0.0:
            raise TMC2209AxisStateError("steps per degree must be positive")
        if self.direction_sign not in (
            TMC2209DirectionSign.POSITIVE,
            TMC2209DirectionSign.NEGATIVE,
        ):
            raise TMC2209AxisStateError("direction sign must be +/-1")

    def steps_from_degrees(self, degrees: float) -> int:
        return int(round(degrees * self.steps_per_degree * int(self.direction_sign)))

    def degrees_from_steps(self, steps: int) -> float:
        return (steps / self.steps_per_degree) * int(self.direction_sign)


@dataclasses.dataclass(frozen=True)
class TMC2209MountConfig:
    axis_mapping: TMC2209AxisMapping = dataclasses.field(default_factory=TMC2209AxisMapping)
    ra_axis_config: TMC2209AxisConfig | None = None
    dec_axis_config: TMC2209AxisConfig | None = None
    initial_ra: LX200Ra = TMC2209LX200Constants.DEFAULT_INITIAL_RA
    initial_dec: LX200Dec = TMC2209LX200Constants.DEFAULT_INITIAL_DEC

    def __post_init__(self) -> None:
        if self.ra_axis_config is None and self.dec_axis_config is None:
            raise TMC2209ConfigError("at least one axis config is required")
