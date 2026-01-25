from __future__ import annotations

import math
from enum import StrEnum

from lx200.protocol import LX200Constants


class PolarAlignmentConstants:
    LOGGER_NAME = "polar_alignment"
    DEFAULT_POLL_INTERVAL_S = 0.5
    DEFAULT_TRACKING_SIGN = LX200Constants.SIGN_POS_INT
    DEFAULT_TIME_FORMAT = "iso"
    FILE_ENCODING = "ascii"
    ZERO_FLOAT = 0.0
    HOURS_PER_DAY = LX200Constants.HOURS_PER_DAY
    SECONDS_PER_HOUR = LX200Constants.SECONDS_PER_HOUR
    SIDEREAL_DAY_SECONDS = 86164.0905
    DEGREES_PER_CIRCLE = 360.0
    RADIANS_PER_CIRCLE = math.tau
    DEG_PER_HOUR = DEGREES_PER_CIRCLE / HOURS_PER_DAY
    SIDEREAL_HOURS_PER_SECOND = HOURS_PER_DAY / SIDEREAL_DAY_SECONDS
    VECTOR_EPS = 1e-9
    AXIS_SIGN_POSITIVE = 1.0
    AXIS_SIGN_NEGATIVE = -1.0


class PolarAlignmentFileKey(StrEnum):
    T1 = "t1"
    T2 = "t2"
    T3 = "t3"


class PolarAlignmentFieldKey(StrEnum):
    TIME = "time"
    RA = "ra"
    DEC = "dec"


class PolarAlignmentCliConstants:
    ARG_INPUT = "--input"
    ARG_POLL = "--poll"
    ARG_TRACKING_SIGN = "--tracking-sign"
    ARG_VERBOSE = "--verbose"
    DEFAULT_TRACKING_SIGN = PolarAlignmentConstants.DEFAULT_TRACKING_SIGN
