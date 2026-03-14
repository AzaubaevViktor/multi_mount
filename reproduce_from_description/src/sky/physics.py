from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


SECONDS_PER_HOUR = 3600.0
SECONDS_PER_MINUTE = 60.0
DEGREES_PER_HOUR = 15.0


def wrap_hours(hours: float) -> float:
    return ((hours + 12.0) % 24.0) - 12.0


def clamp_dec(degrees: float) -> float:
    return max(-90.0, min(90.0, degrees))


def parse_ha(text: str) -> float:
    parts = text.strip().split(":")
    if len(parts) not in {2, 3}:
        raise ValueError(f"Invalid HA value: {text!r}")

    hours = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2]) if len(parts) == 3 else 0.0
    return wrap_hours(hours + minutes / 60.0 + seconds / SECONDS_PER_HOUR)


def parse_dec(text: str) -> float:
    cleaned = text.strip().replace("*", ":")
    sign = -1.0 if cleaned.startswith("-") else 1.0
    cleaned = cleaned.lstrip("+-")
    parts = cleaned.split(":")
    if len(parts) not in {2, 3}:
        raise ValueError(f"Invalid DEC value: {text!r}")

    degrees = int(parts[0])
    minutes = int(parts[1])
    seconds = float(parts[2]) if len(parts) == 3 else 0.0
    total = degrees + minutes / 60.0 + seconds / SECONDS_PER_HOUR
    return clamp_dec(sign * total)


def format_ha(hours: float) -> str:
    wrapped = wrap_hours(hours)
    if wrapped < 0.0:
        wrapped += 24.0

    whole_hours = int(wrapped)
    minutes_full = (wrapped - whole_hours) * 60.0
    whole_minutes = int(minutes_full)
    seconds = int(round((minutes_full - whole_minutes) * 60.0))
    if seconds == 60:
        whole_minutes += 1
        seconds = 0
    if whole_minutes == 60:
        whole_hours = (whole_hours + 1) % 24
        whole_minutes = 0
    return f"{whole_hours:02d}:{whole_minutes:02d}:{seconds:02d}"


def format_dec(degrees: float) -> str:
    clamped = clamp_dec(degrees)
    sign = "+" if clamped >= 0.0 else "-"
    absolute = abs(clamped)
    whole_degrees = int(absolute)
    minutes_full = (absolute - whole_degrees) * 60.0
    whole_minutes = int(minutes_full)
    seconds = int(round((minutes_full - whole_minutes) * 60.0))
    if seconds == 60:
        whole_minutes += 1
        seconds = 0
    if whole_minutes == 60:
        whole_degrees += 1
        whole_minutes = 0
    return f"{sign}{whole_degrees:02d}*{whole_minutes:02d}:{seconds:02d}"


@dataclass(frozen=True, slots=True)
class Second:
    value: float

    def __post_init__(self) -> None:
        if not isfinite(self.value):
            raise ValueError("Second value must be finite")


@dataclass(frozen=True, slots=True)
class Ha:
    hours: float

    def __post_init__(self) -> None:
        if not isfinite(self.hours):
            raise ValueError("HA value must be finite")
        object.__setattr__(self, "hours", wrap_hours(self.hours))

    @classmethod
    def from_lx200(cls, text: str) -> "Ha":
        return cls(parse_ha(text))

    def to_lx200(self) -> str:
        return format_ha(self.hours)

    def shift(self, delta_hours: float) -> "Ha":
        return Ha(self.hours + delta_hours)

    def shortest_delta_to(self, target: "Ha") -> float:
        return wrap_hours(target.hours - self.hours)


@dataclass(frozen=True, slots=True)
class Dec:
    degrees: float

    def __post_init__(self) -> None:
        if not isfinite(self.degrees):
            raise ValueError("DEC value must be finite")
        object.__setattr__(self, "degrees", clamp_dec(self.degrees))

    @classmethod
    def from_lx200(cls, text: str) -> "Dec":
        return cls(parse_dec(text))

    def to_lx200(self) -> str:
        return format_dec(self.degrees)

    def shift(self, delta_degrees: float) -> "Dec":
        return Dec(self.degrees + delta_degrees)

    def delta_to(self, target: "Dec") -> float:
        return target.degrees - self.degrees


@dataclass(frozen=True, slots=True)
class HaPerSecond:
    hours_per_second: float

    def __post_init__(self) -> None:
        if not isfinite(self.hours_per_second):
            raise ValueError("HA speed must be finite")

    @classmethod
    def from_degrees_per_second(cls, degrees_per_second: float) -> "HaPerSecond":
        return cls(degrees_per_second / DEGREES_PER_HOUR)

    def to_degrees_per_second(self) -> float:
        return self.hours_per_second * DEGREES_PER_HOUR

    def scaled(self, factor: float) -> "HaPerSecond":
        return HaPerSecond(self.hours_per_second * factor)

    def absolute(self) -> "HaPerSecond":
        return HaPerSecond(abs(self.hours_per_second))


@dataclass(frozen=True, slots=True)
class DecPerSecond:
    degrees_per_second: float

    def __post_init__(self) -> None:
        if not isfinite(self.degrees_per_second):
            raise ValueError("DEC speed must be finite")

    @classmethod
    def from_arcsec_per_second(cls, arcsec_per_second: float) -> "DecPerSecond":
        return cls(arcsec_per_second / 3600.0)

    def to_arcsec_per_second(self) -> float:
        return self.degrees_per_second * 3600.0

    def scaled(self, factor: float) -> "DecPerSecond":
        return DecPerSecond(self.degrees_per_second * factor)

    def absolute(self) -> "DecPerSecond":
        return DecPerSecond(abs(self.degrees_per_second))


@dataclass(frozen=True, slots=True)
class PointCoordinates:
    ra: Ha
    dec: Dec

    def shifted(self, ra_delta_hours: float, dec_delta_degrees: float) -> "PointCoordinates":
        return PointCoordinates(ra=self.ra.shift(ra_delta_hours), dec=self.dec.shift(dec_delta_degrees))
