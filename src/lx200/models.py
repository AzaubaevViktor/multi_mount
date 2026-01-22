from __future__ import annotations

import dataclasses
import datetime as dt
from typing import Tuple

from .protocol import LX200Constants, LX200ValueError


@dataclasses.dataclass(frozen=True)
class LX200Ra:
    hours: float

    def __post_init__(self) -> None:
        if self.hours < LX200Constants.MIN_HOUR or self.hours >= LX200Constants.HOURS_PER_DAY:
            raise LX200ValueError(f"RA out of range: {self.hours!r}")

    @staticmethod
    def _wrap_hours(hours: float) -> float:
        hours = hours % LX200Constants.HOURS_PER_DAY
        if hours < LX200Constants.MIN_HOUR:
            hours += LX200Constants.HOURS_PER_DAY
        return hours

    @staticmethod
    def _hms_to_hours(hour: int, minute: int, second: int) -> float:
        return (
            hour
            + minute / LX200Constants.MINUTES_PER_HOUR
            + second / LX200Constants.SECONDS_PER_HOUR
        )

    @classmethod
    def _hours_to_hms(cls, hours: float) -> Tuple[int, int, int]:
        hours = cls._wrap_hours(hours)
        hour = int(hours)
        remainder = (hours - hour) * LX200Constants.MINUTES_PER_HOUR
        minute = int(remainder)
        second = int(round((remainder - minute) * LX200Constants.SECONDS_PER_MINUTE))
        if second == LX200Constants.SECONDS_PER_MINUTE:
            second = LX200Constants.DEFAULT_SECOND
            minute += LX200Constants.SIGN_POS_INT
        if minute == LX200Constants.MINUTES_PER_HOUR:
            minute = LX200Constants.MIN_MINUTE
            hour = (hour + LX200Constants.SIGN_POS_INT) % LX200Constants.HOURS_PER_DAY
        return hour, minute, second

    @classmethod
    def _parse_ra_hms(cls, value: str) -> float:
        parts = value.strip().split(LX200Constants.TIME_SEP)
        if len(parts) != LX200Constants.TIME_PARTS:
            raise LX200ValueError(f"bad RA {value!r}")
        hour, minute, second = (int(p) for p in parts)
        return cls._wrap_hours(cls._hms_to_hours(hour, minute, second))

    @classmethod
    def _format_ra(cls, hours: float) -> str:
        hour, minute, second = cls._hours_to_hms(hours)
        return (
            f"{hour:0{LX200Constants.TIME_FIELD_WIDTH}d}"
            f"{LX200Constants.TIME_SEP}"
            f"{minute:0{LX200Constants.TIME_FIELD_WIDTH}d}"
            f"{LX200Constants.TIME_SEP}"
            f"{second:0{LX200Constants.TIME_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    @classmethod
    def from_string(cls, value: str) -> "LX200Ra":
        return cls(cls._parse_ra_hms(value))

    def to_string(self) -> str:
        return self._format_ra(self.hours)


@dataclasses.dataclass(frozen=True)
class LX200Dec:
    degrees: float

    def __post_init__(self) -> None:
        if self.degrees < LX200Constants.MIN_LAT_DEG or self.degrees > LX200Constants.MAX_LAT_DEG:
            raise LX200ValueError(f"DEC out of range: {self.degrees!r}")

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return minimum if value < minimum else maximum if value > maximum else value

    @staticmethod
    def _deg_to_dms(deg: float) -> Tuple[int, int, int, int]:
        sign = LX200Constants.SIGN_POS_INT
        if deg < LX200Constants.MIN_SECOND:
            sign = LX200Constants.SIGN_NEG_INT
            deg = -deg
        degrees = int(deg)
        remainder = (deg - degrees) * LX200Constants.MIN_PER_DEG
        minutes = int(remainder)
        seconds = int(round((remainder - minutes) * LX200Constants.SECONDS_PER_MINUTE))
        if seconds == LX200Constants.SECONDS_PER_MINUTE:
            seconds = LX200Constants.MIN_SECOND
            minutes += LX200Constants.SIGN_POS_INT
        if minutes == LX200Constants.MIN_PER_DEG:
            minutes = LX200Constants.MIN_MINUTE
            degrees += LX200Constants.SIGN_POS_INT
        return sign, degrees, minutes, seconds

    @classmethod
    def _parse_dec_dms(cls, value: str) -> float:
        value = value.strip()
        sign = LX200Constants.SIGN_POS_INT
        if value.startswith(LX200Constants.SIGN_NEG):
            sign = LX200Constants.SIGN_NEG_INT
            value = value[1:]
        elif value.startswith(LX200Constants.SIGN_POS):
            value = value[1:]
        value = value.replace(LX200Constants.DEGREE_SIGN, LX200Constants.DEG_MIN_SEP)
        if LX200Constants.DEG_MIN_SEP not in value:
            raise LX200ValueError(f"bad DEC {value!r}")
        deg_str, rest = value.split(LX200Constants.DEG_MIN_SEP, 1)
        degrees = int(deg_str)
        if LX200Constants.TIME_SEP in rest:
            min_str, sec_str = rest.split(LX200Constants.TIME_SEP, 1)
            minutes = int(min_str)
            seconds = int(sec_str)
        else:
            minutes = int(rest)
            seconds = LX200Constants.DEFAULT_SECOND
        deg_value = sign * (
            degrees
            + minutes / LX200Constants.MIN_PER_DEG
            + seconds / LX200Constants.SECONDS_PER_HOUR
        )
        return cls._clamp(deg_value, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)

    @classmethod
    def _format_dec(cls, degrees: float) -> str:
        degrees = cls._clamp(degrees, LX200Constants.MIN_LAT_DEG, LX200Constants.MAX_LAT_DEG)
        sign, deg_value, minutes, seconds = cls._deg_to_dms(degrees)
        sign_char = LX200Constants.SIGN_POS if sign >= LX200Constants.SIGN_POS_INT else LX200Constants.SIGN_NEG
        return (
            f"{sign_char}{deg_value:0{LX200Constants.LAT_DEG_WIDTH}d}"
            f"{LX200Constants.DEG_MIN_SEP}"
            f"{minutes:0{LX200Constants.MIN_FIELD_WIDTH}d}"
            f"{LX200Constants.TIME_SEP}"
            f"{seconds:0{LX200Constants.TIME_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    @classmethod
    def from_string(cls, value: str) -> "LX200Dec":
        return cls(cls._parse_dec_dms(value))

    def to_string(self) -> str:
        return self._format_dec(self.degrees)


@dataclasses.dataclass(frozen=True)
class LX200Time:
    hour: int
    minute: int
    second: int

    def __post_init__(self) -> None:
        if not (LX200Constants.MIN_HOUR <= self.hour <= LX200Constants.MAX_HOUR):
            raise LX200ValueError(f"hour out of range: {self.hour!r}")
        if not (LX200Constants.MIN_MINUTE <= self.minute <= LX200Constants.MAX_MINUTE):
            raise LX200ValueError(f"minute out of range: {self.minute!r}")
        if not (LX200Constants.MIN_SECOND <= self.second <= LX200Constants.MAX_SECOND):
            raise LX200ValueError(f"second out of range: {self.second!r}")

    @classmethod
    def from_string(cls, value: str) -> "LX200Time":
        parts = value.split(LX200Constants.TIME_SEP)
        if len(parts) == LX200Constants.TIME_PARTS_SHORT:
            hour, minute = (int(p) for p in parts)
            second = LX200Constants.DEFAULT_SECOND
        elif len(parts) == LX200Constants.TIME_PARTS:
            hour, minute, second = (int(p) for p in parts)
        else:
            raise LX200ValueError(f"invalid time: {value!r}")
        return cls(hour=hour, minute=minute, second=second)

    def to_string(self) -> str:
        hour = f"{self.hour:0{LX200Constants.TIME_FIELD_WIDTH}d}"
        minute = f"{self.minute:0{LX200Constants.TIME_FIELD_WIDTH}d}"
        second = f"{self.second:0{LX200Constants.TIME_FIELD_WIDTH}d}"
        return f"{hour}{LX200Constants.TIME_SEP}{minute}{LX200Constants.TIME_SEP}{second}{LX200Constants.TERMINATOR}"


@dataclasses.dataclass(frozen=True)
class LX200Date:
    month: int
    day: int
    year: int

    def __post_init__(self) -> None:
        if not (LX200Constants.MIN_MONTH <= self.month <= LX200Constants.MAX_MONTH):
            raise LX200ValueError(f"month out of range: {self.month!r}")
        if not (LX200Constants.MIN_DAY <= self.day <= LX200Constants.MAX_DAY):
            raise LX200ValueError(f"day out of range: {self.day!r}")
        if not (LX200Constants.YEAR_MIN <= self.year <= LX200Constants.YEAR_MAX):
            raise LX200ValueError(f"year out of range: {self.year!r}")
        try:
            dt.date(self.year, self.month, self.day)
        except ValueError as exc:
            raise LX200ValueError(f"invalid date: {self.year}-{self.month}-{self.day}") from exc

    @classmethod
    def from_string(cls, value: str) -> "LX200Date":
        parts = value.split(LX200Constants.DATE_SEP)
        if len(parts) != LX200Constants.DATE_PARTS:
            raise LX200ValueError(f"invalid date: {value!r}")
        month, day, year = (int(p) for p in parts)
        if year < (LX200Constants.YEAR_BASE % LX200Constants.CENTURY):
            year += LX200Constants.YEAR_BASE
        else:
            year += LX200Constants.YEAR_BASE - LX200Constants.CENTURY
        return cls(month=month, day=day, year=year)

    def to_string(self) -> str:
        yy = self.year % LX200Constants.CENTURY
        month = f"{self.month:0{LX200Constants.DATE_FIELD_WIDTH}d}"
        day = f"{self.day:0{LX200Constants.DATE_FIELD_WIDTH}d}"
        year = f"{yy:0{LX200Constants.YEAR_FIELD_WIDTH}d}"
        return f"{month}{LX200Constants.DATE_SEP}{day}{LX200Constants.DATE_SEP}{year}{LX200Constants.TERMINATOR}"


@dataclasses.dataclass(frozen=True)
class LX200UtcOffset:
    hours: float

    def __post_init__(self) -> None:
        if self.hours < LX200Constants.MIN_UTC_OFFSET or self.hours > LX200Constants.MAX_UTC_OFFSET:
            raise LX200ValueError(f"UTC offset out of range: {self.hours!r}")

    @classmethod
    def from_string(cls, value: str) -> "LX200UtcOffset":
        value = value.strip()
        if LX200Constants.UTC_OFFSET_SEP in value:
            sign = LX200Constants.SIGN_POS_INT
            if value.startswith(LX200Constants.SIGN_NEG):
                sign = LX200Constants.SIGN_NEG_INT
                value = value[1:]
            elif value.startswith(LX200Constants.SIGN_POS):
                value = value[1:]
            parts = value.split(LX200Constants.UTC_OFFSET_SEP)
            if len(parts) != LX200Constants.UTC_OFFSET_PARTS:
                raise LX200ValueError(f"invalid UTC offset: {value!r}")
            hours_part, minutes_part = (int(p) for p in parts)
            hours = sign * (hours_part + minutes_part / LX200Constants.MINUTES_PER_HOUR)
        else:
            try:
                hours = float(value)
            except ValueError as exc:
                raise LX200ValueError(f"invalid UTC offset: {value!r}") from exc
        return cls(hours=hours)

    def to_string(self) -> str:
        whole = round(self.hours)
        if abs(self.hours - whole) < LX200Constants.UTC_OFFSET_EPS:
            return f"{whole:+d}{LX200Constants.TERMINATOR}"
        return f"{self.hours:+.{LX200Constants.UTC_OFFSET_DECIMALS}f}{LX200Constants.TERMINATOR}"


@dataclasses.dataclass(frozen=True)
class LX200Site:
    latitude_deg: float
    longitude_west_deg: float

    def __post_init__(self) -> None:
        if self.latitude_deg < LX200Constants.MIN_LAT_DEG or self.latitude_deg > LX200Constants.MAX_LAT_DEG:
            raise LX200ValueError(f"latitude out of range: {self.latitude_deg!r}")
        if self.longitude_west_deg < LX200Constants.MIN_LON_DEG or self.longitude_west_deg > LX200Constants.MAX_LON_DEG:
            raise LX200ValueError(f"longitude out of range: {self.longitude_west_deg!r}")

    @classmethod
    def _parse_signed_deg_min(cls, value: str) -> Tuple[int, int, int]:
        value = value.strip()
        sign = LX200Constants.SIGN_POS_INT
        if value.startswith(LX200Constants.SIGN_NEG):
            sign = LX200Constants.SIGN_NEG_INT
            value = value[1:]
        elif value.startswith(LX200Constants.SIGN_POS):
            value = value[1:]
        if LX200Constants.DEG_MIN_SEP not in value:
            raise LX200ValueError(f"invalid degrees format: {value!r}")
        deg_str, min_str = value.split(LX200Constants.DEG_MIN_SEP, 1)
        deg = int(deg_str)
        minutes = int(min_str)
        return sign, deg, minutes

    @classmethod
    def latitude_from_string(cls, value: str) -> float:
        sign, deg, minutes = cls._parse_signed_deg_min(value)
        return sign * (deg + minutes / LX200Constants.MIN_PER_DEG)

    @classmethod
    def longitude_from_string(cls, value: str) -> float:
        sign, deg, minutes = cls._parse_signed_deg_min(value)
        return sign * (deg + minutes / LX200Constants.MIN_PER_DEG)

    @classmethod
    def from_lat_lon_strings(cls, lat: str, lon: str) -> "LX200Site":
        lat_deg = cls.latitude_from_string(lat)
        lon_west_deg = cls.longitude_from_string(lon)
        return cls(latitude_deg=lat_deg, longitude_west_deg=lon_west_deg)

    def latitude_to_string(self) -> str:
        sign, deg, minutes, _ = self._deg_to_dms(self.latitude_deg)
        sign_char = (
            LX200Constants.SIGN_POS
            if sign == LX200Constants.SIGN_POS_INT
            else LX200Constants.SIGN_NEG
        )
        return (
            f"{sign_char}{deg:0{LX200Constants.LAT_DEG_WIDTH}d}"
            f"{LX200Constants.DEG_MIN_SEP}{minutes:0{LX200Constants.MIN_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    def longitude_to_string(self) -> str:
        sign, deg, minutes, _ = self._deg_to_dms(self.latitude_deg)
        sign_char = (
            LX200Constants.SIGN_POS
            if sign == LX200Constants.SIGN_POS_INT
            else LX200Constants.SIGN_NEG
        )
        return (
            f"{sign_char}{deg:0{LX200Constants.LON_DEG_WIDTH}d}"
            f"{LX200Constants.DEG_MIN_SEP}{minutes:0{LX200Constants.MIN_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    @staticmethod
    def _deg_to_dms(deg: float) -> Tuple[int, int, int, int]:
        sign = LX200Constants.SIGN_POS_INT
        if deg < LX200Constants.MIN_SECOND:
            sign = LX200Constants.SIGN_NEG_INT
            deg = -deg
        degrees = int(deg)
        remainder = (deg - degrees) * LX200Constants.MIN_PER_DEG
        minutes = int(remainder)
        seconds = int(round((remainder - minutes) * LX200Constants.SECONDS_PER_MINUTE))
        if seconds == LX200Constants.SECONDS_PER_MINUTE:
            seconds = LX200Constants.MIN_SECOND
            minutes += LX200Constants.SIGN_POS_INT
        if minutes == LX200Constants.MIN_PER_DEG:
            minutes = LX200Constants.MIN_MINUTE
            degrees += LX200Constants.SIGN_POS_INT
        return sign, degrees, minutes, seconds

    @staticmethod
    def format_latitude(latitude_deg: float) -> str:
        site = LX200Site(latitude_deg=latitude_deg, longitude_west_deg=LX200Constants.MIN_LON_DEG)
        return site.latitude_to_string()

    @staticmethod
    def format_longitude(longitude_west_deg: float) -> str:
        site = LX200Site(latitude_deg=LX200Constants.MIN_LAT_DEG, longitude_west_deg=longitude_west_deg)
        return site.longitude_to_string()
