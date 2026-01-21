from __future__ import annotations

import dataclasses
import datetime as dt
import logging
from enum import StrEnum
from typing import Optional, Tuple

from coords import deg_to_dms, fmt_dec, fmt_ra, parse_dec_dms, parse_ra_hms

LOGGER = logging.getLogger("lx200")


class LX200Constants:
    PREFIX = ":"
    TERMINATOR = "#"
    TIME_SEP = ":"
    DATE_SEP = "/"
    DEG_MIN_SEP = "*"
    SIGN_POS = "+"
    SIGN_NEG = "-"
    SIGN_POS_INT = 1
    SIGN_NEG_INT = -1
    CMD_LEN = 2
    STOP_CMD_LEN = 1
    MOVE_DIR_LEN = 1
    TIME_PARTS = 3
    DATE_PARTS = 3
    UTC_OFFSET_EPS = 1e-6
    HOURS_PER_DAY = 24
    CENTURY = 100
    TIME_FIELD_WIDTH = 2
    DATE_FIELD_WIDTH = 2
    YEAR_FIELD_WIDTH = 2
    LAT_DEG_WIDTH = 2
    LON_DEG_WIDTH = 3
    MIN_FIELD_WIDTH = 2
    UTC_OFFSET_DECIMALS = 1
    MIN_HOUR = 0
    MAX_HOUR = 23
    MIN_MINUTE = 0
    MAX_MINUTE = 59
    MIN_SECOND = 0
    MAX_SECOND = 59
    MIN_MONTH = 1
    MAX_MONTH = 12
    MIN_DAY = 1
    MAX_DAY = 31
    YEAR_BASE = 2000
    YEAR_MIN = 1900
    YEAR_MAX = 2099
    MIN_LAT_DEG = -90.0
    MAX_LAT_DEG = 90.0
    MIN_LON_DEG = -180.0
    MAX_LON_DEG = 180.0
    MIN_UTC_OFFSET = -24.0
    MAX_UTC_OFFSET = 24.0
    MIN_PER_DEG = 60
    RESPONSE_OK = "1"
    RESPONSE_ERR = "0"
    RESPONSE_EMPTY = ""
    SYNC_OK = "OK"


class LX200Error(Exception):
    pass


class LX200ParseError(LX200Error):
    pass


class LX200UnsupportedCommandError(LX200Error):
    pass


class LX200ValueError(LX200Error):
    pass


class LX200Command(StrEnum):
    GET_RA = "GR"
    GET_DEC = "GD"
    SET_RA = "Sr"
    SET_DEC = "Sd"
    GOTO = "MS"
    SYNC = "CM"
    STOP = "Q"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    MOVE_EAST = "Me"
    MOVE_WEST = "Mw"
    RATE_GUIDE = "RG"
    RATE_CENTER = "RC"
    RATE_FIND = "RM"
    RATE_SLEW = "RS"
    SET_LOCAL_TIME = "SL"
    SET_DATE = "SC"
    SET_UTC_OFFSET = "SG"
    SET_LATITUDE = "St"
    SET_LONGITUDE = "Sg"
    GET_LOCAL_TIME = "GL"
    GET_DATE = "GC"
    GET_UTC_OFFSET = "GG"
    GET_LONGITUDE = "Gg"
    GET_LATITUDE = "Gt"


class LX200MoveDirection(StrEnum):
    NORTH = "n"
    SOUTH = "s"
    EAST = "e"
    WEST = "w"


class LX200SlewRate(StrEnum):
    GUIDE = "RG"
    CENTER = "RC"
    FIND = "RM"
    SLEW = "RS"


class LX200GotoResult(StrEnum):
    OK = "0"
    ALREADY_THERE = "1"
    BELOW_HORIZON = "2"


@dataclasses.dataclass(frozen=True)
class LX200Ra:
    hours: float

    def __post_init__(self) -> None:
        if self.hours < LX200Constants.MIN_HOUR or self.hours >= LX200Constants.HOURS_PER_DAY:
            raise LX200ValueError(f"RA out of range: {self.hours!r}")

    @classmethod
    def from_string(cls, value: str) -> "LX200Ra":
        return cls(parse_ra_hms(value))

    def to_string(self) -> str:
        return fmt_ra(self.hours)


@dataclasses.dataclass(frozen=True)
class LX200Dec:
    degrees: float

    def __post_init__(self) -> None:
        if self.degrees < LX200Constants.MIN_LAT_DEG or self.degrees > LX200Constants.MAX_LAT_DEG:
            raise LX200ValueError(f"DEC out of range: {self.degrees!r}")

    @classmethod
    def from_string(cls, value: str) -> "LX200Dec":
        return cls(parse_dec_dms(value))

    def to_string(self) -> str:
        return fmt_dec(self.degrees)


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
        if len(parts) != LX200Constants.TIME_PARTS:
            raise LX200ValueError(f"invalid time: {value!r}")
        hour, minute, second = (int(p) for p in parts)
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
        sign, deg, minutes, _ = deg_to_dms(abs(self.latitude_deg))
        sign_char = LX200Constants.SIGN_POS if self.latitude_deg >= 0 else LX200Constants.SIGN_NEG
        return (
            f"{sign_char}{deg:0{LX200Constants.LAT_DEG_WIDTH}d}"
            f"{LX200Constants.DEG_MIN_SEP}{minutes:0{LX200Constants.MIN_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    def longitude_to_string(self) -> str:
        sign, deg, minutes, _ = deg_to_dms(abs(self.longitude_west_deg))
        sign_char = LX200Constants.SIGN_POS if self.longitude_west_deg >= 0 else LX200Constants.SIGN_NEG
        return (
            f"{sign_char}{deg:0{LX200Constants.LON_DEG_WIDTH}d}"
            f"{LX200Constants.DEG_MIN_SEP}{minutes:0{LX200Constants.MIN_FIELD_WIDTH}d}"
            f"{LX200Constants.TERMINATOR}"
        )

    @staticmethod
    def format_latitude(latitude_deg: float) -> str:
        site = LX200Site(latitude_deg=latitude_deg, longitude_west_deg=LX200Constants.MIN_LON_DEG)
        return site.latitude_to_string()

    @staticmethod
    def format_longitude(longitude_west_deg: float) -> str:
        site = LX200Site(latitude_deg=LX200Constants.MIN_LAT_DEG, longitude_west_deg=longitude_west_deg)
        return site.longitude_to_string()


@dataclasses.dataclass(frozen=True)
class LX200CommandRequest:
    command: LX200Command
    arg: Optional[str]


class LX200ServerBase:
    def __init__(self, logger: Optional[logging.Logger] = None) -> None:
        self.log = logger or logging.getLogger("lx200.server")
        self._handlers = {
            LX200Command.GET_RA: self._handle_get_ra,
            LX200Command.GET_DEC: self._handle_get_dec,
            LX200Command.SET_RA: self._handle_set_ra,
            LX200Command.SET_DEC: self._handle_set_dec,
            LX200Command.GOTO: self._handle_goto,
            LX200Command.SYNC: self._handle_sync,
            LX200Command.STOP: self._handle_stop,
            LX200Command.MOVE_NORTH: self._handle_move_north,
            LX200Command.MOVE_SOUTH: self._handle_move_south,
            LX200Command.MOVE_EAST: self._handle_move_east,
            LX200Command.MOVE_WEST: self._handle_move_west,
            LX200Command.RATE_GUIDE: self._handle_rate_guide,
            LX200Command.RATE_CENTER: self._handle_rate_center,
            LX200Command.RATE_FIND: self._handle_rate_find,
            LX200Command.RATE_SLEW: self._handle_rate_slew,
            LX200Command.SET_LOCAL_TIME: self._handle_set_local_time,
            LX200Command.SET_DATE: self._handle_set_date,
            LX200Command.SET_UTC_OFFSET: self._handle_set_utc_offset,
            LX200Command.SET_LATITUDE: self._handle_set_latitude,
            LX200Command.SET_LONGITUDE: self._handle_set_longitude,
            LX200Command.GET_LOCAL_TIME: self._handle_get_local_time,
            LX200Command.GET_DATE: self._handle_get_date,
            LX200Command.GET_UTC_OFFSET: self._handle_get_utc_offset,
            LX200Command.GET_LONGITUDE: self._handle_get_longitude,
            LX200Command.GET_LATITUDE: self._handle_get_latitude,
        }

    def handle_command(self, raw: str) -> str:
        request = self._parse_request(raw)
        self.log.debug("lx200 rx command=%s arg=%r", request.command, request.arg)
        handler = self._handlers.get(request.command)
        if handler is None:
            raise LX200UnsupportedCommandError(f"unsupported command {request.command!r}")
        return handler(request.arg)

    def _parse_request(self, raw: str) -> LX200CommandRequest:
        raw = raw.strip()
        if not raw.startswith(LX200Constants.PREFIX) or not raw.endswith(LX200Constants.TERMINATOR):
            raise LX200ParseError(f"invalid framing: {raw!r}")
        body = raw[len(LX200Constants.PREFIX) : -len(LX200Constants.TERMINATOR)]
        if not body:
            raise LX200ParseError(f"empty command: {raw!r}")
        if body.startswith(LX200Command.STOP.value):
            arg = body[LX200Constants.STOP_CMD_LEN :] or None
            if arg is not None and len(arg) != LX200Constants.MOVE_DIR_LEN:
                raise LX200ParseError(f"invalid stop arg: {raw!r}")
            return LX200CommandRequest(command=LX200Command.STOP, arg=arg)
        if len(body) < LX200Constants.CMD_LEN:
            raise LX200ParseError(f"short command: {raw!r}")
        cmd_text = body[: LX200Constants.CMD_LEN]
        arg = body[LX200Constants.CMD_LEN :] or None
        try:
            cmd = LX200Command(cmd_text)
        except ValueError as exc:
            raise LX200UnsupportedCommandError(f"unknown command: {cmd_text!r}") from exc
        return LX200CommandRequest(command=cmd, arg=arg)

    def _require_no_arg(self, cmd: LX200Command, arg: Optional[str]) -> None:
        if arg:
            raise LX200ParseError(f"unexpected arg for {cmd}: {arg!r}")

    def _handle_get_ra(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_RA, arg)
        return self.get_current_ra().to_string()

    def _handle_get_dec(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_DEC, arg)
        return self.get_current_dec().to_string()

    def _handle_set_ra(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing RA argument")
        ra = LX200Ra.from_string(arg)
        accepted = self.set_target_ra(ra)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_set_dec(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing DEC argument")
        dec = LX200Dec.from_string(arg)
        accepted = self.set_target_dec(dec)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_goto(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GOTO, arg)
        return self.slew_to_target().value

    def _handle_sync(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.SYNC, arg)
        return self.sync_to_target()

    def _handle_stop(self, arg: Optional[str]) -> str:
        if arg is None:
            self.stop_all()
            return LX200Constants.RESPONSE_EMPTY
        direction = self._parse_direction(arg)
        self.stop_move(direction)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_move_north(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.MOVE_NORTH, arg)
        self.start_move(LX200MoveDirection.NORTH)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_move_south(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.MOVE_SOUTH, arg)
        self.start_move(LX200MoveDirection.SOUTH)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_move_east(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.MOVE_EAST, arg)
        self.start_move(LX200MoveDirection.EAST)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_move_west(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.MOVE_WEST, arg)
        self.start_move(LX200MoveDirection.WEST)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_rate_guide(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.RATE_GUIDE, arg)
        self.set_slew_rate(LX200SlewRate.GUIDE)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_rate_center(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.RATE_CENTER, arg)
        self.set_slew_rate(LX200SlewRate.CENTER)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_rate_find(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.RATE_FIND, arg)
        self.set_slew_rate(LX200SlewRate.FIND)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_rate_slew(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.RATE_SLEW, arg)
        self.set_slew_rate(LX200SlewRate.SLEW)
        return LX200Constants.RESPONSE_EMPTY

    def _handle_set_local_time(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing local time argument")
        value = LX200Time.from_string(arg)
        accepted = self.set_local_time(value)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_set_date(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing date argument")
        value = LX200Date.from_string(arg)
        accepted = self.set_date(value)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_set_utc_offset(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing UTC offset argument")
        value = LX200UtcOffset.from_string(arg)
        accepted = self.set_utc_offset(value)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_set_latitude(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing latitude argument")
        lat = LX200Site.latitude_from_string(arg)
        accepted = self.set_latitude(lat)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_set_longitude(self, arg: Optional[str]) -> str:
        if arg is None:
            raise LX200ParseError("missing longitude argument")
        lon = LX200Site.longitude_from_string(arg)
        accepted = self.set_longitude(lon)
        return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR

    def _handle_get_local_time(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_LOCAL_TIME, arg)
        return self.get_local_time().to_string()

    def _handle_get_date(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_DATE, arg)
        return self.get_date().to_string()

    def _handle_get_utc_offset(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_UTC_OFFSET, arg)
        return self.get_utc_offset().to_string()

    def _handle_get_longitude(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_LONGITUDE, arg)
        return LX200Site.format_longitude(self.get_longitude())

    def _handle_get_latitude(self, arg: Optional[str]) -> str:
        self._require_no_arg(LX200Command.GET_LATITUDE, arg)
        return LX200Site.format_latitude(self.get_latitude())

    def _parse_direction(self, arg: str) -> LX200MoveDirection:
        value = arg.lower()
        try:
            return LX200MoveDirection(value)
        except ValueError as exc:
            raise LX200ParseError(f"invalid direction: {arg!r}") from exc

    def get_current_ra(self) -> LX200Ra:
        raise NotImplementedError

    def get_current_dec(self) -> LX200Dec:
        raise NotImplementedError

    def set_target_ra(self, ra: LX200Ra) -> bool:
        raise NotImplementedError

    def set_target_dec(self, dec: LX200Dec) -> bool:
        raise NotImplementedError

    def slew_to_target(self) -> LX200GotoResult:
        raise NotImplementedError

    def sync_to_target(self) -> str:
        return f"{LX200Constants.SYNC_OK}{LX200Constants.TERMINATOR}"

    def stop_all(self) -> None:
        raise NotImplementedError

    def start_move(self, direction: LX200MoveDirection) -> None:
        raise NotImplementedError

    def stop_move(self, direction: LX200MoveDirection) -> None:
        raise NotImplementedError

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        raise NotImplementedError

    def set_local_time(self, value: LX200Time) -> bool:
        raise NotImplementedError

    def set_date(self, value: LX200Date) -> bool:
        raise NotImplementedError

    def set_utc_offset(self, value: LX200UtcOffset) -> bool:
        raise NotImplementedError

    def set_latitude(self, latitude_deg: float) -> bool:
        raise NotImplementedError

    def set_longitude(self, longitude_west_deg: float) -> bool:
        raise NotImplementedError

    def get_local_time(self) -> LX200Time:
        raise NotImplementedError

    def get_date(self) -> LX200Date:
        raise NotImplementedError

    def get_utc_offset(self) -> LX200UtcOffset:
        raise NotImplementedError

    def get_latitude(self) -> float:
        raise NotImplementedError

    def get_longitude(self) -> float:
        raise NotImplementedError
