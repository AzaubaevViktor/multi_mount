from __future__ import annotations

import dataclasses
from enum import StrEnum
from typing import Optional


class LX200Constants:
    PREFIX = ":"
    TERMINATOR = "#"
    TIME_SEP = ":"
    DATE_SEP = "/"
    DEG_MIN_SEP = "*"
    UTC_OFFSET_SEP = ":"
    SIGN_POS = "+"
    SIGN_NEG = "-"
    SIGN_POS_INT = 1
    SIGN_NEG_INT = -1
    CMD_LEN = 2
    STOP_CMD_LEN = 1
    SINGLE_CMD_LEN = 1
    MOVE_DIR_LEN = 1
    TIME_PARTS = 3
    TIME_PARTS_SHORT = 2
    DATE_PARTS = 3
    UTC_OFFSET_PARTS = 2
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
    DEFAULT_SECOND = 0
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
    MINUTES_PER_HOUR = 60
    SECONDS_PER_MINUTE = 60
    SECONDS_PER_HOUR = 3600
    DEGREE_SIGN = "\u00b0"
    RESPONSE_OK = "1"
    RESPONSE_ERR = "0"
    RESPONSE_EMPTY = ""
    SYNC_OK = "OK"
    DEFAULT_SITE_NAME = "LX200"
    DEFAULT_TRACKING_RATE = "0"
    DEFAULT_DISTANCE = "0"


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
    GET_DATE_ALT = "Gc"
    GET_UTC_OFFSET = "GG"
    GET_TRACKING_RATE = "GT"
    GET_SITE_NAME = "GM"
    GET_LONGITUDE = "Gg"
    GET_LATITUDE = "Gt"
    SET_OBJECT_SIZE = "So"
    GET_DISTANCE = "D"


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


class LX200SyncResult(StrEnum):
    OK = LX200Constants.SYNC_OK


@dataclasses.dataclass(frozen=True)
class LX200CommandRequest:
    command: LX200Command
    arg: Optional[str]


def parse_request(raw: str) -> LX200CommandRequest:
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
    if len(body) == LX200Constants.SINGLE_CMD_LEN:
        try:
            cmd = LX200Command(body)
        except ValueError as exc:
            raise LX200UnsupportedCommandError(f"unknown command: {body!r}") from exc
        return LX200CommandRequest(command=cmd, arg=None)
    if len(body) < LX200Constants.CMD_LEN:
        raise LX200ParseError(f"short command: {raw!r}")
    cmd_text = body[: LX200Constants.CMD_LEN]
    arg = body[LX200Constants.CMD_LEN :] or None
    try:
        cmd = LX200Command(cmd_text)
    except ValueError as exc:
        raise LX200UnsupportedCommandError(f"unknown command: {cmd_text!r}") from exc
    return LX200CommandRequest(command=cmd, arg=arg)
