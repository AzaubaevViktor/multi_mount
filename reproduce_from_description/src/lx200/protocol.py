from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LX200Command(str, Enum):
    GET_RA = "GR"
    GET_DEC = "GD"
    SET_RA = "Sr"
    SET_DEC = "Sd"
    SYNC = "CM"
    SLEW = "MS"
    STATUS = "D"
    MOVE_EAST = "Me"
    MOVE_WEST = "Mw"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    HALT_ALL = "Q"
    HALT_EAST = "Qe"
    HALT_WEST = "Qw"
    HALT_NORTH = "Qn"
    HALT_SOUTH = "Qs"
    RATE_GUIDE = "RG"
    RATE_CENTER = "RC"
    RATE_FIND = "RM"
    RATE_MAX = "RS"
    GUIDE = "Mg"
    GET_UTC_OFFSET = "GG"
    GET_TIME = "GL"
    GET_DATE = "GC"
    GET_LONGITUDE = "Gg"
    GET_LATITUDE = "Gt"
    GET_SITE_NAME = "GM"
    GET_CLOCK_FORMAT = "Gc"
    GET_TELESCOPE_NAME = "GT"
    SET_LONGITUDE = "Sg"
    SET_LATITUDE = "St"
    SET_UTC_OFFSET = "SG"
    SET_TIME = "SL"
    SET_DATE = "SC"
    SET_HIGHEST_ELEVATION = "Sh"
    SET_MINIMUM_ELEVATION = "So"


@dataclass(frozen=True, slots=True)
class LX200Reply:
    payload: str
    terminated: bool = False

    def to_bytes(self) -> bytes:
        suffix = "#" if self.terminated and not self.payload.endswith("#") else ""
        return f"{self.payload}{suffix}".encode("ascii")


def bool_reply(value: bool) -> LX200Reply:
    return LX200Reply("1" if value else "0")


def string_reply(value: str) -> LX200Reply:
    return LX200Reply(value, terminated=True)


def empty_reply() -> LX200Reply:
    return LX200Reply("")
