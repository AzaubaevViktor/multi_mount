from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple

from ..models import LX200Site
from ..protocol import LX200Command, LX200Constants, LX200ParseError
from ..server import CommandSpec


def parse_no_arg(arg: Optional[str]) -> Tuple[()]:
    if arg:
        raise LX200ParseError(f"unexpected arg: {arg!r}")
    return ()


def parse_latitude_arg(arg: Optional[str]) -> Tuple[float]:
    if arg is None:
        raise LX200ParseError("missing latitude argument")
    return (LX200Site.latitude_from_string(arg),)


def parse_longitude_arg(arg: Optional[str]) -> Tuple[float]:
    if arg is None:
        raise LX200ParseError("missing longitude argument")
    return (LX200Site.longitude_from_string(arg),)


def format_ok(accepted: bool) -> str:
    return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR


def format_latitude(value: float) -> str:
    return LX200Site.format_latitude(value)


def format_longitude(value: float) -> str:
    return LX200Site.format_longitude(value)


def format_site_name(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"


class LX200SiteBackend(Protocol):
    def set_latitude(self, latitude_deg: float) -> bool:
        raise NotImplementedError

    def set_longitude(self, longitude_west_deg: float) -> bool:
        raise NotImplementedError

    def get_latitude(self) -> float:
        raise NotImplementedError

    def get_longitude(self) -> float:
        raise NotImplementedError

    def get_site_name(self) -> str:
        raise NotImplementedError


class LX200SitePlugin:
    def __init__(self, backend: LX200SiteBackend) -> None:
        self._backend = backend

    def specs(self) -> list[CommandSpec[Any]]:
        return [
            CommandSpec(LX200Command.SET_LATITUDE, parse_latitude_arg, self.set_latitude, format_ok),
            CommandSpec(LX200Command.SET_LONGITUDE, parse_longitude_arg, self.set_longitude, format_ok),
            CommandSpec(LX200Command.GET_LATITUDE, parse_no_arg, self.get_latitude, format_latitude),
            CommandSpec(LX200Command.GET_LONGITUDE, parse_no_arg, self.get_longitude, format_longitude),
            CommandSpec(LX200Command.GET_SITE_NAME, parse_no_arg, self.get_site_name, format_site_name),
        ]

    def set_latitude(self, latitude_deg: float) -> bool:
        return self._backend.set_latitude(latitude_deg)

    def set_longitude(self, longitude_west_deg: float) -> bool:
        return self._backend.set_longitude(longitude_west_deg)

    def get_latitude(self) -> float:
        return self._backend.get_latitude()

    def get_longitude(self) -> float:
        return self._backend.get_longitude()

    def get_site_name(self) -> str:
        return self._backend.get_site_name()
