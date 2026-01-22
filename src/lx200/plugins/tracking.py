from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple

from ..protocol import LX200Command, LX200Constants, LX200ParseError, LX200SlewRate
from ..server import CommandSpec


def parse_no_arg(arg: Optional[str]) -> Tuple[()]:
    if arg:
        raise LX200ParseError(f"unexpected arg: {arg!r}")
    return ()


def format_empty(_: Optional[object]) -> str:
    return LX200Constants.RESPONSE_EMPTY


def format_tracking_rate(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"


class LX200TrackingBackend(Protocol):
    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        raise NotImplementedError

    def get_tracking_rate(self) -> str:
        raise NotImplementedError


class LX200TrackingPlugin:
    def __init__(self, backend: LX200TrackingBackend) -> None:
        self._backend = backend

    def specs(self) -> list[CommandSpec[Any]]:
        return [
            CommandSpec(LX200Command.RATE_GUIDE, parse_no_arg, self.set_rate_guide, format_empty),
            CommandSpec(LX200Command.RATE_CENTER, parse_no_arg, self.set_rate_center, format_empty),
            CommandSpec(LX200Command.RATE_FIND, parse_no_arg, self.set_rate_find, format_empty),
            CommandSpec(LX200Command.RATE_SLEW, parse_no_arg, self.set_rate_slew, format_empty),
            CommandSpec(LX200Command.GET_TRACKING_RATE, parse_no_arg, self.get_tracking_rate, format_tracking_rate),
        ]

    def set_rate_guide(self) -> None:
        self._backend.set_slew_rate(LX200SlewRate.GUIDE)
        return None

    def set_rate_center(self) -> None:
        self._backend.set_slew_rate(LX200SlewRate.CENTER)
        return None

    def set_rate_find(self) -> None:
        self._backend.set_slew_rate(LX200SlewRate.FIND)
        return None

    def set_rate_slew(self) -> None:
        self._backend.set_slew_rate(LX200SlewRate.SLEW)
        return None

    def get_tracking_rate(self) -> str:
        return self._backend.get_tracking_rate()
