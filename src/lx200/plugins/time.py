from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple

from ..models import LX200Date, LX200Time, LX200UtcOffset
from ..protocol import LX200Command, LX200Constants, LX200ParseError
from ..server import CommandSpec


def parse_no_arg(arg: Optional[str]) -> Tuple[()]:
    if arg:
        raise LX200ParseError(f"unexpected arg: {arg!r}")
    return ()


def parse_time_arg(arg: Optional[str]) -> Tuple[LX200Time]:
    if arg is None:
        raise LX200ParseError("missing local time argument")
    return (LX200Time.from_string(arg),)


def parse_date_arg(arg: Optional[str]) -> Tuple[LX200Date]:
    if arg is None:
        raise LX200ParseError("missing date argument")
    return (LX200Date.from_string(arg),)


def parse_utc_offset_arg(arg: Optional[str]) -> Tuple[LX200UtcOffset]:
    if arg is None:
        raise LX200ParseError("missing UTC offset argument")
    return (LX200UtcOffset.from_string(arg),)


def format_ok(accepted: bool) -> str:
    return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR


def format_time(value: LX200Time) -> str:
    return value.to_string()


def format_date(value: LX200Date) -> str:
    return value.to_string()


def format_utc_offset(value: LX200UtcOffset) -> str:
    return value.to_string()


class LX200TimeBackend(Protocol):
    def set_local_time(self, value: LX200Time) -> bool:
        raise NotImplementedError

    def set_date(self, value: LX200Date) -> bool:
        raise NotImplementedError

    def set_utc_offset(self, value: LX200UtcOffset) -> bool:
        raise NotImplementedError

    def get_local_time(self) -> LX200Time:
        raise NotImplementedError

    def get_date(self) -> LX200Date:
        raise NotImplementedError

    def get_utc_offset(self) -> LX200UtcOffset:
        raise NotImplementedError


class LX200TimePlugin:
    def __init__(self, backend: LX200TimeBackend) -> None:
        self._backend = backend

    def specs(self) -> list[CommandSpec[Any]]:
        return [
            CommandSpec(LX200Command.SET_LOCAL_TIME, parse_time_arg, self.set_local_time, format_ok),
            CommandSpec(LX200Command.SET_DATE, parse_date_arg, self.set_date, format_ok),
            CommandSpec(LX200Command.SET_UTC_OFFSET, parse_utc_offset_arg, self.set_utc_offset, format_ok),
            CommandSpec(LX200Command.GET_LOCAL_TIME, parse_no_arg, self.get_local_time, format_time),
            CommandSpec(LX200Command.GET_DATE, parse_no_arg, self.get_date, format_date),
            CommandSpec(LX200Command.GET_DATE_ALT, parse_no_arg, self.get_date_alt, format_date),
            CommandSpec(LX200Command.GET_UTC_OFFSET, parse_no_arg, self.get_utc_offset, format_utc_offset),
        ]

    def set_local_time(self, value: LX200Time) -> bool:
        return self._backend.set_local_time(value)

    def set_date(self, value: LX200Date) -> bool:
        return self._backend.set_date(value)

    def set_utc_offset(self, value: LX200UtcOffset) -> bool:
        return self._backend.set_utc_offset(value)

    def get_local_time(self) -> LX200Time:
        return self._backend.get_local_time()

    def get_date(self) -> LX200Date:
        return self._backend.get_date()

    def get_date_alt(self) -> LX200Date:
        return self._backend.get_date()

    def get_utc_offset(self) -> LX200UtcOffset:
        return self._backend.get_utc_offset()
