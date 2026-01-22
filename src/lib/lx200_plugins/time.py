from __future__ import annotations

from typing import Any, Protocol

from lx200_models import LX200Date, LX200Time, LX200UtcOffset
from lx200_parsers import (
    format_date,
    format_ok,
    format_time,
    format_utc_offset,
    parse_date_arg,
    parse_no_arg,
    parse_time_arg,
    parse_utc_offset_arg,
)
from lx200_protocol import LX200Command
from lx200_server import CommandSpec


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
