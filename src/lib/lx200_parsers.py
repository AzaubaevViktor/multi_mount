from __future__ import annotations

from typing import Optional, Tuple

from lx200_models import (
    LX200Date,
    LX200Dec,
    LX200Ra,
    LX200Site,
    LX200Time,
    LX200UtcOffset,
)
from lx200_protocol import (
    LX200Constants,
    LX200GotoResult,
    LX200MoveDirection,
    LX200ParseError,
    LX200SyncResult,
)


def parse_no_arg(arg: Optional[str]) -> Tuple[()]:
    if arg:
        raise LX200ParseError(f"unexpected arg: {arg!r}")
    return ()


def parse_ra_arg(arg: Optional[str]) -> Tuple[LX200Ra]:
    if arg is None:
        raise LX200ParseError("missing RA argument")
    return (LX200Ra.from_string(arg),)


def parse_dec_arg(arg: Optional[str]) -> Tuple[LX200Dec]:
    if arg is None:
        raise LX200ParseError("missing DEC argument")
    return (LX200Dec.from_string(arg),)


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


def parse_latitude_arg(arg: Optional[str]) -> Tuple[float]:
    if arg is None:
        raise LX200ParseError("missing latitude argument")
    return (LX200Site.latitude_from_string(arg),)


def parse_longitude_arg(arg: Optional[str]) -> Tuple[float]:
    if arg is None:
        raise LX200ParseError("missing longitude argument")
    return (LX200Site.longitude_from_string(arg),)


def parse_object_size_arg(arg: Optional[str]) -> Tuple[str]:
    if arg is None:
        raise LX200ParseError("missing object size argument")
    return (arg,)


def parse_stop_arg(arg: Optional[str]) -> Tuple[Optional[LX200MoveDirection]]:
    if arg is None:
        return (None,)
    value = arg.lower()
    try:
        return (LX200MoveDirection(value),)
    except ValueError as exc:
        raise LX200ParseError(f"invalid direction: {arg!r}") from exc


def format_ok(accepted: bool) -> str:
    return LX200Constants.RESPONSE_OK if accepted else LX200Constants.RESPONSE_ERR


def format_empty(_: Optional[object]) -> str:
    return LX200Constants.RESPONSE_EMPTY


def format_ra(value: LX200Ra) -> str:
    return value.to_string()


def format_dec(value: LX200Dec) -> str:
    return value.to_string()


def format_time(value: LX200Time) -> str:
    return value.to_string()


def format_date(value: LX200Date) -> str:
    return value.to_string()


def format_utc_offset(value: LX200UtcOffset) -> str:
    return value.to_string()


def format_latitude(value: float) -> str:
    return LX200Site.format_latitude(value)


def format_longitude(value: float) -> str:
    return LX200Site.format_longitude(value)


def format_goto(value: LX200GotoResult) -> str:
    return value.value


def format_sync(value: LX200SyncResult) -> str:
    return f"{value.value}{LX200Constants.TERMINATOR}"


def format_tracking_rate(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"


def format_site_name(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"


def format_distance(value: str) -> str:
    return f"{value}{LX200Constants.TERMINATOR}"
