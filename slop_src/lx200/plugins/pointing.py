from __future__ import annotations

from typing import Any, Optional, Protocol, Tuple

from ..models import LX200Dec, LX200Ra
from ..protocol import (
    LX200Command,
    LX200Constants,
    LX200GotoResult,
    LX200MoveDirection,
    LX200ParseError,
    LX200SyncResult,
)
from ..server import CommandSpec


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


def format_goto(value: LX200GotoResult) -> str:
    return value.value


def format_sync(value: LX200SyncResult) -> str:
    return f"{value.value}{LX200Constants.TERMINATOR}"


class LX200PointingBackend(Protocol):
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

    def sync_to_target(self) -> LX200SyncResult:
        raise NotImplementedError

    def stop_all(self) -> None:
        raise NotImplementedError

    def start_move(self, direction: LX200MoveDirection) -> None:
        raise NotImplementedError

    def stop_move(self, direction: LX200MoveDirection) -> None:
        raise NotImplementedError


class LX200PointingPlugin:
    def __init__(self, backend: LX200PointingBackend) -> None:
        self._backend = backend

    def specs(self) -> list[CommandSpec[Any]]:
        return [
            CommandSpec(LX200Command.GET_RA, parse_no_arg, self.get_current_ra, format_ra),
            CommandSpec(LX200Command.GET_DEC, parse_no_arg, self.get_current_dec, format_dec),
            CommandSpec(LX200Command.SET_RA, parse_ra_arg, self.set_target_ra, format_ok),
            CommandSpec(LX200Command.SET_DEC, parse_dec_arg, self.set_target_dec, format_ok),
            CommandSpec(LX200Command.GOTO, parse_no_arg, self.slew_to_target, format_goto),
            CommandSpec(LX200Command.SYNC, parse_no_arg, self.sync_to_target, format_sync),
            CommandSpec(LX200Command.STOP, parse_stop_arg, self.stop, format_empty),
            CommandSpec(LX200Command.MOVE_NORTH, parse_no_arg, self.start_move_north, format_empty),
            CommandSpec(LX200Command.MOVE_SOUTH, parse_no_arg, self.start_move_south, format_empty),
            CommandSpec(LX200Command.MOVE_EAST, parse_no_arg, self.start_move_east, format_empty),
            CommandSpec(LX200Command.MOVE_WEST, parse_no_arg, self.start_move_west, format_empty),
        ]

    def get_current_ra(self) -> LX200Ra:
        return self._backend.get_current_ra()

    def get_current_dec(self) -> LX200Dec:
        return self._backend.get_current_dec()

    def set_target_ra(self, ra: LX200Ra) -> bool:
        return self._backend.set_target_ra(ra)

    def set_target_dec(self, dec: LX200Dec) -> bool:
        return self._backend.set_target_dec(dec)

    def slew_to_target(self) -> LX200GotoResult:
        return self._backend.slew_to_target()

    def sync_to_target(self) -> LX200SyncResult:
        return self._backend.sync_to_target()

    def stop(self, direction: Optional[LX200MoveDirection]) -> None:
        if direction is None:
            self._backend.stop_all()
            return None
        self._backend.stop_move(direction)
        return None

    def start_move_north(self) -> None:
        self._backend.start_move(LX200MoveDirection.NORTH)
        return None

    def start_move_south(self) -> None:
        self._backend.start_move(LX200MoveDirection.SOUTH)
        return None

    def start_move_east(self) -> None:
        self._backend.start_move(LX200MoveDirection.EAST)
        return None

    def start_move_west(self) -> None:
        self._backend.start_move(LX200MoveDirection.WEST)
        return None
