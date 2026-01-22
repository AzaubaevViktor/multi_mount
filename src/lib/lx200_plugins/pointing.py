from __future__ import annotations

from typing import Any, Optional, Protocol

from lx200_models import LX200Dec, LX200Ra
from lx200_parsers import (
    format_dec,
    format_empty,
    format_goto,
    format_ok,
    format_ra,
    format_sync,
    parse_dec_arg,
    parse_no_arg,
    parse_ra_arg,
    parse_stop_arg,
)
from lx200_protocol import (
    LX200Command,
    LX200GotoResult,
    LX200MoveDirection,
    LX200SyncResult,
)
from lx200_server import CommandSpec


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
