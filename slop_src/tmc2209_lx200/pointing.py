from __future__ import annotations

from lx200.models import LX200Dec, LX200Ra
from lx200.plugins.pointing import LX200PointingBackend
from lx200.protocol import LX200GotoResult, LX200MoveDirection, LX200SyncResult

from .mount import TMC2209Mount


class TMC2209PointingBackend(LX200PointingBackend):
    def __init__(self, mount: TMC2209Mount) -> None:
        self._mount = mount

    def get_current_ra(self) -> LX200Ra:
        return self._mount.get_current_ra()

    def get_current_dec(self) -> LX200Dec:
        return self._mount.get_current_dec()

    def set_target_ra(self, ra: LX200Ra) -> bool:
        return self._mount.set_target_ra(ra)

    def set_target_dec(self, dec: LX200Dec) -> bool:
        return self._mount.set_target_dec(dec)

    def slew_to_target(self) -> LX200GotoResult:
        return self._mount.slew_to_target()

    def sync_to_target(self) -> LX200SyncResult:
        return self._mount.sync_to_target()

    def stop_all(self) -> None:
        self._mount.stop_all()

    def start_move(self, direction: LX200MoveDirection) -> None:
        self._mount.start_move(direction)

    def stop_move(self, direction: LX200MoveDirection) -> None:
        self._mount.stop_move(direction)
