from __future__ import annotations

from lx200.plugins.tracking import LX200TrackingBackend
from lx200.protocol import LX200SlewRate

from .mount import TMC2209Mount


class TMC2209TrackingBackend(LX200TrackingBackend):
    def __init__(self, mount: TMC2209Mount) -> None:
        self._mount = mount

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        self._mount.set_slew_rate(rate)
