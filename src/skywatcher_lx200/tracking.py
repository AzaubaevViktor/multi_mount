from __future__ import annotations

from lx200.plugins.tracking import LX200TrackingBackend
from lx200.protocol import LX200SlewRate

from .mount import SkyWatcherMount


class SkyWatcherTrackingBackend(LX200TrackingBackend):
    def __init__(self, mount: SkyWatcherMount) -> None:
        self._mount = mount

    def initialize(self) -> None:
        self._mount.initialize()

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        self._mount.set_slew_rate(rate)

    def get_tracking_rate(self) -> str:
        return self._mount.get_tracking_rate()
