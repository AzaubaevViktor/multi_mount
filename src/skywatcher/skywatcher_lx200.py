import logging
import time

from lx200.base import LX200Base
from lx200.protocols import LX200Ha
from .skywatcher import SkyWatcherMount, SlewMode


DEGREES_PER_HOUR = 15


class SkyWatcherLX200(LX200Base):
    _ACCEPTED_DELTA_S = 0.01

    def __init__(self, mount: SkyWatcherMount) -> None:
        self.logger = logging.getLogger("SkyWatcherLX200")
        self.mount = mount
        self._ra_seconds = 0.0
        self._last_mount_seconds: float = 0
        self._last_update_s: float = 0
    
    def connect(self):
        self.mount.connect()

        self.set_telescope_ra(LX200Ha.from_hours(0))
        self._last_mount_seconds = 0
        self._last_mount_seconds = time.monotonic()

        self.mount.start_tracking()

    def get_telescope_ra(self) -> LX200Ha:
        now = time.monotonic()
        mount_seconds = self.mount.get_telescope_ra().to_seconds()

        elapsed_s = now - self._last_update_s

        expected_delta_seconds = elapsed_s * (self.mount.STELLAR_SPEED / DEGREES_PER_HOUR)
        # TODO: Write tests for 23:59:59 -> 00:00:01
        actual_delta_seconds = (mount_seconds - self._last_mount_seconds) % LX200Ha.SECONDS_PER_CIRCLE

        delta = expected_delta_seconds - actual_delta_seconds
        self.logger.debug("Calculated delta: %f = (%f - %f)", delta, expected_delta_seconds, actual_delta_seconds)

        if delta < self._ACCEPTED_DELTA_S:
            delta = 0
        
        self._ra_seconds = (self._ra_seconds + delta) % LX200Ha.SECONDS_PER_CIRCLE

        self._last_mount_seconds = float(mount_seconds)
        self._last_update_s = now

        ra_seconds = int(round(self._ra_seconds)) % LX200Ha.SECONDS_PER_CIRCLE
        return LX200Ha.from_seconds(ra_seconds)
    
    def set_telescope_ra(self, position: LX200Ha) -> bool:
        self._ra_seconds = float(position.to_seconds())
        # Don't need to calculate delta
        self._last_mount_seconds = self.mount.get_telescope_ra().to_seconds()
        self._last_update_s = time.monotonic()
        return True
    
    def stop(self) -> bool:
        self.mount.gracefully_stop_motor()
        return True
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        return self.mount.slew_to_ra(position)

    def get_site1_name(self) -> str:
        return "skywatcher"
    
    def get_distance(self) -> str:
        if self.mount.get_status().slew_mode == SlewMode.GOTO:
            return "|"
        else:
            return ""

