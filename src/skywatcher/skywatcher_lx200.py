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
        self._manual_slew_rate = self.mount.MAX_RATE
    
    def connect(self):
        self.mount.connect()

        self.set_telescope_ra(LX200Ha.from_hours(0))

        self.mount.start_tracking()

    def get_telescope_ra(self) -> LX200Ha:
        now = time.monotonic()
        mount_seconds = self.mount.get_telescope_ra().to_seconds()

        elapsed_s = now - self._last_update_s

        expected_delta_seconds = elapsed_s * (self.mount.STELLAR_SPEED / DEGREES_PER_HOUR)
        # TODO: Write tests for 23:59:59 -> 00:00:01
        actual_delta_seconds = (mount_seconds - self._last_mount_seconds) % LX200Ha.SECONDS_PER_CIRCLE

        delta = expected_delta_seconds - actual_delta_seconds
        self.logger.debug("Calculated delta: %f = (%f - %f); %f", delta, expected_delta_seconds, actual_delta_seconds, self._ra_seconds)

        if abs(delta) < self._ACCEPTED_DELTA_S:
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
    
    def halt_all(self) -> bool:
        self.mount.wait_till_stop(do_stop=True)
        self.mount.resume_tracking()
        return True
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        # TODO: Need to keep in mind STELLAR_SPEED
        return self.mount.slew_to_ra(LX200Ha.from_seconds(self._ra_seconds - position.to_seconds()))

    def get_site1_name(self) -> str:
        return "skywatcher"
    
    def get_distance(self) -> str:
        if self.mount.get_status().slew_mode == SlewMode.GOTO:
            return "|"
        else:
            # Here we understand that INDI wants us to go to track mode
            self.mount.resume_tracking()
            return ""

    def set_slew_to_find(self) -> bool:
        self._manual_slew_rate = self.mount.MAX_RATE
        return True

    def move_east(self) -> bool:
        return self._start_manual_move(self._manual_slew_rate)

    def move_north(self) -> bool:
        return False

    def move_south(self) -> bool:
        return False

    def move_west(self) -> bool:
        return self._start_manual_move(-self._manual_slew_rate)

    def halt_east(self) -> bool:
        return self._stop_manual_move()

    def halt_north(self) -> bool:
        return False

    def halt_south(self) -> bool:
        return False

    def halt_west(self) -> bool:
        return self._stop_manual_move()

    def _start_manual_move(self, rate: float) -> bool:
        return self.mount.move_ra(rate)

    def _stop_manual_move(self) -> bool:
        self.mount.wait_till_stop(do_stop=True)
        self.mount.resume_tracking()
        return True
