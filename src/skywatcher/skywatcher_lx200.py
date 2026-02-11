import logging
import threading
import time

from lx200.base import LX200Base
from lx200.protocols import LX200Ha
from .skywatcher import SkyWatcherMount, SlewMode


DEGREES_PER_HOUR = 15


class SkyWatcherLX200(LX200Base):
    _ACCEPTED_DELTA_S = 0.01
    _RA_CHECK_TIME_S = .25
    _STOP_GOTO_SECONDS = 1

    def __init__(self, mount: SkyWatcherMount) -> None:
        self.logger = logging.getLogger("SkyWatcherLX200")
        self.mount = mount
        self._ra_seconds = 0.0
        self._last_mount_seconds: float = 0
        self._last_update_s: float = 0
        self._manual_slew_rate = self.mount.MAX_RATE

        self._goto_to: LX200Ha | None = None
        self._goto_direction_sign: int = 0
        
        self._check_ra_thread = threading.Thread(target=self._do_check_ra, name="SW_RA")
        self._check_goto_thread = threading.Thread(target=self._check_goto, name="SW_GOTO")
        self._ra_update_lock = threading.Lock()
        self._working = True

    def _do_check_ra(self):
        while self._working:
            if not self.mount.is_connected:
                time.sleep(self._RA_CHECK_TIME_S)
                continue
            
            # Base RA update
            with self._ra_update_lock:
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

    def _check_goto(self):
        logger = self.logger.getChild("goto")

        while self._working:
            if not self.mount.is_connected:
                time.sleep(self._RA_CHECK_TIME_S)
                continue

            if self._goto_to:
                with self._ra_update_lock:
                    current_ra = self._ra_seconds

                if current_ra - self._goto_to.to_seconds() < self._STOP_GOTO_SECONDS:
                    self.mount.wait_till_stop(do_stop=True)
                    self.mount.resume_tracking()
                    
                    with self._ra_update_lock:
                        _current_ra = self._ra_seconds
                    _current_ra = LX200Ha.from_seconds(_current_ra)
                    logger.info("GOTO to %s finished in %s with delta: %fs", 
                                self._goto_to, 
                                _current_ra, 
                                _current_ra - self._goto_to
                                )
                    
                    self._goto_to = None
                else:
                    if self.mount.get_status().slew_mode != SlewMode.GOTO:
                        # start goto
                        raw_delta_seconds = current_ra - self._goto_to.to_seconds()
                        
                        # Add sky moving approximation
                        real_delta_seconds = raw_delta_seconds + abs(raw_delta_seconds) / (self.mount.STELLAR_SPEED / DEGREES_PER_HOUR)
                        real_delta = LX200Ha.from_seconds(real_delta_seconds)
                        
                        self.mount.slew_to_ra(real_delta)
                        self._goto_direction_sign = 1 if (current_ra - self._goto_to.to_seconds() > 0) else -1

                        logger.info("Run GOTO to %s from %s with delta %s (x%f)",
                                    self._goto_to,
                                    LX200Ha.from_seconds(current_ra),
                                    real_delta,
                                    real_delta_seconds / raw_delta_seconds,
                                    )
                    else:
                        if ((delta_seconds := (current_ra - self._goto_to.to_seconds()) > 0) ^ (self._goto_direction_sign > 0)):
                            # Too far away
                            self.mount.gracefully_stop_motor()
                            logger.warning("GOTO to %s went too far away (%s) %s, stop motor",
                                           self._goto_to, 
                                           LX200Ha.from_seconds(current_ra),
                                           LX200Ha.from_seconds(delta_seconds),
                                           )
                        

    def __del__(self):
        self._working = False
        if self._check_ra_thread and self._check_ra_thread.is_alive():
            self._check_ra_thread.join(timeout=self._RA_CHECK_TIME_S * 5)
        if self._check_goto_thread and self._check_goto_thread.is_alive():
            self._check_goto_thread.join(timeout=self._RA_CHECK_TIME_S * 5)
    
    def connect(self):
        self.mount.connect()

        self.sync_telescope_ra(LX200Ha.from_hours(0))

        self.mount.start_tracking()

    def get_telescope_ra(self) -> LX200Ha:
        ra_seconds = int(round(self._ra_seconds)) % LX200Ha.SECONDS_PER_CIRCLE
        return LX200Ha.from_seconds(ra_seconds)
    
    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        with self._ra_update_lock:
            self._ra_seconds = position.to_seconds()
            # Don't need to calculate delta
            self._last_mount_seconds = self.mount.get_telescope_ra().to_seconds()
            self._last_update_s = time.monotonic()
        return True
    
    def halt_all(self) -> bool:
        self.mount.wait_till_stop(do_stop=True)
        self.mount.resume_tracking()
        return True
    
    def slew_to_ra(self, position: LX200Ha) -> bool:
        self._goto_to = position
        return True

    def get_site1_name(self) -> str:
        return "skywatcher"
    
    def get_distance(self) -> str:
        if self._goto_to:
            return "|"
        else:
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
    
    def guide_west(self) -> bool:
        return self.mount.set_ra_rate(1.5)
    
    def guide_east(self) -> bool:
        return self.mount.set_ra_rate(0.5)
    
    def guide_reset(self) -> bool:
        return self.mount.set_ra_rate(1)
