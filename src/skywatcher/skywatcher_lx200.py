import threading
import time

from lx200.base import LX200RAHandler
from lx200.protocols import LX200Ha
from .skywatcher import SkyWatcherMount, SkyWatcherStatus, SlewMode


DEGREES_PER_HOUR = 15


class SkyWatcherLX200(LX200RAHandler):
    _ACCEPTED_DELTA_S = 0.01
    _STOP_GOTO_SECONDS = 1

    _GOTO_CHECK_INTERVAL_S = .5

    _DEFAULT_TRACKING_RATE = 1

    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount

        super().__init__()

        self._manual_slew_rate = self.mount.MAX_RATE

        self._goto_to: LX200Ha | None = None  # TODO: Refactor to float
        self._goto_direction_sign: int = 0
        self._check_goto_thread = threading.Thread(target=self._check_goto, name="SW_GOTO")
        self._last_check_goto: float = 0

        self._check_goto_thread.start()

    def _is_motor_connected(self) -> bool:
        return self.mount.is_connected
    
    def _get_motor_status(self) -> SkyWatcherStatus:
        return self.mount.get_status()
    
    def _get_motor_raw_position(self) -> float:
        return self.motor_position()[0]
    
    def _get_default_tracking_speed(self) -> float:
        return self.mount.STELLAR_SPEED / DEGREES_PER_HOUR
    
    def _wrap_mount_position(self, mount_position: float) -> float:
        return mount_position % LX200Ha.SECONDS_PER_CIRCLE
    
    def _set_tracking_rate(self, rate: float):
        self.mount.start_tracking(rate)
    
    def _halt_motion(self):
        self.logger.info("Stop motion")
        self._goto_to = None
        self.mount.wait_till_stop(do_stop=True)
        self.logger.info("RA motion stopped")

    # TODO: Understand wtf and fix it
    MAGIC_SECONDS_MINUS_SLEW = 3

    def _check_goto(self):
        logger = self.logger.getChild("goto")
        circle_seconds = LX200Ha.SECONDS_PER_CIRCLE
        half_circle_seconds = circle_seconds / 2

        while self._working:
            if not self.mount.is_connected:
                time.sleep(self._GOTO_CHECK_INTERVAL_S)
                continue
            
            if (delay := self._GOTO_CHECK_INTERVAL_S - (time.monotonic() - self._last_check_goto)) > 0:
                time.sleep(delay)

            _goto_to = self._goto_to  # Prevent race (look at halt_all)

            if _goto_to:
                try:
                    with self._position_update_lock:
                        current_ra = self._mount_position_raw

                    delta_to_target_seconds = (_goto_to.to_seconds() - current_ra + half_circle_seconds) % circle_seconds - half_circle_seconds
                    delta_to_target_abs_seconds = abs(delta_to_target_seconds)

                    if delta_to_target_abs_seconds < self._STOP_GOTO_SECONDS:
                        logger.info("Stop mount in %s (%s), Δ=%.3fs", LX200Ha.from_seconds(current_ra), _goto_to, delta_to_target_abs_seconds)
                        self.halt_motion()
                        
                        with self._position_update_lock:
                            _current_ra = self._mount_position_raw
                        _current_ra = LX200Ha.from_seconds(_current_ra)

                        logger.info("Finished GOTO to %s in %s with delta: %fs", 
                                    _goto_to, 
                                    _current_ra, 
                                    (_current_ra - _goto_to).to_seconds()
                                    )
                        
                        self._goto_to = None
                    else:
                        logger.debug("Continuing slewing, %.3fs still need to moved", delta_to_target_abs_seconds)
                        if self.mount.get_status().slew_mode != SlewMode.GOTO:
                            # start goto
                            raw_delta_seconds = delta_to_target_seconds
                            
                            real_rate = self.mount.get_slew_real_rate(raw_delta_seconds)
                            # Add sky moving approximation
                            real_delta_seconds = raw_delta_seconds + abs(raw_delta_seconds) / self._get_default_tracking_speed() / real_rate
                            if real_delta_seconds < 0:
                                real_delta_seconds -= self.MAGIC_SECONDS_MINUS_SLEW  # add 6 magic seconds, because of accel/deccel/stop/star
                            mount_delta_seconds = -real_delta_seconds  # why tf minus here and it works?
                            real_delta = LX200Ha.from_seconds(mount_delta_seconds)

                            self.mount.slew_delta(real_delta)
                            if raw_delta_seconds > 0:
                                self._goto_direction_sign = 1
                            elif raw_delta_seconds < 0:
                                self._goto_direction_sign = -1
                            else:
                                self._goto_direction_sign = 0

                            logger.info("Run GOTO to %s from %s with delta %s (x%f)",
                                        _goto_to,
                                        LX200Ha.from_seconds(current_ra),
                                        real_delta_seconds,
                                        real_delta_seconds / raw_delta_seconds,
                                        )
                        elif self._goto_direction_sign and ((delta_to_target_seconds > 0) ^ (self._goto_direction_sign > 0)):
                            # Too far away
                            self.halt_motion()
                            logger.warning("GOTO to %s went too far away (%s) %s, stop motor",
                                            _goto_to, 
                                            LX200Ha.from_seconds(current_ra),
                                            delta_to_target_seconds,
                                            )
                            self._goto_to = None
                except Exception:
                    logger.exception("While processing GOTO to %s", _goto_to)

            self._last_check_goto = time.monotonic()
    
    def stop(self):
        self.halt_motion()
        self._working = False
        super().stop()
        if self._check_goto_thread and self._check_goto_thread.is_alive():
            self._check_goto_thread.join(timeout=self._GOTO_CHECK_INTERVAL_S * 5)
        self.mount.disconnect()
    
    def connect(self):
        self.logger.info("Connect SkyWatcher LX200")
        self.mount.connect()

        self.sync_telescope_ra(LX200Ha.from_hours(0))

        self.mount.start_tracking(self.DEFAULT_TRACKING_RATE)
        self.logger.info("SkyWatcher LX200 connected")

    def get_telescope_ra(self) -> LX200Ha:
        ra_seconds = int(round(self._mount_position_raw)) % LX200Ha.SECONDS_PER_CIRCLE
        return LX200Ha.from_seconds(ra_seconds)
    
    def motor_position(self) -> tuple[float, float]:
        return self.mount.get_telesope_seconds(), 0
    
    def sync_telescope_ra(self, position: LX200Ha) -> bool:
        self.logger.info("Sync RA to %s", position)
        with self._position_update_lock:
            self._mount_position_raw = position.to_seconds()
            # Don't need to calculate delta
            self._motor_position_raw = self.mount.get_telesope_seconds()
            self._last_update_s = time.monotonic()
        return True

    def slew_to_ra(self, position: LX200Ha) -> bool:
        self.logger.info("Queue GOTO RA to %s", position)
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
        return self.mount.move_ra(self._manual_slew_rate)

    def move_north(self) -> bool:
        return False

    def move_south(self) -> bool:
        return False

    def move_west(self) -> bool:
        return self.mount.move_ra(-self._manual_slew_rate)
