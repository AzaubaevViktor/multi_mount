import threading
import time

from lx200.base import HaPerSecond, LX200RAHandler
from sky.physics import SECONDS_PER_DAY, Dec, Direction, Ha, Second
from .skywatcher import SkyWatcherMount, SkyWatcherStatus, SlewMode


class SkyWatcherLX200(LX200RAHandler):
    _ACCEPTED_DELTA_S = 0.01
    _STOP_GOTO_SECONDS = Ha(1)

    _GOTO_CHECK_INTERVAL_S = .5

    _DEFAULT_TRACKING_RATE = 1

    def __init__(self, mount: SkyWatcherMount) -> None:
        self.mount = mount

        super().__init__()

        self._manual_slew_rate = self.mount.MAX_SPEED

        self._goto_to: Ha | None = None
        self._goto_direction_sign: Direction = Direction.STOP
        self._check_goto_thread = threading.Thread(target=self._check_goto, name="SW_GOTO")
        self._last_check_goto: float = 0

        self._check_goto_thread.start()

    def _is_motor_connected(self) -> bool:
        return self.mount.is_connected
    
    def _get_motor_status(self) -> SkyWatcherStatus:
        return self.mount.get_status()
    
    def _get_motor_raw_position(self) -> Ha:
        return self.motor_position()[0]
    
    def _set_tracking_speed(self, rate: HaPerSecond):
        self.mount.start_tracking(rate)
    
    def _halt_motion(self):
        self.logger.info("Stop motion")
        self._goto_to = None
        self.mount.wait_till_stop(do_stop=True)
        self.logger.info("RA motion stopped")

    # TODO: Understand wtf and fix it
    MAGIC_SECONDS_MINUS_SLEW = Ha(3)

    def _check_goto(self):
        logger = self.logger.getChild("goto")
        circle_seconds = SECONDS_PER_DAY
        half_circle_seconds = Ha(float(circle_seconds / 2))

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

                    # TODO: Move this to smart goto method in splitter
                    delta_to_target_seconds = (_goto_to - current_ra + half_circle_seconds).wrap() - half_circle_seconds
                    delta_to_target_abs_seconds = abs(delta_to_target_seconds)

                    if delta_to_target_abs_seconds < self._STOP_GOTO_SECONDS:
                        logger.info("Stop mount in %s (%s), Δ=%.3fs", current_ra, _goto_to, delta_to_target_abs_seconds)
                        self.halt_motion()
                        
                        with self._position_update_lock:
                            _current_ra = self._mount_position_raw

                        logger.info("Finished GOTO to %s in %s with delta: %fs", 
                                    _goto_to, 
                                    _current_ra, 
                                    (_current_ra - _goto_to)
                                    )
                        
                        self._goto_to = None
                    else:
                        logger.debug("Continuing slewing, %.3fs still need to moved", delta_to_target_abs_seconds)
                        if self.mount.get_status().slew_mode != SlewMode.GOTO:
                            # start goto
                            raw_delta_seconds = delta_to_target_seconds
                            
                            real_rate = self.mount.get_slew_real_speed(raw_delta_seconds)
                            # Add sky moving approximation
                            real_delta_seconds = raw_delta_seconds + abs(raw_delta_seconds) / (self._sky_track_rate / real_rate)
                            if real_delta_seconds < Ha(0):
                                real_delta_seconds -= self.MAGIC_SECONDS_MINUS_SLEW  # add 6 magic seconds, because of accel/deccel/stop/star
                            real_delta = -real_delta_seconds  # why tf minus here and it works?

                            self.mount.slew_delta(real_delta)
                            if real_delta > Ha(0):
                                self._goto_direction_sign = Direction.FORWARD
                            elif real_delta < Ha(0):
                                self._goto_direction_sign = Direction.BACKWARD
                            else:
                                self._goto_direction_sign = Direction.STOP

                            logger.info("Run GOTO to %s from %s with delta %s (x%f)",
                                        _goto_to,
                                        current_ra,
                                        real_delta_seconds,
                                        real_delta_seconds / raw_delta_seconds,
                                        )
                        elif self._goto_direction_sign and ((delta_to_target_seconds > Ha(0)) ^ (self._goto_direction_sign == Direction.FORWARD)):
                            # Too far away
                            self.halt_motion()
                            logger.warning("GOTO to %s went too far away (%s) %s, stop motor",
                                            _goto_to, 
                                            current_ra,
                                            delta_to_target_seconds,
                                            )
                            self._goto_to = None
                except Exception:
                    logger.exception("While processing GOTO to %s", _goto_to)

            self._last_check_goto = time.monotonic()
    
    def stop(self):
        self._halt_motion()
        self._working = False
        super().stop()
        if self._check_goto_thread and self._check_goto_thread.is_alive():
            self._check_goto_thread.join(timeout=self._GOTO_CHECK_INTERVAL_S * 5)
        self.mount.disconnect()
    
    def connect(self):
        self.logger.info("Connect SkyWatcher LX200")
        self.mount.connect()

        self.sync_telescope_ra(Ha(0))

        self.mount.start_tracking(self.DEFAULT_TRACKING_RATE)
        self.logger.info("SkyWatcher LX200 connected")

    def get_telescope_ra(self) -> Ha:
        return self._mount_position_raw
    
    def motor_position(self) -> tuple[Ha, Dec]:
        return self.mount.get_telescope_ha(), Dec(0)
    
    def sync_telescope_ra(self, position: Ha) -> bool:
        self.logger.info("Sync RA to %s", position)
        with self._position_update_lock:
            self._mount_position_raw = position
            # Don't need to calculate delta
            self._motor_position_raw = self.mount.get_telescope_ha()
            self._last_update_s = Second.monotonic()
        return True

    def slew_to_ra(self, position: Ha) -> bool:
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
        self._manual_slew_rate = self.mount.MAX_SPEED
        return True

    def move_east(self) -> bool:
        return self.mount.move_ra(self._manual_slew_rate)

    def move_north(self) -> bool:
        return False

    def move_south(self) -> bool:
        return False

    def move_west(self) -> bool:
        return self.mount.move_ra(-self._manual_slew_rate)
