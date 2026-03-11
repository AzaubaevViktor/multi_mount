import logging
import threading
import time
from typing import Callable
from lx200.base import LX200DECHandler, LX200Handler, LX200RAHandler
from lx200.guide_compensator import compute_pole_offset, compute_guide_speeds
from lx200.protocol import AlignmentMode
from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, HaPerSecond, Ha, SkyDirection, Second


class PolarCompensator:
    SETTLE_THRESHOLD_RA = HaPerSecond(0.05)
    SETTLE_THRESHOLD_DEC = DecPerSecond(0.05)

    CORRECTION_DELTA_S = 5
    DISABLED_RESET_AFTER_S = 6

    class Status:
        DISABLED = 'disabled'
        WAITING = 'waiting'
        SETTLE = 'settle'
        GUIDING = 'guiding'

    def __init__(self, do_correction: Callable[[HaPerSecond, DecPerSecond], None], settle_count: int = 6) -> None:
        self.logger = logging.getLogger("PolarCompensator")
        self._do_correction_func = do_correction

        self._settle_count = settle_count
        self._current_settle_count = 0
        self._last_correction_time = time.monotonic()

        self.eps_N: Dec = Dec(0)
        self.eps_E: Ha = Ha(0)

        self._last_guide_update: Second = Second.monotonic()
        self.current_ha: Ha = Ha(0)
        self.current_dec: Dec = Dec(0)

        self._prev_dec_drift: DecPerSecond = DecPerSecond(0)
        self._dec_drift: DecPerSecond = DecPerSecond(0)

        self._prev_ra_speed: HaPerSecond = STELLAR_SPEED
        self._ra_speed: HaPerSecond = STELLAR_SPEED

        self._updated_ra = threading.Event()
        self._updated_dec = threading.Event()

        self.status = self.Status.DISABLED
        self._working = True

        self._thread = threading.Thread(target=self._do_check, name="PolarCompensator")

    def start(self):
        self._thread.start()
    
    def stop(self):
        self._working = False
        self._updated_ra.set()
        self._updated_dec.set()
        self._thread.join()

    def disable(self, reset_rates: bool = True):
        self.logger.info("Disable polar compensator: reset_rates=%s", reset_rates)
        self._current_settle_count = 0
        self.status = self.Status.DISABLED
        self.eps_N = Dec(0)
        self.eps_E = Ha(0)

        if not reset_rates:
            return

        self._prev_dec_drift = DecPerSecond(0)
        self._dec_drift = DecPerSecond(0)
        self._prev_ra_speed = STELLAR_SPEED
        self._ra_speed = STELLAR_SPEED
        self._last_guide_update = Second.monotonic()
        self._last_correction_time = time.monotonic()
        self._updated_ra.clear()
        self._updated_dec.clear()
        self._do_correction(self._ra_speed, self._dec_drift)

    def _do_correction(self, ha_drift: HaPerSecond, dec_drift: DecPerSecond):
        self.logger.debug("Applying correction with ha_drift=%.4f, dec_drift=%.4f", ha_drift, dec_drift)
        self._do_correction_func(ha_drift, dec_drift)
        self._last_guide_update = Second.monotonic()

    def set_coordiantes(self, ha: Ha | None, dec: Dec | None):
        """ Update pointing coordinates """
        if ha is not None:
            self.current_ha = ha
        if dec is not None:
            self.current_dec = dec

    def set_guide_speed_dec(self, speed: DecPerSecond):
        self._prev_dec_drift = self._dec_drift
        self._dec_drift = speed
        self._last_guide_update = Second.monotonic()
        self._updated_dec.set()
    
    def set_guide_speed_ra(self, speed: HaPerSecond):
        self._prev_ra_speed = self._ra_speed
        self._ra_speed = speed
        self._last_guide_update = Second.monotonic()
        self._updated_ra.set()

    def _do_check(self):
        while self._working:
            ra_updated = self._updated_ra.wait(.5)
            dec_updated = self._updated_dec.wait(.5)

            if not (ra_updated or dec_updated):
                now = time.monotonic()
                since_last_guide = Second.monotonic() - self._last_guide_update

                if self.status in (self.Status.DISABLED, self.Status.WAITING) and since_last_guide > Second(self.DISABLED_RESET_AFTER_S):
                    if self._ra_speed != STELLAR_SPEED or self._dec_drift != DecPerSecond(0):
                        self.disable(reset_rates=True)
                    continue

                if self.status == self.Status.SETTLE:
                    self.status = self.Status.GUIDING

                if self.status == self.Status.GUIDING:
                    if (now - self._last_correction_time) > self.CORRECTION_DELTA_S:
                        self._do_correction(*compute_guide_speeds(self.eps_N, self.eps_E, self.current_ha, self.current_dec))
                        self._last_correction_time = now

                continue

            if self.status == self.Status.DISABLED:
                self.logger.info("Start waiting for external guide settle")
                self.status = self.Status.WAITING

            self._updated_ra.clear()
            self._updated_dec.clear()
            
            is_ra_settled = abs(self._prev_ra_speed - self._ra_speed) < self.SETTLE_THRESHOLD_RA
            is_dec_settled = abs(self._prev_dec_drift - self._dec_drift) < self.SETTLE_THRESHOLD_DEC
            self._do_correction(self._ra_speed, self._dec_drift)
            self._last_correction_time = time.monotonic()

            if is_ra_settled and is_dec_settled:
                self._current_settle_count += 1
            else:
                self.disable(reset_rates=False)

            if self._current_settle_count >= self._settle_count:
                self.status = self.Status.SETTLE
                self.eps_N, self.eps_E = compute_pole_offset(self._dec_drift, self._ra_speed, self.current_ha, self.current_dec)


class LX200Splitter(LX200Handler):
    def __init__(self, ra: LX200RAHandler, dec: LX200DECHandler) -> None:
        super().__init__()
        self.logger = logging.getLogger("splitter")
        self.ra = ra
        self.dec = dec
        self.logger.info("RA: %r; DEC: %r", self.ra, self.dec)

        self._active_guide_dec = False
        self._active_guide_ra = False

        self._polar_compensator = PolarCompensator(self._do_correction)
    
    def connect(self):
        self.ra.connect()
        self.dec.connect()
        self._polar_compensator.start()
        super().connect()

    def disable_polar_compensator(self):
        self._polar_compensator.disable()

    def stop(self):
        super().stop()
        try:
            self.ra.stop()
        except:
            pass

        try:
            self.dec.stop()
        except:
            pass

        try:
            self._polar_compensator.stop()
        except:
            pass

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def get_telescope_ra(self) -> Ha:
        ra = self.ra.get_telescope_ra()
        self._polar_compensator.set_coordiantes(ha=ra, dec=None)
        return ra

    def motor_position(self) -> tuple[Ha, Dec]:
        return (
            self.ra.motor_position()[0],
            self.dec.motor_position()[1],
        )
    
    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        result = self.ra.slew_to_ra(ra)
        result &= self.dec.slew_to_dec(dec)
        return result

    def slew_to_ra(self, position: Ha) -> bool:
        # TODO: Reimplement just slew_to with both ra and dec, because we need to set correct dec when crossing pole
        return self.ra.slew_to_ra(position)
    
    def slew_to_dec(self, position: Dec) -> bool:
        return self.dec.slew_to_dec(position)

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        result = self.ra.sync_telescope_ra(ra)
        result &= self.dec.sync_telescope_dec(dec)
        return result

    def sync_telescope_ra(self, position: Ha) -> bool:
        return self.ra.sync_telescope_ra(position)
    
    def get_telescope_dec(self) -> Dec:
        # Implement ra compenstaion when moving throught (+/-)90°
        dec = self.dec.get_telescope_dec()
        self._polar_compensator.set_coordiantes(ha=None, dec=dec)
        return dec
    
    def sync_telescope_dec(self, position: Dec) -> bool:
        return self.dec.sync_telescope_dec(position)
    
    def get_site1_name(self) -> str:
        return f"splitter_ra_{self.ra.get_site1_name()}_dec_{self.dec.get_site1_name()}"

    def get_distance(self) -> str:
        return self.ra.get_distance() + self.dec.get_distance()
    
    def halt_all(self) -> bool:
        try:
            self.ra.halt_all()
        except Exception:
            self.logger.exception("While stop RA")
        
        try:
            self.dec.halt_all()
        except Exception:
            self.logger.exception("While stop DEC")
        
        return True

    def move_east(self) -> bool:
        return self.ra.move_east()

    def move_north(self) -> bool:
        return self.dec.move_north()

    def move_south(self) -> bool:
        return self.dec.move_south()

    def move_west(self) -> bool:
        return self.ra.move_west()

    def halt_east(self) -> bool:
        try:
            return self.ra.halt_east()
        except Exception:
            self.logger.exception("While stop RA east")
            return False

    def halt_north(self) -> bool:
        try:
            return self.dec.halt_north()
        except Exception:
            self.logger.exception("While stop DEC north")
            return False

    def halt_south(self) -> bool:
        try:
            return self.dec.halt_south()
        except Exception:
            self.logger.exception("While stop DEC south")
            return False

    def halt_west(self) -> bool:
        try:
            return self.ra.halt_west()
        except Exception:
            self.logger.exception("While stop RA west")
            return False

    def set_slew_to_find(self) -> bool:
        ok = True
        try:
            self.ra.set_slew_to_find()
        except Exception:
            self.logger.exception("While set RA slew rate")
            ok = False
        try:
            self.dec.set_slew_to_find()
        except Exception:
            self.logger.exception("While set DEC slew rate")
            ok = False
        return ok
    
    def _do_correction(self, ha_drift: HaPerSecond, dec_drift: DecPerSecond):
        self.logger.debug("Applying correction with ha_drift=%.4f, dec_drift=%.4f", ha_drift, dec_drift)
        try:
            self.dec.set_tracking_speed(dec_drift, update_sky_speed=True)
        except Exception:
            self.logger.exception("While set DEC guide speed")

        try:
            self.ra.set_tracking_speed(ha_drift, update_sky_speed=True)
        except Exception:
            self.logger.exception("While set RA guide speed")

    def _convert_guide_speed(self, direction: SkyDirection, ms: int) -> None:
        dec_speed = None
        if (ra_speed := self.ra.calculate_guide_speed(direction, ms)) is None and (dec_speed := self.dec.calculate_guide_speed(direction, ms)) is None:
            raise ValueError(f"Unknown move direction: {direction}")
        
        # External pulse width controls the requested guide magnitude only.
        # We keep the last guide correction latched after the external guider
        # stops sending pulses, because these guide updates are also used to
        # estimate polar alignment error and continuously recompute tracking
        # correction relative to that measured pole offset.
        if ra_speed is not None:
            self._polar_compensator.set_guide_speed_ra(ra_speed)

        if dec_speed is not None:   
            self._polar_compensator.set_guide_speed_dec(dec_speed)

    def guide_east(self, ms: int):
        self._convert_guide_speed(SkyDirection.EAST, ms)

    def guide_north(self, ms: int):
        self._convert_guide_speed(SkyDirection.NORTH, ms)

    def guide_south(self, ms: int):
        self._convert_guide_speed(SkyDirection.SOUTH, ms)

    def guide_west(self, ms: int):
        self._convert_guide_speed(SkyDirection.WEST, ms)
