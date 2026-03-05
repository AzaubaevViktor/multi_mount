import logging
import threading
import time
from typing import Callable
from lx200.base import LX200DECHandler, LX200Handler, LX200RAHandler
from lx200.guide_compensator import compute_pole_offset, compute_guide_rates
from lx200.protocol import AlignmentMode
from lx200.protocols import Dec, Ha


class PolarCompensator:
    SETTLE_THRESHOLD_RA_COEF = 0.05
    SETTLE_THRESHOLD_DEC = 0.05

    CORRECTION_DELTA_S = 5

    class Status:
        DISABLED = 'disabled'
        SETTLE = 'settle'
        GUIDING = 'guiding'

    def __init__(self, do_correction: Callable[[float, float], None], settle_count: int = 6) -> None:
        self.logger = logging.getLogger("PolarCompensator")
        self._do_correction_func = do_correction

        self._settle_count = settle_count
        self._current_settle_count = 0
        self._last_correction_time = time.monotonic()

        self.eps_N: float = 0
        self.eps_E: float = 0

        self._last_guide_update: float = time.monotonic()
        self.current_ha: float = 0
        self.current_dec: float = 0

        self._prev_drift: float = 0
        self._dec_drift: float = 0

        self._prev_ra_coeff: float = 1.0
        self._ra_coeff: float = 1.0

        self._updated = threading.Event()

        self.status = self.Status.DISABLED
        self._working = True

        self._thread = threading.Thread(target=self._do_check, name="PolarCompensator")

    def start(self):
        self._thread.start()
    
    def stop(self):
        self._working = False
        self._updated.set()
        self._thread.join()

    def _do_correction(self, d: float, k: float):
        self.logger.debug("Applying correction with d=%.4f, k=%.4f", d, k)
        self._do_correction_func(d, k)

    def set_coordiantes(self, ha: float | None, dec: float | None):
        if ha is not None:
            self.current_ha = ha
        if dec is not None:
            self.current_dec = dec

    def set_guide_speed_dec(self, speed: float):
        self._prev_drift = self._dec_drift
        self._dec_drift = speed
        self._last_guide_update = time.monotonic()
        self._updated.set()
    
    def set_guide_speed_ra(self, coeff: float):
        self._prev_ra_coeff = self._ra_coeff
        self._ra_coeff = coeff
        self._last_guide_update = time.monotonic()
        self._updated.set()

    def _do_check(self):
        while self._working:
            if not self._updated.wait(.5):
                now = time.monotonic()

                if self.status == self.Status.SETTLE:
                    self.status = self.Status.GUIDING

                if self.status == self.Status.GUIDING:
                    if (now - self._last_correction_time) > self.CORRECTION_DELTA_S:
                        self._do_correction(*compute_guide_rates(self.eps_N, self.eps_E, self.current_ha, self.current_dec))
                        self._last_correction_time = now

                continue
            
            is_ra_settled = abs(self._prev_ra_coeff - self._ra_coeff) < self.SETTLE_THRESHOLD_RA_COEF
            is_dec_settled = abs(self._prev_drift - self._dec_drift) < self.SETTLE_THRESHOLD_DEC
            self._do_correction(self._dec_drift, self._ra_coeff)
            if is_ra_settled and is_dec_settled:
                self._current_settle_count += 1
            else:
                self._current_settle_count = 0
                self.status = self.Status.DISABLED

            if self._current_settle_count >= self._settle_count:
                self.status = self.Status.SETTLE
                self.eps_N, self.eps_E = compute_pole_offset(self._dec_drift, self._ra_coeff, self.current_ha, self.current_dec)


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
        self._polar_compensator.set_coordiantes(ha=ra.to_seconds(), dec=None)
        return ra

    def motor_position(self) -> tuple[float, float]:
        return (
            self.ra.motor_position()[0],
            self.dec.motor_position()[1],
        )
    
    def slew_to_ra(self, position: Ha) -> bool:
        return self.ra.slew_to_ra(position)
    
    def slew_to_dec(self, position: Dec) -> bool:
        return self.dec.slew_to_dec(position)

    def sync_telescope_ra(self, position: Ha) -> bool:
        return self.ra.sync_telescope_ra(position)
    
    def get_telescope_dec(self) -> Dec:
        dec = self.dec.get_telescope_dec()
        self._polar_compensator.set_coordiantes(ha=None, dec=dec.to_arcseconds())
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
    
    def _do_correction(self, d: float, k: float):
        self.logger.debug("Applying correction with d=%.4f, k=%.4f", d, k)
        try:
            # TODO: Somehow we need to set dec rate in arcsec/sec directly
            self.dec.set_tracking_rate(d, update_sky_rate=True)
        except Exception:
            self.logger.exception("While set DEC guide speed")

        try:
            # TODO: Maybe we need to set RA rate in sec/sec instead of coefficient
            self.ra.set_tracking_rate(k, update_sky_rate=True)
        except Exception:
            self.logger.exception("While set RA guide speed")

    def guide_east(self, ms: int):
        self._active_guide_ra = True
        self.ra.guide_east(ms)

    def guide_north(self, ms: int):
        self._active_guide_dec = True
        self.dec.guide_north(ms)

    def guide_south(self, ms: int):
        self._active_guide_dec = True
        self.dec.guide_south(ms)

    def guide_west(self, ms: int):
        self._active_guide_ra = True
        self.ra.guide_west(ms)
