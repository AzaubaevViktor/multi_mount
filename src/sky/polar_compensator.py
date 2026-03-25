import logging
from typing import Sequence

from sky.constants import STELLAR_SPEED
from sky.physics import AxisSpeed, DecPerSecond, HaPerSecond, Dec, Ha, Second

"""
Guide-speed stabilizer for polar-alignment workflow.

The module no longer computes a polar-offset solution from mount pointing.
It only tracks incoming guide pulses, detects when RA/DEC guide rates become
stable, and replays the last stable guide speeds after external guiding stops.
"""


class PolarCompensator:
    """
    Tracks guide-pulse timing and stable guide speeds.

    When external guide pulses stop and both axes were stable long enough, the
    last averaged guide speeds are replayed as sky speeds until a timeout or a
    new external guide pulse interrupts them.
    """

    STABLE_GUIDE_PULSES_COUNT = 5
    DROP_GUIDE_PULSES_COUNT_AFTER = Second(20)
    STOP_AXIS_AFTER = Second(4.1)
    RA_SPEED_TOLERANCE_PERCENT = 5
    DEC_SPEED_TOLERANCE_PERCENT = 5

    def __init__(self) -> None:
        self.logger: logging.Logger = logging.getLogger("PolarCompensator")

        self._ra_speeds: list[HaPerSecond]
        self._dec_speeds: list[DecPerSecond]
        self.stable_guide_ra_pulses_count: int
        self.stable_guide_dec_pulses_count: int
        self.last_guide_pulse: Second
        self.last_ra_guide_pulse: Second
        self.last_dec_guide_pulse: Second
        self.current_ha: Ha | None
        self.current_dec: Dec | None
        self.eps_E: Ha | None
        self.eps_N: Dec | None
        self.is_guiding: bool

        self.reset(last_guide_pulse=Second.monotonic())

    def reset(self, last_guide_pulse: Second = Second(0)) -> None:
        self.current_ha = None
        self.current_dec = None
        self.eps_E = None
        self.eps_N = None
        self.is_guiding = False

        self.ra_speed = None
        self.dec_speed = None

        self.stable_guide_ra_pulses_count = 0
        self.stable_guide_dec_pulses_count = 0

        self.last_guide_pulse = last_guide_pulse
        self.last_ra_guide_pulse = last_guide_pulse
        self.last_dec_guide_pulse = last_guide_pulse

    @property
    def ra_speed(self) -> HaPerSecond:
        return self._clean(self._ra_speeds)

    @ra_speed.setter
    def ra_speed(self, speed: HaPerSecond | None) -> None:
        """ 
        Guide is unstable and for now I can't find good values from source data, it's too noisy
        """
        if speed is None:
            self._ra_speeds = [STELLAR_SPEED]
            return
        self._ra_speeds.append(speed)
        self._ra_speeds = self._ra_speeds[-self.STABLE_GUIDE_PULSES_COUNT * 10:]

    @property
    def dec_speed(self) -> DecPerSecond:
        return self._clean(self._dec_speeds)

    @dec_speed.setter
    def dec_speed(self, speed: DecPerSecond | None) -> None:
        if speed is None:
            self._dec_speeds = [DecPerSecond(0)]
            return
        self._dec_speeds.append(speed)
        self._dec_speeds = self._dec_speeds[-self.STABLE_GUIDE_PULSES_COUNT * 4:]

    @staticmethod
    def _clean(values: Sequence[AxisSpeed]) -> AxisSpeed:
        return type(values[0])(sum(float(x) for x in values) / len(values))
    
    def guide_ra(self, speed: HaPerSecond) -> None:
        prev_speed = self._ra_speeds[-1]
        prev_average_speed = self.ra_speed
        now = Second.monotonic()

        if now - self.last_guide_pulse > self.DROP_GUIDE_PULSES_COUNT_AFTER:
            self.stable_guide_ra_pulses_count = 0

        self.last_guide_pulse = now
        self.last_ra_guide_pulse = now

        percent_delta = (speed - prev_speed) / (prev_speed if prev_speed != 0 else (speed if speed != 0 else 1)) * 100

        if self.stable_guide_ra_pulses_count == 0 and prev_speed == STELLAR_SPEED:
            self.stable_guide_ra_pulses_count = 1
        elif abs(percent_delta) < self.RA_SPEED_TOLERANCE_PERCENT:
            self.stable_guide_ra_pulses_count += 1
        else:
            self.stable_guide_ra_pulses_count = 0

        self.ra_speed = speed
        
        self.logger.info("RA (%d/%d) Δ%.3f%% guide speed: \n%s (added %s) -> %s", self.stable_guide_ra_pulses_count, self.STABLE_GUIDE_PULSES_COUNT, percent_delta, prev_average_speed, speed, self.ra_speed)

    def guide_dec(self, speed: DecPerSecond) -> None:
        prev_speed = self._dec_speeds[-1]
        prev_average_speed = self.dec_speed
        now = Second.monotonic()

        if now - self.last_guide_pulse > self.DROP_GUIDE_PULSES_COUNT_AFTER:
            self.stable_guide_dec_pulses_count = 0

        self.last_guide_pulse = now
        self.last_dec_guide_pulse = now

        percent_delta = (speed - prev_speed) / (prev_speed if prev_speed != 0 else (speed if speed != 0 else 1)) * 100

        if self.stable_guide_dec_pulses_count == 0 and prev_speed == DecPerSecond(0):
            self.stable_guide_dec_pulses_count = 1
        elif abs(percent_delta) < self.DEC_SPEED_TOLERANCE_PERCENT:
            self.stable_guide_dec_pulses_count += 1
        else:
            self.stable_guide_dec_pulses_count = 0

        self.dec_speed = speed

        self.logger.info("DEC (%d/%d) Δ%.3f%% guide speed: \n%s (added %s) -> %s", self.stable_guide_dec_pulses_count, self.STABLE_GUIDE_PULSES_COUNT, percent_delta, prev_average_speed, speed, self.dec_speed)

    def get_guide_speeds(self) -> tuple[HaPerSecond | None, DecPerSecond | None] | None:
        """Returns guide speeds to apply when external guiding is inactive."""
        now = Second.monotonic()
        is_external_guide = now - self.last_guide_pulse < self.DROP_GUIDE_PULSES_COUNT_AFTER
        is_stable_guide = self.stable_guide_ra_pulses_count >= self.STABLE_GUIDE_PULSES_COUNT and self.stable_guide_dec_pulses_count >= self.STABLE_GUIDE_PULSES_COUNT
        is_external_guide_ra = now - self.last_ra_guide_pulse < self.STOP_AXIS_AFTER
        is_external_guide_dec = now - self.last_dec_guide_pulse < self.STOP_AXIS_AFTER

        if is_external_guide:
            self.is_guiding = False

            if not is_external_guide_ra and is_external_guide_dec and self.ra_speed != STELLAR_SPEED:
                self.logger.info("No RA guide pulse for %.1fs while DEC external guide is active, stop RA axis", now - self.last_ra_guide_pulse)
                self.ra_speed = None
                return STELLAR_SPEED, None

            if not is_external_guide_dec and is_external_guide_ra and self.dec_speed != DecPerSecond(0):
                self.logger.info("No DEC guide pulse for %.1fs while RA external guide is active, stop DEC axis", now - self.last_dec_guide_pulse)
                self.dec_speed = None
                return None, DecPerSecond(0)

        if not is_external_guide and is_stable_guide:
            self.is_guiding = True
            self.logger.info("Replay stable guide speeds: %s, %s", self.ra_speed, self.dec_speed)
            return self.ra_speed, self.dec_speed

        if not is_stable_guide and not is_external_guide:
            had_guide_state = (
                self.stable_guide_ra_pulses_count > 0
                or self.stable_guide_dec_pulses_count > 0
                or self.ra_speed != STELLAR_SPEED
                or self.dec_speed != DecPerSecond(0)
            )
            idle_for = now - self.last_guide_pulse
            self.reset()
            if had_guide_state:
                self.logger.info("No guide pulses for %.1fs, reset guide speeds to sidereal", idle_for)
                return STELLAR_SPEED, DecPerSecond(0)

        self.is_guiding = False
        self.logger.info("No guide speeds: ra: %s, dec: %s, external guide: %s, stable guide: %s", is_external_guide_ra, is_external_guide_dec, is_external_guide, is_stable_guide)

        return None
