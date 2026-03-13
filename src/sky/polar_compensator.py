import logging
import math

from sky.constants import STELLAR_SPEED
from sky.physics import DecPerSecond, HaPerSecond, Dec, Ha, Second

"""
Polar misalignment model.

Let:
eps_N - polar axis error to the north
eps_E - polar axis error to the east
HA    - star hour angle
dec   - star declination
S     - sidereal tracking speed

Then the guide rates caused by polar misalignment at a given sky position are:

dec_drift / S_deg = eps_N * cos(HA) - eps_E * sin(HA)
(ra_drift - S) / (S * tan(dec)) = eps_N * sin(HA) + eps_E * cos(HA)

This is a 2x2 rotation system:

[rhs_dec] = [ cos(HA)  -sin(HA)] [eps_N]
[rhs_ra ]   [ sin(HA)   cos(HA)] [eps_E]

so from measured guide rates and current pointing we recover the polar offset:

eps_N = rhs_dec * cos(HA) + rhs_ra * sin(HA)
eps_E = -rhs_dec * sin(HA) + rhs_ra * cos(HA)

Then for any other sky position we solve the forward problem again and compute the
guide rates needed to compensate the same polar offset there.
"""

def compute_pole_offset(dec_drift: DecPerSecond, ra_drift: HaPerSecond, ha: Ha, dec_: Dec) -> tuple[Ha, Dec]:
    """
    dec_drift      - Dec drift ["/s], positive = north
    ra_drift      - RA drift ["/s]
    HA_deg - star hour angle [degrees], west is positive
    dec_deg- star declination [degrees]

    Returns (eps_N, eps_E) in arcseconds.
    eps_N > 0 - pole is north of the mount axis
    eps_E > 0 - pole is east of the mount axis
    """

    ha_deg = ha.to_hours_deg()
    dec_deg = dec_.to_degrees()

    HA  = math.radians(ha_deg)
    dec = math.radians(dec_deg)

    tan_dec = math.tan(dec)
    if abs(tan_dec) < 1e-6:
        raise ValueError("Declination is too close to 0° - RA equation is degenerate")

    # Normalized right-hand sides
    rhs_ra  = float((ra_drift - STELLAR_SPEED) / STELLAR_SPEED) / tan_dec  # from RA equation
    rhs_dec = float(dec_drift) / float(STELLAR_SPEED.to_ha_deg_per_hour())  # from Dec equation

    eps_E = Ha(-rhs_dec * math.sin(HA) + rhs_ra * math.cos(HA))
    eps_N = Dec(rhs_dec * math.cos(HA) + rhs_ra * math.sin(HA))

    return eps_E, eps_N


def compute_guide_speeds(eps_E: Ha, eps_N: Dec, HA_deg: Ha, dec_deg: Dec) -> tuple[HaPerSecond, DecPerSecond]:
    """
    Computes theoretical ra_speed and dec_speed from known polar offset and star position
    (the forward problem).
    """
    HA  = math.radians(HA_deg.to_hours_deg())
    dec = math.radians(dec_deg.to_degrees())
    
    dec_drift = float(STELLAR_SPEED.to_ha_deg_per_hour()) * (float(eps_N) * math.cos(HA) - float(eps_E) * math.sin(HA))
    ra_drift = STELLAR_SPEED * (1.0 + math.tan(dec) * (float(eps_N) * math.sin(HA) + float(eps_E) * math.cos(HA)))
    return ra_drift, DecPerSecond(dec_drift)


class PolarCompensator:
    """
    Based on current pointing position and stable guide speeds, computes the polar offset and guide speeds to replace external guiding.
    """

    STABLE_GUIDE_PULSES_COUNT = 5
    DROP_GUIDE_PULSES_COUNT_AFTER = Second(20)
    RA_SPEED_TOLERANCE = HaPerSecond(0.05)
    DEC_SPEED_TOLERANCE = DecPerSecond(0.05)

    def __init__(self):
        self.logger = logging.getLogger("PolarCompensator")

        self.current_ha: Ha = Ha(0)
        self.current_dec: Dec = Dec(0)

        self.eps_E: Ha | None = None
        self.eps_N: Dec | None = None
        
        self.ra_speed: HaPerSecond = STELLAR_SPEED
        self.dec_speed: DecPerSecond = DecPerSecond(0)

        # We expect RA and DEC guide pulses to arrive as a pair, so one shared timestamp is enough here.
        self.last_guide_pulse: Second = Second.monotonic()
        self.stable_guide_ra_pulses_count = 0
        self.stable_guide_dec_pulses_count = 0
    
    def update_position(self, ha: Ha, dec: Dec) -> None:
        self.current_ha = ha
        self.current_dec = dec
    
    def guide_ra(self, speed: HaPerSecond) -> None:
        prev_speed = self.ra_speed
        now = Second.monotonic()

        if now - self.last_guide_pulse > self.DROP_GUIDE_PULSES_COUNT_AFTER:
            self.stable_guide_ra_pulses_count = 0

        self.last_guide_pulse = now

        if abs(speed - prev_speed) < self.RA_SPEED_TOLERANCE:
            self.stable_guide_ra_pulses_count += 1
        else:
            self.stable_guide_ra_pulses_count = 0

        self.logger.info("RA (%d/%d) guide speed: %s -> %s", self.stable_guide_ra_pulses_count, self.STABLE_GUIDE_PULSES_COUNT, prev_speed, speed)
        
        self.ra_speed = speed

    def guide_dec(self, speed: DecPerSecond) -> None:
        prev_speed = self.dec_speed
        now = Second.monotonic()

        if now - self.last_guide_pulse > self.DROP_GUIDE_PULSES_COUNT_AFTER:
            self.stable_guide_dec_pulses_count = 0

        self.last_guide_pulse = now

        if abs(speed - prev_speed) < self.DEC_SPEED_TOLERANCE:
            self.stable_guide_dec_pulses_count += 1
        else:
            self.stable_guide_dec_pulses_count = 0
        
        self.logger.info("DEC (%d/%d) guide speed: %s -> %s", self.stable_guide_dec_pulses_count, self.STABLE_GUIDE_PULSES_COUNT, prev_speed, speed)

        self.dec_speed = speed

    def get_polar_offset(self) -> tuple[Ha, Dec]:
        if self.stable_guide_ra_pulses_count < self.STABLE_GUIDE_PULSES_COUNT or self.stable_guide_dec_pulses_count < self.STABLE_GUIDE_PULSES_COUNT:
            return Ha(0), Dec(0)

        return compute_pole_offset(self.dec_speed, self.ra_speed, self.current_ha, self.current_dec)

    def get_guide_speeds(self) -> tuple[HaPerSecond, DecPerSecond] | None:
        """ Returns actual guide speeds ONLY when no external guide and has stable guide speeds """
        is_external_guide = Second.monotonic() - self.last_guide_pulse < self.DROP_GUIDE_PULSES_COUNT_AFTER
        is_stable_guide = self.stable_guide_ra_pulses_count >= self.STABLE_GUIDE_PULSES_COUNT and self.stable_guide_dec_pulses_count >= self.STABLE_GUIDE_PULSES_COUNT

        if not is_external_guide and is_stable_guide:
            try:
                self.eps_E, self.eps_N = self.get_polar_offset()
                self.logger.info("Polar offset: ε_E: %s, ε_N: %s", self.eps_E, self.eps_N)
            except ValueError as e:
                self.logger.exception("Error getting polar offset:\n    %s", e)
                return None

            ha_speed, dec_speed = compute_guide_speeds(self.eps_E, self.eps_N, self.current_ha, self.current_dec)
            self.logger.info("Guide speeds: %s, %s (from ε_E: %s, ε_N: %s) for HA: %s, DEC: %s", ha_speed, dec_speed, self.eps_E, self.eps_N, self.current_ha, self.current_dec)
            return ha_speed, dec_speed

        if not is_stable_guide and not is_external_guide:
            self.ra_speed = STELLAR_SPEED
            self.dec_speed = DecPerSecond(0)
            self.eps_E = None
            if self.eps_N is not None or self.eps_E is not None:
                self.logger.info("No guide pulses for %.1fs, reset guide speeds to sidereal", Second.monotonic() - self.last_guide_pulse)
                self.eps_N = None
                self.eps_E = None
                return STELLAR_SPEED, DecPerSecond(0)
        
        self.logger.info("No guide speeds: external guide: %s, stable guide: %s", is_external_guide, is_stable_guide)

        return None
