import math

from sky.constants import STELLAR_SPEED
from sky.physics import DecPerSecond, HaPerSecond, Dec, Ha

# TODO: Write formulas

def compute_pole_offset(dec_drift: DecPerSecond, ra_drift: HaPerSecond, ha: Ha, dec_: Dec) -> tuple[Dec, Ha]:
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
        raise ValueError("Move declination from the 0° (equaltor) - RA equation is degenerate")

    # Normalized right-hand sides
    rhs_dec = float(dec_drift) / float(STELLAR_SPEED.to_ha_deg_per_hour())  # from Dec equation
    rhs_ra  = float((ra_drift - STELLAR_SPEED) / STELLAR_SPEED) / tan_dec  # from RA equation

    eps_N = Dec(rhs_dec * math.cos(HA) + rhs_ra * math.sin(HA))
    eps_E = Ha(-rhs_dec * math.sin(HA) + rhs_ra * math.cos(HA))

    return eps_N, eps_E


def compute_guide_speeds(eps_N: Dec, eps_E: Ha, HA_deg: Ha, dec_deg: Dec) -> tuple[HaPerSecond, DecPerSecond]:
    """
    Computes theoretical d and k from known polar offset and star position
    (the forward problem).
    """
    HA  = math.radians(HA_deg.to_hours_deg())
    dec = math.radians(dec_deg.to_degrees())

    dec_drift = float(STELLAR_SPEED.to_ha_deg_per_hour()) * (float(eps_N) * math.cos(HA) - float(eps_E) * math.sin(HA))
    ra_drift = STELLAR_SPEED * (1.0 + math.tan(dec) * (float(eps_N) * math.sin(HA) + float(eps_E) * math.cos(HA)))
    return ra_drift, DecPerSecond(dec_drift)
