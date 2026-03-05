import math

from sky.constants import STELLAR_SPEED

def compute_pole_offset(d: float, k: float, HA_deg: float, dec_deg: float):
    """
    d      - Dec drift ["/s], positive = north
    k      - RA coefficient relative to sidereal (1.0 = exact)
    HA_deg - star hour angle [degrees], west is positive
    dec_deg- star declination [degrees]

    Returns (eps_N, eps_E) in arcseconds.
    eps_N > 0 - pole is north of the mount axis
    eps_E > 0 - pole is east of the mount axis
    """

    HA  = math.radians(HA_deg)
    dec = math.radians(dec_deg)

    tan_dec = math.tan(dec)
    if abs(tan_dec) < 1e-6:
        raise ValueError("Declination is too close to 0° - RA equation is degenerate")

    # Normalized right-hand sides
    rhs_dec = d / STELLAR_SPEED  # from Dec equation
    rhs_ra  = (k - 1) / tan_dec  # from RA equation

    eps_N = rhs_dec * math.cos(HA) + rhs_ra * math.sin(HA)
    eps_E = -rhs_dec * math.sin(HA) + rhs_ra * math.cos(HA)

    return eps_N, eps_E


def compute_guide_rates(eps_N: float, eps_E: float, HA_deg: float, dec_deg: float) -> tuple[float, float]:
    """
    Computes theoretical d and k from known polar offset and star position
    (the forward problem).
    """
    HA  = math.radians(HA_deg)
    dec = math.radians(dec_deg)
    
    d = STELLAR_SPEED * (eps_N * math.cos(HA) - eps_E * math.sin(HA))
    k = 1.0 + math.tan(dec) * (eps_N * math.sin(HA) + eps_E * math.cos(HA))
    return d, k


def main():
    print("=== Pole Offset Calculation ===\n")
    d       = float(input("Dec drift [\"/s] (+ north, - south): "))
    k       = float(input("RA coefficient (for example 1.0003): "))
    HA_deg  = float(input("Star hour angle [degrees] (+ west): "))
    dec_deg = float(input("Star declination [degrees]: "))

    eps_N, eps_E = compute_pole_offset(d, k, HA_deg, dec_deg)

    print(f"\nPole offset from the mount axis:")
    print(f"  ε_N = {eps_N:+.2f}\"  ({'pole north of' if eps_N > 0 else 'pole south of'} axis)")
    print(f"  ε_E = {eps_E:+.2f}\"  ({'pole east of' if eps_E > 0 else 'pole west of'} axis)")
    print(f"\nTotal offset: {math.hypot(eps_N, eps_E):.2f}\"")

    az_err = math.degrees(math.atan2(eps_E, eps_N))
    print(f"Offset direction (from north toward east): {az_err:.1f}°")
    print(f"\nTo align the axis with the pole:")
    print(f"  {'Increase' if eps_N < 0 else 'Decrease'} altitude by {abs(eps_N):.2f}\"")
    print(f"  Rotate azimuth {'east' if eps_E < 0 else 'west'} by {abs(eps_E):.2f}\"")


if __name__ == "__main__":
    main()
