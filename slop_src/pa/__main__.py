import math
import re
import argparse
import datetime as _dt
from zoneinfo import ZoneInfo
from dataclasses import dataclass
from typing import List, Tuple, Union
import random

AngleLike = Union[float, int, str]

class Constants:
    POINTS_REQUIRED = 3
    POINT_INDEX_START = 1
    POINT_INDEX_END_OFFSET = 1
    COORD_PRINT_DECIMALS = 6
    ZERO = 0.0
    ONE = 1.0
    NEG_ONE = -1.0
    INPUT_PROMPT_TEMPLATE = "Line {index}/{total}: "
    PARSED_LINE_TEMPLATE = "Parsed line {index}: RA={ra} h, Dec={dec} deg"
    PARSE_ERROR_TEMPLATE = "Parse error: {error}"
    STDIN_USAGE_MESSAGE = (
        "Need {count} non-empty lines on stdin. Example:\n"
        "  Syncing to RA (10h 44m 23s) DEC ( 33° 04' 58\")\n"
        "  Syncing to RA (10h 33m 12s) DEC ( 33° 05' 01\")\n"
        "  Syncing to RA (10h 44m 00s) DEC ( 33° 04' 42\")\n"
    )
    DIRECTION_ARROW_LEN = 0.6
    DIRECTION_ARC_RADIUS = 0.55
    DIRECTION_ARC_SWEEP_DEG = 55.0
    DIRECTION_LABEL_SCALE = 1.1
    DIRECTION_EPS_DEG = 1e-6
    DIRECTION_ALT_RAISE_LABEL = "ALT: raise"
    DIRECTION_ALT_LOWER_LABEL = "ALT: lower"
    DIRECTION_AZ_EAST_LABEL = "AZ: east"
    DIRECTION_AZ_WEST_LABEL = "AZ: west"
    DIRECTION_ALT_COLOR = "tab:green"
    DIRECTION_AZ_COLOR = "tab:orange"
    PLOT_SHOW_HELP = "Show interactive 3D window in addition to saving PNG."

class InputLineParseError(Exception):
    pass

class InputCountError(Exception):
    pass

@dataclass
class EqPoint:
    ra_hours: AngleLike  # RA in hours (0..24) or "HH:MM:SS"
    dec_deg: AngleLike   # Dec in degrees (-90..+90) or "±DD:MM:SS"

_NUM_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")

def _extract_ra_part(s: str) -> str:
    up = s.upper()
    if "RA" in up and "DEC" in up:
        part = up.split("RA", 1)[1]
        part = part.split("DEC", 1)[0]
        return part
    return s

def _extract_dec_part(s: str) -> str:
    up = s.upper()
    if "RA" in up and "DEC" in up:
        part = up.split("DEC", 1)[1]
        return part
    return s

def _parse_ra_to_hours(s: str) -> float:
    s = _extract_ra_part(s)
    nums = _NUM_RE.findall(s)
    if len(nums) < 3:
        raise ValueError(f"RA must have 3 fields (H M S), got: {s!r}")
    h = float(nums[0])
    m = float(nums[1])
    sec = float(nums[2])
    if m < 0 or m >= 60 or sec < 0 or sec >= 60:
        raise ValueError(f"Invalid RA minutes/seconds in: {s!r}")
    return h + m / 60.0 + sec / 3600.0

def _parse_dec_to_degrees(s: str) -> float:
    s = _extract_dec_part(s).strip()
    sign = 1.0
    if s.startswith("-"):
        sign = -1.0
    nums = _NUM_RE.findall(s)
    if len(nums) < 3:
        raise ValueError(f"Dec must have 3 fields (D M S), got: {s!r}")
    d = float(nums[0])
    m = float(nums[1])
    sec = float(nums[2])
    if m < 0 or m >= 60 or sec < 0 or sec >= 60:
        raise ValueError(f"Invalid Dec minutes/seconds in: {s!r}")
    return sign * (abs(d) + m / 60.0 + sec / 3600.0)

def parse_ra_hours(value: AngleLike) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        return _parse_ra_to_hours(value)
    raise TypeError(f"Unsupported RA type: {type(value)}")

def parse_dec_deg(value: AngleLike) -> float:
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        return _parse_dec_to_degrees(value)
    raise TypeError(f"Unsupported Dec type: {type(value)}")

def parse_sync_line(line: str) -> EqPoint:
    """Parse a line like: Syncing to RA (10h 44m 23s) DEC ( 33° 04' 58")"""
    # Reuse the robust parsers that can extract RA/DEC parts.
    ra_h = _parse_ra_to_hours(line)
    dec_d = _parse_dec_to_degrees(line)
    # Store as numeric to avoid re-parsing later
    return EqPoint(ra_hours=ra_h, dec_deg=dec_d)

# TODO: prompt
def _parse_point_line(line: str) -> EqPoint:
    try:
        point = parse_sync_line(line)
    except Exception as exc:
        parts = line.split(None, 1)
        if len(parts) != 2:
            raise InputLineParseError(f"Invalid line format: {line!r}") from exc
        point = EqPoint(ra_hours=parts[0], dec_deg=parts[1])
    return EqPoint(
        ra_hours=parse_ra_hours(point.ra_hours),
        dec_deg=parse_dec_deg(point.dec_deg),
    )

def _format_point(point: EqPoint) -> Tuple[str, str]:
    fmt = f".{Constants.COORD_PRINT_DECIMALS}f"
    ra_str = format(point.ra_hours, fmt)
    dec_str = format(point.dec_deg, fmt)
    return ra_str, dec_str

def _default_points() -> List[EqPoint]:
    """Built-in example points for quick testing."""
    return [
        EqPoint(ra_hours=parse_ra_hours("09:59:22"), dec_deg=parse_dec_deg("61:02:17")),
        EqPoint(ra_hours=parse_ra_hours("10:24:50"), dec_deg=parse_dec_deg("61:23:41")),
        EqPoint(ra_hours=parse_ra_hours("10:49:00"), dec_deg=parse_dec_deg("61:41:29")),
    ]

def _read_points_interactive() -> List[EqPoint]:
    pts: List[EqPoint] = []
    end_index = Constants.POINTS_REQUIRED + Constants.POINT_INDEX_END_OFFSET
    for idx in range(Constants.POINT_INDEX_START, end_index):
        while True:
            prompt = Constants.INPUT_PROMPT_TEMPLATE.format(
                index=idx,
                total=Constants.POINTS_REQUIRED,
            )
            line = input(prompt).strip()
            if not line:
                continue
            try:
                point = _parse_point_line(line)
            except Exception as exc:
                print(Constants.PARSE_ERROR_TEMPLATE.format(error=exc))
                continue
            ra_str, dec_str = _format_point(point)
            print(
                Constants.PARSED_LINE_TEMPLATE.format(
                    index=idx,
                    ra=ra_str,
                    dec=dec_str,
                )
            )
            pts.append(point)
            break
    return pts

def _read_points_stdin() -> List[EqPoint]:
    import sys
    raw_lines = [ln.strip() for ln in sys.stdin.read().splitlines() if ln.strip()]
    if len(raw_lines) < Constants.POINTS_REQUIRED:
        raise InputCountError(
            Constants.STDIN_USAGE_MESSAGE.format(count=Constants.POINTS_REQUIRED)
        )
    pts: List[EqPoint] = []
    for idx, line in enumerate(raw_lines[:Constants.POINTS_REQUIRED], start=Constants.POINT_INDEX_START):
        point = _parse_point_line(line)
        ra_str, dec_str = _format_point(point)
        print(
            Constants.PARSED_LINE_TEMPLATE.format(
                index=idx,
                ra=ra_str,
                dec=dec_str,
            )
        )
        pts.append(point)
    return pts

def radec_to_unitvec(ra_hours: AngleLike, dec_deg: AngleLike) -> Tuple[float, float, float]:
    ra_h = parse_ra_hours(ra_hours)
    dec_d = parse_dec_deg(dec_deg)
    ra = math.radians(ra_h * 15.0)
    dec = math.radians(dec_d)
    x = math.cos(dec) * math.cos(ra)
    y = math.cos(dec) * math.sin(ra)
    z = math.sin(dec)
    return (x, y, z)

def vec_sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])
def norm(a): return math.sqrt(dot(a, a))
def unit(a):
    n = norm(a)
    if n == 0:
        raise ValueError("Zero-length vector")
    return (a[0]/n, a[1]/n, a[2]/n)

def axis_from_three_points(p1: EqPoint, p2: EqPoint, p3: EqPoint) -> Tuple[float, float, float]:
    v1 = radec_to_unitvec(p1.ra_hours, p1.dec_deg)
    v2 = radec_to_unitvec(p2.ra_hours, p2.dec_deg)
    v3 = radec_to_unitvec(p3.ra_hours, p3.dec_deg)

    # plane normal through 3 points in 3D: n = (v2-v1) x (v3-v1)
    n = cross(vec_sub(v2, v1), vec_sub(v3, v1))
    if norm(n) < 1e-10:
        raise ValueError("Points are nearly collinear / RA shift too small -> unstable geometry")
    a = unit(n)

    # choose sign so it points to северное полушарие (z >= 0)
    if a[2] < 0:
        a = (-a[0], -a[1], -a[2])
    return a

def vec_to_radec_hours_deg(v: Tuple[float, float, float]) -> Tuple[float, float]:
    x, y, z = unit(v)
    ra = math.degrees(math.atan2(y, x)) % 360.0
    dec = math.degrees(math.asin(z))
    return (ra / 15.0, dec)

def polar_error_arcmin(axis_vec: Tuple[float, float, float]) -> float:
    # True north celestial pole in equatorial coordinates is (0,0,1)
    pole = (0.0, 0.0, 1.0)
    c = max(-1.0, min(1.0, dot(unit(axis_vec), pole)))
    err_rad = math.acos(c)
    return math.degrees(err_rad) * 60.0


# --- Geometry / uncertainty estimation ---

def _angular_sep_deg(u: Tuple[float, float, float], v: Tuple[float, float, float]) -> float:
    """Great-circle angular separation between unit vectors, in degrees."""
    cuv = max(-1.0, min(1.0, dot(unit(u), unit(v))))
    return math.degrees(math.acos(cuv))


def _points_separations_deg(p1: 'EqPoint', p2: 'EqPoint', p3: 'EqPoint') -> Tuple[float, float, float]:
    v1 = radec_to_unitvec(p1.ra_hours, p1.dec_deg)
    v2 = radec_to_unitvec(p2.ra_hours, p2.dec_deg)
    v3 = radec_to_unitvec(p3.ra_hours, p3.dec_deg)
    return (
        _angular_sep_deg(v1, v2),
        _angular_sep_deg(v2, v3),
        _angular_sep_deg(v1, v3),
    )


def _perturb_point(p: 'EqPoint', sigma_arcsec: float) -> 'EqPoint':
    """Add small Gaussian noise to RA/Dec.

    sigma_arcsec is interpreted as 1-sigma on-sky angular error.
    RA noise is scaled by 1/cos(dec) to keep on-sky distance ~sigma.
    """
    sigma_deg = sigma_arcsec / 3600.0
    dec = float(p.dec_deg)
    # Dec perturbation in degrees
    d_dec = random.gauss(0.0, sigma_deg)
    # RA perturbation in degrees of angle (not hours); scale by cos(dec)
    c = math.cos(math.radians(dec))
    if abs(c) < 1e-6:
        c = 1e-6
    d_ra_deg = random.gauss(0.0, sigma_deg / c)
    ra_h = float(p.ra_hours)
    ra_deg = ra_h * 15.0
    ra_deg_p = (ra_deg + d_ra_deg) % 360.0
    ra_h_p = ra_deg_p / 15.0
    dec_p = max(-90.0, min(90.0, dec + d_dec))
    return EqPoint(ra_hours=ra_h_p, dec_deg=dec_p)


def estimate_delta_uncertainty(
    *,
    p1: 'EqPoint',
    p2: 'EqPoint',
    p3: 'EqPoint',
    lat_deg: float,
    lon_deg: float,
    dt_utc: _dt.datetime,
    sigma_arcsec: float,
    n_mc: int,
) -> Tuple[float, float, float]:
    """Monte-Carlo estimate of 1-sigma uncertainty for ΔAlt, ΔAz, and Total (all in arcmin).

    Uses the current geometry (separations between points) implicitly.
    """
    # Compute nominal axis and nominal deltas
    axis0 = axis_from_three_points(p1, p2, p3)
    ra0, dec0 = vec_to_radec_hours_deg(axis0)
    alt0, az0 = _eq_to_altaz(ra0, dec0, lat_deg, lon_deg, dt_utc)
    d_alt0 = lat_deg - alt0
    d_az0 = _short_angle_deg(0.0, az0)
    v_actual0 = _enu_unit_from_altaz(alt0, az0)
    v_ideal0 = _enu_unit_from_altaz(lat_deg, 0.0)
    total0 = _angle_deg_between(v_actual0, v_ideal0)

    # Samples
    d_alt_samples = []
    d_az_samples = []
    total_samples = []

    for _ in range(max(1, int(n_mc))):
        pp1 = _perturb_point(p1, sigma_arcsec)
        pp2 = _perturb_point(p2, sigma_arcsec)
        pp3 = _perturb_point(p3, sigma_arcsec)

        try:
            ax = axis_from_three_points(pp1, pp2, pp3)
        except Exception:
            # Degenerate perturbed geometry; skip
            continue

        ra, dec = vec_to_radec_hours_deg(ax)
        alt, az = _eq_to_altaz(ra, dec, lat_deg, lon_deg, dt_utc)
        d_alt = lat_deg - alt
        d_az = _short_angle_deg(0.0, az)
        v_actual = _enu_unit_from_altaz(alt, az)
        v_ideal = _enu_unit_from_altaz(lat_deg, 0.0)
        total = _angle_deg_between(v_actual, v_ideal)

        # Store residuals around nominal (degrees)
        d_alt_samples.append((d_alt - d_alt0) * 60.0)
        d_az_samples.append((d_az - d_az0) * 60.0)
        total_samples.append((total - total0) * 60.0)

    def _std_arcmin(xs: List[float]) -> float:
        if len(xs) < 2:
            return float('nan')
        m = sum(xs) / len(xs)
        v = sum((x - m) ** 2 for x in xs) / (len(xs) - 1)
        return math.sqrt(v)

    return (
        _std_arcmin(d_alt_samples),
        _std_arcmin(d_az_samples),
        _std_arcmin(total_samples),
    )


# --- New percentile-based MC uncertainty estimation ---
def _percentile(xs: List[float], q: float) -> float:
    """Return q-quantile for q in [0,1] using linear interpolation."""
    if not xs:
        return float("nan")
    xs_sorted = sorted(xs)
    if q <= 0.0:
        return xs_sorted[0]
    if q >= 1.0:
        return xs_sorted[-1]
    pos = q * (len(xs_sorted) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return xs_sorted[lo]
    frac = pos - lo
    return xs_sorted[lo] * (1 - frac) + xs_sorted[hi] * frac


def estimate_delta_pm_deg(
    *,
    p1: EqPoint,
    p2: EqPoint,
    p3: EqPoint,
    lat_deg: float,
    lon_deg: float,
    dt_utc: _dt.datetime,
    sigma_arcsec: float,
    n_mc: int,
    ci: float,
) -> Tuple[float, float, float]:
    """Half-width (±) in degrees for (ΔAlt, ΔAz, Total) using MC percentile interval.

    Example: ci=0.68 -> half-width of [16%,84%] interval; ci=0.95 -> [2.5%,97.5%].
    """
    d_alt_vals: List[float] = []
    d_az_vals: List[float] = []
    total_vals: List[float] = []

    for _ in range(max(1, int(n_mc))):
        pp1 = _perturb_point(p1, sigma_arcsec)
        pp2 = _perturb_point(p2, sigma_arcsec)
        pp3 = _perturb_point(p3, sigma_arcsec)
        try:
            ax = axis_from_three_points(pp1, pp2, pp3)
        except Exception:
            continue

        ra, dec = vec_to_radec_hours_deg(ax)
        alt, az = _eq_to_altaz(ra, dec, lat_deg, lon_deg, dt_utc)

        d_alt_vals.append(lat_deg - alt)
        d_az_vals.append(_short_angle_deg(0.0, az))

        v_actual = _enu_unit_from_altaz(alt, az)
        v_ideal = _enu_unit_from_altaz(lat_deg, 0.0)
        total_vals.append(_angle_deg_between(v_actual, v_ideal))

    lo = (1.0 - ci) / 2.0
    hi = 1.0 - lo

    def _pm(xs: List[float]) -> float:
        p_lo = _percentile(xs, lo)
        p_hi = _percentile(xs, hi)
        return 0.5 * (p_hi - p_lo)

    return (
        _pm(d_alt_vals),
        _pm(d_az_vals),
        _pm(total_vals),
    )


# --- Helper functions for Alt/Az and adjustment suggestion ---
def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def _julian_date(dt_utc: _dt.datetime) -> float:
    """Julian Date for a timezone-aware UTC datetime."""
    if dt_utc.tzinfo is None:
        raise ValueError("dt_utc must be timezone-aware")
    dt_utc = dt_utc.astimezone(_dt.timezone.utc)
    # Unix epoch to JD: JD = 2440587.5 + seconds/86400
    return 2440587.5 + dt_utc.timestamp() / 86400.0

def _gmst_deg(jd: float) -> float:
    """Greenwich Mean Sidereal Time in degrees [0,360)."""
    T = (jd - 2451545.0) / 36525.0
    theta = (
        280.46061837
        + 360.98564736629 * (jd - 2451545.0)
        + 0.000387933 * (T ** 2)
        - (T ** 3) / 38710000.0
    )
    return theta % 360.0

def _lst_deg(jd: float, lon_deg_east: float) -> float:
    """Local Sidereal Time in degrees [0,360). lon east-positive."""
    return (_gmst_deg(jd) + lon_deg_east) % 360.0

def _eq_to_altaz(ra_hours: float, dec_deg: float, lat_deg: float, lon_deg: float, dt_utc: _dt.datetime) -> Tuple[float, float]:
    """Convert equatorial (RA/Dec) to local Alt/Az.

    Az is degrees east of true north in [0,360).
    Alt is degrees above horizon.
    """
    jd = _julian_date(dt_utc)
    lst = math.radians(_lst_deg(jd, lon_deg))

    ra = math.radians(ra_hours * 15.0)
    dec = math.radians(dec_deg)
    lat = math.radians(lat_deg)

    ha = (lst - ra)  # hour angle, radians

    sin_alt = math.sin(dec) * math.sin(lat) + math.cos(dec) * math.cos(lat) * math.cos(ha)
    alt = math.asin(_clamp(sin_alt, -1.0, 1.0))

    # Azimuth, east of north:
    y = -math.cos(dec) * math.cos(lat) * math.sin(ha)
    x = math.sin(dec) - math.sin(alt) * math.sin(lat)
    az = math.atan2(y, x)
    az_deg = (math.degrees(az) % 360.0)
    alt_deg = math.degrees(alt)
    return alt_deg, az_deg

def _short_angle_deg(target: float, current: float) -> float:
    """Signed shortest rotation (deg) to go from current to target, in (-180,180]."""
    d = (target - current) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


# --- 3D Schematic Plotting Helpers ---
def _enu_unit_from_altaz(alt_deg: float, az_deg: float) -> Tuple[float, float, float]:
    """Unit vector in ENU from Alt/Az.

    Az is degrees east of north.
    Returns (E, N, U).
    """
    alt = math.radians(alt_deg)
    az = math.radians(az_deg)
    ch = math.cos(alt)
    e = ch * math.sin(az)
    n = ch * math.cos(az)
    u = math.sin(alt)
    return (e, n, u)

def _angle_deg_between(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    au = unit(a)
    bu = unit(b)
    c = max(-1.0, min(1.0, dot(au, bu)))
    return math.degrees(math.acos(c))

# --- Curved arrow helpers for adjustment suggestion ---
def _plot_arc_arrow_enu_az(ax, *, sweep_deg: float, radius: float, color: str) -> Tuple[float, float, float]:
    """Draw a curved arrow in the E-N plane (U=0) indicating azimuth adjustment.

    Az is defined east-of-north. Positive sweep means rotate toward East.
    Returns the tip position (x,y,z) for labeling.
    """
    steps = 60
    a0 = 0.0
    a1 = math.radians(sweep_deg)
    angles = [a0 + (a1 - a0) * i / (steps - 1) for i in range(steps)]
    xs = [radius * math.sin(a) for a in angles]  # E
    ys = [radius * math.cos(a) for a in angles]  # N
    zs = [0.0 for _ in angles]
    ax.plot(xs, ys, zs, linewidth=2.5, color=color)

    # Arrow head: tangent direction at the end of the arc
    a = angles[-1]
    x, y, z = xs[-1], ys[-1], 0.0
    tx, ty, tz = math.cos(a), -math.sin(a), 0.0
    tnorm = math.hypot(tx, ty)
    if tnorm > 0:
        tx, ty = tx / tnorm, ty / tnorm
    ax.quiver(x, y, z, tx, ty, tz, length=0.15, normalize=True, color=color)
    return (x, y, z)


def _plot_arc_arrow_enu_alt(ax, *, sweep_deg: float, radius: float, color: str) -> Tuple[float, float, float]:
    """Draw a curved arrow in the N-U plane (E=0) indicating altitude adjustment.

    Positive sweep means rotate from North toward Up.
    Returns the tip position (x,y,z) for labeling.
    """
    steps = 60
    a0 = 0.0
    a1 = math.radians(sweep_deg)
    angles = [a0 + (a1 - a0) * i / (steps - 1) for i in range(steps)]
    xs = [0.0 for _ in angles]
    ys = [radius * math.cos(a) for a in angles]  # N
    zs = [radius * math.sin(a) for a in angles]  # U
    ax.plot(xs, ys, zs, linewidth=2.5, color=color)

    # Arrow head: tangent direction at the end of the arc
    a = angles[-1]
    x, y, z = 0.0, ys[-1], zs[-1]
    tx, ty, tz = 0.0, -math.sin(a), math.cos(a)
    tnorm = math.hypot(ty, tz)
    if tnorm > 0:
        ty, tz = ty / tnorm, tz / tnorm
    ax.quiver(x, y, z, tx, ty, tz, length=0.15, normalize=True, color=color)
    return (x, y, z)

def save_3d_schematic(
    *,
    axis_vec_eq: Tuple[float, float, float],
    lat_deg: float,
    lon_deg: float,
    dt_utc: _dt.datetime,
    out_path: str,
    show: bool = False,
    pm68: Tuple[float, float, float] | None = None,
    pm95: Tuple[float, float, float] | None = None,
) -> str:
    """Save a 3D schematic PNG.

    - Thin dashed line: ideal polar axis (Az=0, Alt=lat)
    - Thick solid line: measured mount polar axis
    - Shows ENU axes and angle annotations: ΔAlt, ΔAz, total error.
    """
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D 
    except Exception as exc:
        raise RuntimeError("matplotlib is required for --plot") from exc

    ra_h, dec_d = vec_to_radec_hours_deg(axis_vec_eq)
    alt_axis, az_axis = _eq_to_altaz(ra_h, dec_d, lat_deg, lon_deg, dt_utc)

    alt_ideal = lat_deg
    az_ideal = 0.0

    d_alt = alt_ideal - alt_axis
    d_az = _short_angle_deg(az_ideal, az_axis)

    v_actual = _enu_unit_from_altaz(alt_axis, az_axis)
    v_ideal = _enu_unit_from_altaz(alt_ideal, az_ideal)

    total_err = _angle_deg_between(v_actual, v_ideal)

    # Build figure
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    # ENU axes
    axis_len = 1.0
    ax.plot([0, axis_len], [0, 0], [0, 0], linewidth=1)  # E
    ax.plot([0, 0], [0, axis_len], [0, 0], linewidth=1)  # N
    ax.plot([0, 0], [0, 0], [0, axis_len], linewidth=1)  # U
    ax.text(axis_len, 0, 0, "E", fontsize=10)
    ax.text(0, axis_len, 0, "N", fontsize=10)
    ax.text(0, 0, axis_len, "U", fontsize=10)

    # Ideal (thin dashed)
    ax.plot(
        [0, v_ideal[0]],
        [0, v_ideal[1]],
        [0, v_ideal[2]],
        linestyle="--",
        linewidth=1,
    )

    # Actual (thick)
    ax.plot(
        [0, v_actual[0]],
        [0, v_actual[1]],
        [0, v_actual[2]],
        linewidth=4,
    )

    # Angle annotations
    # Put ΔAlt text in the N-U plane (E=0): that's the rotation plane for altitude adjustment.
    # Put ΔAz text in the E-N plane (U=0): that's the rotation plane for azimuth adjustment.
    pm_total = pm95[2] if pm95 is not None else (pm68[2] if pm68 is not None else None)
    pm_total_txt = f" ±{pm_total:.3f}°" if pm_total is not None else ""
    total_txt = f"Total = {total_err:.3f}° ({total_err*60:.1f}′){pm_total_txt}"
    ax.text(0.02, 0.02, 0.02, total_txt, fontsize=10)

    # ΔAz label position: along horizontal projection of the actual axis (E-N plane)
    proj_h = (v_actual[0], v_actual[1], 0.0)
    ph_n = math.hypot(proj_h[0], proj_h[1])
    if ph_n < 1e-9:
        # fallback: point to North
        az_pos = (0.0, 0.45, 0.0)
    else:
        az_pos = (0.45 * proj_h[0] / ph_n, 0.45 * proj_h[1] / ph_n, 0.0)
    pm_az = pm95[1] if pm95 is not None else (pm68[1] if pm68 is not None else None)
    pm_az_txt = f" ±{pm_az:.3f}°" if pm_az is not None else ""
    az_txt = f"ΔAz = {d_az:+.3f}° ({d_az*60:+.1f}′){pm_az_txt}"
    ax.text(az_pos[0], az_pos[1], az_pos[2], az_txt, fontsize=10, color=Constants.DIRECTION_AZ_COLOR)

    # ΔAlt label position: along projection of the actual axis into the N-U plane (E=0)
    proj_v = (0.0, v_actual[1], v_actual[2])
    pv_n = math.hypot(proj_v[1], proj_v[2])
    if pv_n < 1e-9:
        # fallback: point Up
        alt_pos = (0.0, 0.0, 0.55)
    else:
        alt_pos = (0.0, 0.55 * proj_v[1] / pv_n, 0.55 * proj_v[2] / pv_n)
    pm_alt = pm95[0] if pm95 is not None else (pm68[0] if pm68 is not None else None)
    pm_alt_txt = f" ±{pm_alt:.3f}°" if pm_alt is not None else ""
    alt_txt = f"ΔAlt = {d_alt:+.3f}° ({d_alt*60:+.1f}′){pm_alt_txt}"
    ax.text(alt_pos[0], alt_pos[1], alt_pos[2], alt_txt, fontsize=10, color=Constants.DIRECTION_ALT_COLOR)

    # Suggested adjustment directions as curved arrows (more intuitive than axis-aligned vectors)
    radius = Constants.DIRECTION_ARC_RADIUS
    sweep = Constants.DIRECTION_ARC_SWEEP_DEG

    if abs(d_alt) > Constants.DIRECTION_EPS_DEG:
        alt_label = (
            Constants.DIRECTION_ALT_RAISE_LABEL
            if d_alt > 0
            else Constants.DIRECTION_ALT_LOWER_LABEL
        )
        tip = _plot_arc_arrow_enu_alt(
            ax,
            sweep_deg=(sweep if d_alt > 0 else -sweep),
            radius=radius,
            color=Constants.DIRECTION_ALT_COLOR,
        )
        ax.text(
            tip[0] * Constants.DIRECTION_LABEL_SCALE,
            tip[1] * Constants.DIRECTION_LABEL_SCALE,
            tip[2] * Constants.DIRECTION_LABEL_SCALE,
            alt_label,
            fontsize=10,
            color=Constants.DIRECTION_ALT_COLOR,
        )

    if abs(d_az) > Constants.DIRECTION_EPS_DEG:
        az_label = (
            Constants.DIRECTION_AZ_EAST_LABEL
            if d_az > 0
            else Constants.DIRECTION_AZ_WEST_LABEL
        )
        tip = _plot_arc_arrow_enu_az(
            ax,
            sweep_deg=(sweep if d_az > 0 else -sweep),
            radius=radius,
            color=Constants.DIRECTION_AZ_COLOR,
        )
        ax.text(
            tip[0] * Constants.DIRECTION_LABEL_SCALE,
            tip[1] * Constants.DIRECTION_LABEL_SCALE,
            tip[2] * Constants.DIRECTION_LABEL_SCALE,
            az_label,
            fontsize=10,
            color=Constants.DIRECTION_AZ_COLOR,
        )

    # Formatting: equal-ish scale and view
    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_zlabel("Up")
    ax.set_xlim(-1.05, 1.05)
    ax.set_ylim(-1.05, 1.05)
    ax.set_zlim(-0.05, 1.05)
    ax.view_init(elev=20, azim=-55)

    # Title with time/location
    ax.set_title(
        f"Polar axis (local ENU) | lat={lat_deg:.4f} lon={lon_deg:.4f} | {dt_utc.isoformat()} UTC"
    )

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)
    return out_path

def suggest_adjustments(
    axis_vec: Tuple[float, float, float],
    lat_deg: float,
    lon_deg: float,
    dt_utc: _dt.datetime,
    pm68: Tuple[float, float, float] | None = None,
    pm95: Tuple[float, float, float] | None = None,
) -> str:
    """Return human-readable suggestions for altitude/azimuth knobs.

    Notes:
    - This uses current (or provided) time to compute Alt/Az.
    - Ideal EQ axis: az=0 (true north), alt=lat (north hemisphere).
    """
    ra_h, dec_d = vec_to_radec_hours_deg(axis_vec)
    alt_axis, az_axis = _eq_to_altaz(ra_h, dec_d, lat_deg, lon_deg, dt_utc)

    alt_ideal = lat_deg
    az_ideal = 0.0

    d_alt = alt_ideal - alt_axis  # + => need raise
    d_az = _short_angle_deg(az_ideal, az_axis)  # + => rotate east, - => rotate west

    pm_alt = pm95[0] if pm95 is not None else (pm68[0] if pm68 is not None else None)
    pm_az = pm95[1] if pm95 is not None else (pm68[1] if pm68 is not None else None)
    pm_total = pm95[2] if pm95 is not None else (pm68[2] if pm68 is not None else None)

    pm_alt_txt = f" ±{pm_alt:.4f}°" if pm_alt is not None else ""
    pm_az_txt = f" ±{pm_az:.4f}°" if pm_az is not None else ""

    lines = []
    lines.append(f"Axis in local frame at {dt_utc.isoformat()} (UTC):")
    lines.append(
        f"  Alt(axis) = {alt_axis:.3f}° ; ideal Alt = {alt_ideal:.3f}° -> ΔAlt = {d_alt:+.3f}°{pm_alt_txt}"
    )
    lines.append(
        f"  Az(axis)  = {az_axis:.3f}° (E of N) ; ideal Az = {az_ideal:.3f}° -> ΔAz  = {d_az:+.3f}°{pm_az_txt}"
    )
    if pm_total is not None:
        lines.append(f"  Total error ±{pm_total:.4f}° (from MC, {'95%' if pm95 is not None else '68%'})")

    # Convert to arcmin for knob feel
    d_alt_arcmin = d_alt * 60.0
    d_az_arcmin = d_az * 60.0

    lines.append("")

    if abs(d_alt_arcmin) < 1.0 and abs(d_az_arcmin) < 1.0:
        lines.append("Suggestion: already within ~1 arcmin in both axes.")
        return "\n".join(lines)

    if d_alt_arcmin > 0:
        lines.append(f"Suggestion (ALT): raise the polar axis by about {abs(d_alt_arcmin):.1f} arcmin.")
    elif d_alt_arcmin < 0:
        lines.append(f"Suggestion (ALT): lower the polar axis by about {abs(d_alt_arcmin):.1f} arcmin.")

    if d_az_arcmin > 0:
        lines.append(f"Suggestion (AZ): move the polar axis EAST by about {abs(d_az_arcmin):.1f} arcmin.")
    elif d_az_arcmin < 0:
        lines.append(f"Suggestion (AZ): move the polar axis WEST by about {abs(d_az_arcmin):.1f} arcmin.")

    lines.append("")
    lines.append("Note: direction (E/W, raise/lower) assumes azimuth is measured east-of-true-north and your knobs move the mount head accordingly.")
    return "\n".join(lines)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Estimate polar misalignment from 3 RA-only plate-solve points read from stdin.")
    parser.add_argument("--lat", type=float, default=43.2383, help="Observer latitude in degrees (default: Almaty)")
    parser.add_argument("--lon", type=float, default=76.9450, help="Observer longitude east-positive degrees (default: Almaty)")
    parser.add_argument("--tz", type=str, default="Asia/Almaty", help="Local timezone name for --time parsing")
    parser.add_argument(
        "--time",
        type=str,
        default=None,
        help="Local time of the measurement, ISO-like (e.g. 2026-01-25T22:10:00). If omitted, uses now().",
    )
    parser.add_argument(
        "--use-default-points",
        action="store_true",
        default=True,
        help="Use built-in default test points instead of reading points from stdin.",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        default=True,
        help="Save a 3D schematic (ENU axes + ideal vs actual polar axis) to a PNG file.",
    )
    parser.add_argument(
        "--plot-show",
        action="store_true",
        default=True,
        help=Constants.PLOT_SHOW_HELP,
    )
    parser.add_argument(
        "--sigma-arcsec",
        type=float,
        default=10.0,
        help="Assumed 1-sigma plate-solve pointing noise per frame (arcsec). Used to estimate ΔAlt/ΔAz uncertainty.",
    )
    parser.add_argument(
        "--mc",
        type=int,
        default=2000,
        help="Monte-Carlo samples for uncertainty estimation (default: 2000).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for Monte-Carlo (optional).",
    )
    parser.add_argument(
        "--plot-out",
        type=str,
        default="polar_alignment_{now}.png",
        help="Output PNG path for --plot (default: polar_alignment_{now}.png)",
    )
    args = parser.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    # Read 3 points from stdin. Accept either full "Syncing to RA (...) DEC (...)" lines
    # or simple "RA Dec" lines; blank lines are ignored.
    import sys

    try:
        if args.use_default_points:
            pts = _default_points()
        elif sys.stdin.isatty():
            pts = _read_points_interactive()
        else:
            pts = _read_points_stdin()
    
    except (InputCountError, InputLineParseError) as exc:
        raise SystemExit(str(exc)) from exc

    t1, t2, t3 = pts

    sep12, sep23, sep13 = _points_separations_deg(t1, t2, t3)
    min_sep = min(sep12, sep23)
    print(f"Point separations (deg): sep12={sep12:.3f}, sep23={sep23:.3f}, sep13={sep13:.3f}")
    if min_sep < 5.0:
        print("WARNING: separations are small (<5°). ΔAlt/ΔAz estimates will be noisy; use larger RA shifts (e.g., 15–30°).")
    print()

    axis = axis_from_three_points(t1, t2, t3)
    ra_ax_h, dec_ax_deg = vec_to_radec_hours_deg(axis)
    err_arcmin = polar_error_arcmin(axis)

    print("Mount RA-axis (approx, no-time):")
    print(f"  RA  = {ra_ax_h:.6f} h")
    print(f"  Dec = {dec_ax_deg:.6f} deg")
    print(f"Polar misalignment magnitude ≈ {err_arcmin:.2f} arcmin")
    print()

    # For actionable knob directions, we need a time+location to express the axis in local Alt/Az.
    tz = ZoneInfo(args.tz)
    if args.time is None:
        dt_local = _dt.datetime.now(tz=tz)
    else:
        # Accept both 'YYYY-MM-DDTHH:MM:SS' and 'YYYY-MM-DD HH:MM:SS'
        s = args.time.replace(" ", "T")
        dt_local = _dt.datetime.fromisoformat(s)
        if dt_local.tzinfo is None:
            dt_local = dt_local.replace(tzinfo=tz)
        else:
            dt_local = dt_local.astimezone(tz)

    dt_utc = dt_local.astimezone(_dt.timezone.utc)
    pm68_dalt, pm68_daz, pm68_total = estimate_delta_pm_deg(
        p1=t1,
        p2=t2,
        p3=t3,
        lat_deg=args.lat,
        lon_deg=args.lon,
        dt_utc=dt_utc,
        sigma_arcsec=args.sigma_arcsec,
        n_mc=args.mc,
        ci=0.68,
    )
    pm95_dalt, pm95_daz, pm95_total = estimate_delta_pm_deg(
        p1=t1,
        p2=t2,
        p3=t3,
        lat_deg=args.lat,
        lon_deg=args.lon,
        dt_utc=dt_utc,
        sigma_arcsec=args.sigma_arcsec,
        n_mc=args.mc,
        ci=0.95,
    )
    print(
        f"Estimated ± (deg) from MC with σ_frame={args.sigma_arcsec:.1f}\" and N={args.mc}:\n "
        f"  68%: ΔAlt ±{pm68_dalt:.4f}°, ΔAz ±{pm68_daz:.4f}°, Total ±{pm68_total:.4f}°\n"
        f"  95%: ΔAlt ±{pm95_dalt:.4f}°, ΔAz ±{pm95_daz:.4f}°, Total ±{pm95_total:.4f}°"
    )
    print()
    print(suggest_adjustments(
        axis,
        lat_deg=args.lat,
        lon_deg=args.lon,
        dt_utc=dt_utc,
        pm68=(pm68_dalt, pm68_daz, pm68_total),
        pm95=(pm95_dalt, pm95_daz, pm95_total),
    ))

    if args.plot:
        out = save_3d_schematic(
            axis_vec_eq=axis,
            lat_deg=args.lat,
            lon_deg=args.lon,
            dt_utc=dt_utc,
            out_path=args.plot_out.format(now=_dt.datetime.now().strftime("%Y%m%d_%H%M%S")),
            pm68=(pm68_dalt, pm68_daz, pm68_total),
            pm95=(pm95_dalt, pm95_daz, pm95_total),
            show=args.plot_show,
        )
        print()
        print(f"Saved 3D schematic to: {out}")
