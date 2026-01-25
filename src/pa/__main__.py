import math
import re
import argparse
import datetime as _dt
from zoneinfo import ZoneInfo
from dataclasses import dataclass
from typing import List, Tuple, Union

AngleLike = Union[float, int, str]

class Constants:
    POINTS_REQUIRED = 3
    POINT_INDEX_START = 1
    POINT_INDEX_END_OFFSET = 1
    COORD_PRINT_DECIMALS = 6
    INPUT_PROMPT_TEMPLATE = "Line {index}/{total}: "
    PARSED_LINE_TEMPLATE = "Parsed line {index}: RA={ra} h, Dec={dec} deg"
    PARSE_ERROR_TEMPLATE = "Parse error: {error}"
    STDIN_USAGE_MESSAGE = (
        "Need {count} non-empty lines on stdin. Example:\n"
        "  Syncing to RA (10h 44m 23s) DEC ( 33° 04' 58\")\n"
        "  Syncing to RA (10h 33m 12s) DEC ( 33° 05' 01\")\n"
        "  Syncing to RA (10h 44m 00s) DEC ( 33° 04' 42\")\n"
    )

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

def suggest_adjustments(axis_vec: Tuple[float, float, float], lat_deg: float, lon_deg: float, dt_utc: _dt.datetime) -> str:
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

    lines = []
    lines.append(f"Axis in local frame at {dt_utc.isoformat()} (UTC):")
    lines.append(f"  Alt(axis) = {alt_axis:.3f}° ; ideal Alt = {alt_ideal:.3f}° -> ΔAlt = {d_alt:+.3f}°")
    lines.append(f"  Az(axis)  = {az_axis:.3f}° (E of N) ; ideal Az = {az_ideal:.3f}° -> ΔAz  = {d_az:+.3f}°")

    # Convert to arcmin for knob feel
    d_alt_arcmin = d_alt * 60.0
    d_az_arcmin = d_az * 60.0

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
    args = parser.parse_args()

    # Read 3 points from stdin. Accept either full "Syncing to RA (...) DEC (...)" lines
    # or simple "RA Dec" lines; blank lines are ignored.
    import sys

    try:
        if sys.stdin.isatty():
            pts = _read_points_interactive()
        else:
            pts = _read_points_stdin()
    except (InputCountError, InputLineParseError) as exc:
        raise SystemExit(str(exc)) from exc

    t1, t2, t3 = pts

    axis = axis_from_three_points(t1, t2, t3)
    ra_ax_h, dec_ax_deg = vec_to_radec_hours_deg(axis)
    err_arcmin = polar_error_arcmin(axis)

    print("Mount RA-axis (approx, no-time):")
    print(f"  RA  = {ra_ax_h:.6f} h")
    print(f"  Dec = {dec_ax_deg:.6f} deg")
    print(f"Polar misalignment magnitude ≈ {err_arcmin:.2f} arcmin")

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
    print()
    print(suggest_adjustments(axis, lat_deg=args.lat, lon_deg=args.lon, dt_utc=dt_utc))
