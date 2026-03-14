"""
polar_align_lib.py

Single-file library for 3-solve polar alignment estimation and live mechanical adjustment.

What it does
------------
1. Accepts three plate-solved sky positions (RA/Dec + timestamp).
2. Estimates the actual RA axis direction from the three points.
3. Computes polar misalignment.
4. Converts misalignment into base Alt/Az correction.
5. Computes the sky coordinate where the telescope should point after a successful
   mechanical base adjustment, assuming motor RA/DEC angles are not changed.
6. Provides a simple state machine for workflow control.

Coordinate conventions
----------------------
- RA is in hours by default in input dataclasses, internal vector math uses radians.
- Dec, latitude, longitude are in degrees in input dataclasses, internal math uses radians.
- ENU basis: [East, North, Up].
- Azimuth is measured east of north: North=0 deg, East=90 deg.
- Target polar axis is the NORTH celestial pole by default.

Assumptions
-----------
- The three solves are taken with DEC fixed and RA changed between frames.
- All solved coordinates are in one consistent frame/epoch.
- For best results use apparent/of-date coordinates consistently.
- Refraction, cone error, flexure, non-perpendicularity and encoder/index errors
  are not modeled in the minimal solver.

Usage sketch
------------
    from datetime import datetime, timezone
    from polar_align_lib import (
        PlateSolve, ObserverSite, PolarAlignmentSession,
        AlignmentState
    )

    site = ObserverSite(latitude_deg=43.238949, longitude_deg=76.889709)
    session = PolarAlignmentSession(site=site)

    session.add_solve(PlateSolve(ra_hours=5.2, dec_deg=20.1, timestamp=datetime.now(timezone.utc)))
    session.add_solve(PlateSolve(ra_hours=5.9, dec_deg=20.2, timestamp=datetime.now(timezone.utc)))
    session.add_solve(PlateSolve(ra_hours=6.6, dec_deg=20.1, timestamp=datetime.now(timezone.utc)))

    result = session.solve_alignment()
    print(result.delta_alt_arcmin, result.delta_az_arcmin)
    print(result.target_ra_hours, result.target_dec_deg)

    # During live adjustment:
    verify = session.verify_against_target(
        PlateSolve(ra_hours=5.25, dec_deg=20.0, timestamp=datetime.now(timezone.utc))
    )
    print(verify.remaining_error_arcmin)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum, auto
from math import acos, asin, atan2, cos, degrees, pi, radians, sin, sqrt
from typing import Iterable, List, Optional, Sequence, Tuple


# =========================
# Small numeric helpers
# =========================

EPS = 1e-15
TWOPI = 2.0 * pi
SIDEREAL_TO_SOLAR = 1.00273790935


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return sqrt(self.dot(self))

    def normalize(self) -> "Vec3":
        n = self.norm()
        if n < EPS:
            raise ValueError("Cannot normalize near-zero vector")
        return self * (1.0 / n)

    def as_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


# =========================
# Domain models
# =========================


@dataclass(frozen=True)
class ObserverSite:
    latitude_deg: float
    longitude_deg: float  # East positive


@dataclass(frozen=True)
class PlateSolve:
    ra_hours: float
    dec_deg: float
    timestamp: datetime
    label: str = ""

    def __post_init__(self) -> None:
        if self.timestamp.tzinfo is None:
            raise ValueError("timestamp must be timezone-aware")


@dataclass(frozen=True)
class AltAzCoord:
    alt_deg: float
    az_deg: float


@dataclass(frozen=True)
class EquatorialCoord:
    ra_hours: float
    dec_deg: float


@dataclass(frozen=True)
class AxisEstimate:
    axis_eq: Vec3
    axis_enu: Vec3
    pole_eq: Vec3
    pole_enu: Vec3
    angular_error_deg: float
    angular_error_arcmin: float
    delta_alt_deg: float
    delta_alt_arcmin: float
    delta_az_deg: float
    delta_az_arcmin: float
    fit_residual_deg: float


@dataclass(frozen=True)
class TargetPoint:
    target_eq_vec: Vec3
    target_ra_hours: float
    target_dec_deg: float
    target_alt_deg: float
    target_az_deg: float


@dataclass(frozen=True)
class VerificationResult:
    measured_ra_hours: float
    measured_dec_deg: float
    remaining_error_deg: float
    remaining_error_arcmin: float
    delta_alt_deg: float
    delta_az_deg: float


@dataclass(frozen=True)
class AlignmentResult:
    axis: AxisEstimate
    target: TargetPoint
    current_before_adjust: EquatorialCoord
    current_before_adjust_altaz: AltAzCoord

    @property
    def delta_alt_arcmin(self) -> float:
        return self.axis.delta_alt_arcmin

    @property
    def delta_az_arcmin(self) -> float:
        return self.axis.delta_az_arcmin

    @property
    def target_ra_hours(self) -> float:
        return self.target.target_ra_hours

    @property
    def target_dec_deg(self) -> float:
        return self.target.target_dec_deg


class AlignmentState(Enum):
    EMPTY = auto()
    COLLECTING_SOLVES = auto()
    READY_TO_SOLVE = auto()
    SOLVED = auto()
    ADJUSTING = auto()
    VERIFIED = auto()


@dataclass
class SessionLog:
    messages: List[str] = field(default_factory=list)

    def add(self, msg: str) -> None:
        self.messages.append(msg)


# =========================
# Time and astronomy helpers
# =========================


def julian_date(dt: datetime) -> float:
    """UTC datetime -> Julian Date."""
    dt_utc = dt.astimezone(timezone.utc)
    year = dt_utc.year
    month = dt_utc.month
    day = dt_utc.day
    hour = dt_utc.hour
    minute = dt_utc.minute
    second = dt_utc.second + dt_utc.microsecond / 1_000_000.0

    if month <= 2:
        year -= 1
        month += 12

    a = year // 100
    b = 2 - a + (a // 4)

    day_fraction = (hour + minute / 60.0 + second / 3600.0) / 24.0
    jd = (
        int(365.25 * (year + 4716))
        + int(30.6001 * (month + 1))
        + day
        + day_fraction
        + b
        - 1524.5
    )
    return jd


def gmst_rad(dt: datetime) -> float:
    """Greenwich Mean Sidereal Time in radians."""
    jd = julian_date(dt)
    t = (jd - 2451545.0) / 36525.0
    gmst_deg = (
        280.46061837
        + 360.98564736629 * (jd - 2451545.0)
        + 0.000387933 * t * t
        - (t * t * t) / 38710000.0
    )
    return radians(gmst_deg % 360.0)


def lst_rad(dt: datetime, longitude_deg: float) -> float:
    return (gmst_rad(dt) + radians(longitude_deg)) % TWOPI


# =========================
# Coordinate conversions
# =========================


def ra_hours_to_rad(ra_hours: float) -> float:
    return (ra_hours * pi / 12.0) % TWOPI


def rad_to_ra_hours(rad_angle: float) -> float:
    return ((rad_angle % TWOPI) * 12.0 / pi) % 24.0


def radec_to_vec(ra_hours: float, dec_deg: float) -> Vec3:
    a = ra_hours_to_rad(ra_hours)
    d = radians(dec_deg)
    return Vec3(cos(d) * cos(a), cos(d) * sin(a), sin(d)).normalize()


def vec_to_radec(v: Vec3) -> EquatorialCoord:
    vn = v.normalize()
    ra = atan2(vn.y, vn.x) % TWOPI
    dec = asin(clamp(vn.z, -1.0, 1.0))
    return EquatorialCoord(ra_hours=rad_to_ra_hours(ra), dec_deg=degrees(dec))


def radec_to_altaz(ra_hours: float, dec_deg: float, dt: datetime, site: ObserverSite) -> AltAzCoord:
    phi = radians(site.latitude_deg)
    a = ra_hours_to_rad(ra_hours)
    d = radians(dec_deg)
    h_angle = (lst_rad(dt, site.longitude_deg) - a + pi) % TWOPI - pi

    sin_alt = sin(phi) * sin(d) + cos(phi) * cos(d) * cos(h_angle)
    alt = asin(clamp(sin_alt, -1.0, 1.0))

    y = -cos(d) * sin(h_angle)
    x = sin(d) * cos(phi) - cos(d) * sin(phi) * cos(h_angle)
    az = atan2(y, x) % TWOPI
    return AltAzCoord(alt_deg=degrees(alt), az_deg=degrees(az))


def altaz_to_enu(alt_deg: float, az_deg: float) -> Vec3:
    alt = radians(alt_deg)
    az = radians(az_deg)
    return Vec3(
        cos(alt) * sin(az),
        cos(alt) * cos(az),
        sin(alt),
    ).normalize()


def radec_to_enu(ra_hours: float, dec_deg: float, dt: datetime, site: ObserverSite) -> Vec3:
    aa = radec_to_altaz(ra_hours, dec_deg, dt, site)
    return altaz_to_enu(aa.alt_deg, aa.az_deg)


def eq_vec_to_enu(v_eq: Vec3, dt: datetime, site: ObserverSite) -> Vec3:
    """Rotate equatorial vector to local ENU using hour angle basis."""
    eq = vec_to_radec(v_eq)
    return radec_to_enu(eq.ra_hours, eq.dec_deg, dt, site)


def enu_vec_to_altaz(v_enu: Vec3) -> AltAzCoord:
    vn = v_enu.normalize()
    alt = asin(clamp(vn.z, -1.0, 1.0))
    az = atan2(vn.x, vn.y) % TWOPI
    return AltAzCoord(alt_deg=degrees(alt), az_deg=degrees(az))


def north_celestial_pole_eq() -> Vec3:
    return Vec3(0.0, 0.0, 1.0)


def north_celestial_pole_enu(site: ObserverSite) -> Vec3:
    phi = radians(site.latitude_deg)
    return Vec3(0.0, cos(phi), sin(phi)).normalize()


# =========================
# Alignment math
# =========================


def angular_distance_deg(a: Vec3, b: Vec3) -> float:
    an = a.normalize()
    bn = b.normalize()
    return degrees(acos(clamp(an.dot(bn), -1.0, 1.0)))


def estimate_ra_axis_from_three_solves(solves: Sequence[PlateSolve]) -> Vec3:
    if len(solves) != 3:
        raise ValueError("Exactly three solves are required")

    s1 = radec_to_vec(solves[0].ra_hours, solves[0].dec_deg)
    s2 = radec_to_vec(solves[1].ra_hours, solves[1].dec_deg)
    s3 = radec_to_vec(solves[2].ra_hours, solves[2].dec_deg)

    raw = (s2 - s1).cross(s3 - s1)
    if raw.norm() < EPS:
        raise ValueError("The three solves are degenerate; cannot estimate RA axis")

    u = raw.normalize()
    pole = north_celestial_pole_eq()
    if u.dot(pole) < 0.0:
        u = u * -1.0
    return u


def estimate_cone_fit_residual_deg(axis_eq: Vec3, solves: Sequence[PlateSolve]) -> float:
    """Residual: spread of angular distances from solves to the estimated axis."""
    angles = []
    for s in solves:
        v = radec_to_vec(s.ra_hours, s.dec_deg)
        ang = acos(clamp(axis_eq.dot(v), -1.0, 1.0))
        angles.append(ang)
    mean = sum(angles) / len(angles)
    rms = sqrt(sum((a - mean) ** 2 for a in angles) / len(angles))
    return degrees(rms)


def axis_error_to_altaz(axis_enu: Vec3, site: ObserverSite) -> Tuple[float, float]:
    """Return (delta_alt_deg, delta_az_deg).

    delta_alt_deg > 0 => raise polar axis altitude
    delta_az_deg  > 0 => increase azimuth east-of-north in mathematical convention
    Mechanical knob direction depends on mount hardware.
    """
    u = axis_enu.normalize()
    az_u = atan2(u.x, u.y)
    alt_u = asin(clamp(u.z, -1.0, 1.0))

    target_az = 0.0
    target_alt = radians(site.latitude_deg)

    delta_az = (target_az - az_u + pi) % TWOPI - pi
    delta_alt = target_alt - alt_u
    return degrees(delta_alt), degrees(delta_az)


def rodrigues_rotate(v: Vec3, axis: Vec3, angle_rad: float) -> Vec3:
    k = axis.normalize()
    return (
        v * cos(angle_rad)
        + k.cross(v) * sin(angle_rad)
        + k * (k.dot(v) * (1.0 - cos(angle_rad)))
    ).normalize()


def rotation_to_move_axis(current_axis_eq: Vec3, target_axis_eq: Vec3) -> Tuple[Vec3, float]:
    u = current_axis_eq.normalize()
    p = target_axis_eq.normalize()
    cross = u.cross(p)
    cross_norm = cross.norm()
    dot = clamp(u.dot(p), -1.0, 1.0)

    if cross_norm < EPS:
        # Already aligned or anti-parallel. Anti-parallel is non-physical here for north pole case,
        # but handle cleanly.
        if dot > 0:
            return Vec3(1.0, 0.0, 0.0), 0.0
        # 180 deg fallback axis orthogonal to u.
        ref = Vec3(1.0, 0.0, 0.0)
        if abs(u.dot(ref)) > 0.9:
            ref = Vec3(0.0, 1.0, 0.0)
        axis = u.cross(ref).normalize()
        return axis, pi

    axis = cross * (1.0 / cross_norm)
    angle = atan2(cross_norm, dot)
    return axis, angle


def compute_target_after_polar_fix(
    current_point_eq: Vec3,
    current_axis_eq: Vec3,
    target_axis_eq: Optional[Vec3] = None,
) -> Vec3:
    target_axis = target_axis_eq or north_celestial_pole_eq()
    rot_axis, rot_angle = rotation_to_move_axis(current_axis_eq, target_axis)
    return rodrigues_rotate(current_point_eq.normalize(), rot_axis, rot_angle)


def live_remaining_error_deg(measured_eq: Vec3, target_eq: Vec3) -> float:
    return angular_distance_deg(measured_eq, target_eq)


# =========================
# Session / state machine
# =========================


@dataclass
class PolarAlignmentSession:
    site: ObserverSite
    solves: List[PlateSolve] = field(default_factory=list)
    state: AlignmentState = AlignmentState.EMPTY
    result: Optional[AlignmentResult] = None
    log: SessionLog = field(default_factory=SessionLog)

    def reset(self) -> None:
        self.solves.clear()
        self.state = AlignmentState.EMPTY
        self.result = None
        self.log.add("Session reset")

    def add_solve(self, solve: PlateSolve) -> None:
        if self.state in (AlignmentState.SOLVED, AlignmentState.ADJUSTING, AlignmentState.VERIFIED):
            self.log.add("Adding a new solve resets previous solution")
            self.result = None
            self.state = AlignmentState.EMPTY
            self.solves.clear()

        self.solves.append(solve)
        if len(self.solves) == 0:
            self.state = AlignmentState.EMPTY
        elif len(self.solves) < 3:
            self.state = AlignmentState.COLLECTING_SOLVES
        elif len(self.solves) == 3:
            self.state = AlignmentState.READY_TO_SOLVE
        else:
            raise ValueError("This workflow accepts exactly three calibration solves")
        self.log.add(f"Added solve #{len(self.solves)} ({solve.label or 'unnamed'})")

    def solve_alignment(self, use_solve_index_for_target: int = 2) -> AlignmentResult:
        if len(self.solves) != 3:
            raise ValueError("Need exactly three solves before solving alignment")

        axis_eq = estimate_ra_axis_from_three_solves(self.solves)
        residual_deg = estimate_cone_fit_residual_deg(axis_eq, self.solves)

        # Use timestamp of the target/current frame, because ENU and Alt/Az are time-dependent.
        current = self.solves[use_solve_index_for_target]
        axis_enu = eq_vec_to_enu(axis_eq, current.timestamp, self.site)
        pole_eq = north_celestial_pole_eq()
        pole_enu = north_celestial_pole_enu(self.site)

        error_deg = angular_distance_deg(axis_eq, pole_eq)
        delta_alt_deg, delta_az_deg = axis_error_to_altaz(axis_enu, self.site)

        axis = AxisEstimate(
            axis_eq=axis_eq,
            axis_enu=axis_enu,
            pole_eq=pole_eq,
            pole_enu=pole_enu,
            angular_error_deg=error_deg,
            angular_error_arcmin=error_deg * 60.0,
            delta_alt_deg=delta_alt_deg,
            delta_alt_arcmin=delta_alt_deg * 60.0,
            delta_az_deg=delta_az_deg,
            delta_az_arcmin=delta_az_deg * 60.0,
            fit_residual_deg=residual_deg,
        )

        current_vec = radec_to_vec(current.ra_hours, current.dec_deg)
        current_altaz = radec_to_altaz(current.ra_hours, current.dec_deg, current.timestamp, self.site)

        target_vec = compute_target_after_polar_fix(current_vec, axis_eq, pole_eq)
        target_eq = vec_to_radec(target_vec)
        target_altaz = radec_to_altaz(target_eq.ra_hours, target_eq.dec_deg, current.timestamp, self.site)

        target = TargetPoint(
            target_eq_vec=target_vec,
            target_ra_hours=target_eq.ra_hours,
            target_dec_deg=target_eq.dec_deg,
            target_alt_deg=target_altaz.alt_deg,
            target_az_deg=target_altaz.az_deg,
        )

        self.result = AlignmentResult(
            axis=axis,
            target=target,
            current_before_adjust=EquatorialCoord(current.ra_hours, current.dec_deg),
            current_before_adjust_altaz=current_altaz,
        )
        self.state = AlignmentState.SOLVED
        self.log.add("Alignment solved from 3 plate solves")
        self.log.add(
            f"Polar error={axis.angular_error_arcmin:.2f} arcmin, "
            f"delta_alt={axis.delta_alt_arcmin:.2f} arcmin, delta_az={axis.delta_az_arcmin:.2f} arcmin"
        )
        self.log.add(
            f"Target after successful base adjustment: RA={target.target_ra_hours:.6f}h Dec={target.target_dec_deg:.6f}deg"
        )
        return self.result

    def begin_adjustment(self) -> None:
        if self.result is None:
            raise ValueError("No alignment result available")
        self.state = AlignmentState.ADJUSTING
        self.log.add("Entered live adjustment mode")

    def verify_against_target(self, solve: PlateSolve) -> VerificationResult:
        if self.result is None:
            raise ValueError("No alignment result available")

        measured_vec = radec_to_vec(solve.ra_hours, solve.dec_deg)
        target_vec = self.result.target.target_eq_vec
        rem_err_deg = live_remaining_error_deg(measured_vec, target_vec)

        # Delta in local Alt/Az at the time of verification.
        measured_altaz = radec_to_altaz(solve.ra_hours, solve.dec_deg, solve.timestamp, self.site)
        target_altaz = radec_to_altaz(
            self.result.target.target_ra_hours,
            self.result.target.target_dec_deg,
            solve.timestamp,
            self.site,
        )

        delta_alt_deg = target_altaz.alt_deg - measured_altaz.alt_deg
        delta_az_deg = normalize_signed_degrees(target_altaz.az_deg - measured_altaz.az_deg)

        ver = VerificationResult(
            measured_ra_hours=solve.ra_hours,
            measured_dec_deg=solve.dec_deg,
            remaining_error_deg=rem_err_deg,
            remaining_error_arcmin=rem_err_deg * 60.0,
            delta_alt_deg=delta_alt_deg,
            delta_az_deg=delta_az_deg,
        )

        self.state = AlignmentState.VERIFIED if rem_err_deg < (2.0 / 60.0) else AlignmentState.ADJUSTING
        self.log.add(
            f"Verification: remaining={ver.remaining_error_arcmin:.2f} arcmin, "
            f"delta_alt={ver.delta_alt_deg * 60.0:.2f} arcmin, delta_az={ver.delta_az_deg * 60.0:.2f} arcmin"
        )
        return ver

    def summary_dict(self) -> dict:
        data = {
            "state": self.state.name,
            "solve_count": len(self.solves),
            "log": list(self.log.messages),
        }
        if self.result is not None:
            data["result"] = {
                "axis_error_arcmin": self.result.axis.angular_error_arcmin,
                "delta_alt_arcmin": self.result.axis.delta_alt_arcmin,
                "delta_az_arcmin": self.result.axis.delta_az_arcmin,
                "fit_residual_deg": self.result.axis.fit_residual_deg,
                "target_ra_hours": self.result.target.target_ra_hours,
                "target_dec_deg": self.result.target.target_dec_deg,
                "target_alt_deg": self.result.target.target_alt_deg,
                "target_az_deg": self.result.target.target_az_deg,
            }
        return data


# =========================
# Misc helpers for UI / integration
# =========================


def normalize_signed_degrees(angle_deg: float) -> float:
    return ((angle_deg + 180.0) % 360.0) - 180.0


def format_arcmin(angle_deg: float) -> str:
    return f"{angle_deg * 60.0:+.2f} arcmin"


def describe_knob_guidance(delta_alt_deg: float, delta_az_deg: float) -> str:
    alt_text = "raise ALT" if delta_alt_deg > 0 else "lower ALT"
    az_text = "increase AZ" if delta_az_deg > 0 else "decrease AZ"
    return (
        f"Base correction: {alt_text} by {abs(delta_alt_deg) * 60.0:.2f} arcmin; "
        f"{az_text} by {abs(delta_az_deg) * 60.0:.2f} arcmin. "
        f"Map signs to physical knob directions on your mount once experimentally."
    )


__all__ = [
    "AlignmentResult",
    "AlignmentState",
    "AltAzCoord",
    "AxisEstimate",
    "EquatorialCoord",
    "ObserverSite",
    "PlateSolve",
    "PolarAlignmentSession",
    "SessionLog",
    "TargetPoint",
    "VerificationResult",
    "Vec3",
    "altaz_to_enu",
    "angular_distance_deg",
    "axis_error_to_altaz",
    "compute_target_after_polar_fix",
    "describe_knob_guidance",
    "enu_vec_to_altaz",
    "eq_vec_to_enu",
    "estimate_cone_fit_residual_deg",
    "estimate_ra_axis_from_three_solves",
    "format_arcmin",
    "gmst_rad",
    "julian_date",
    "live_remaining_error_deg",
    "lst_rad",
    "north_celestial_pole_enu",
    "north_celestial_pole_eq",
    "normalize_signed_degrees",
    "ra_hours_to_rad",
    "rad_to_ra_hours",
    "radec_to_altaz",
    "radec_to_enu",
    "radec_to_vec",
    "rodrigues_rotate",
    "rotation_to_move_axis",
    "vec_to_radec",
]
