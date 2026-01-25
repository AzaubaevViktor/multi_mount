from __future__ import annotations

import dataclasses
import datetime as dt
import math
from typing import Mapping

from lx200.coords import clamp, wrap_hours
from lx200.models import LX200Dec, LX200Ra
from lx200.protocol import LX200Constants

from .constants import PolarAlignmentConstants, PolarAlignmentFieldKey, PolarAlignmentFileKey


class PolarAlignmentError(Exception):
    pass


class PolarAlignmentParseError(PolarAlignmentError):
    pass


class PolarAlignmentDataError(PolarAlignmentError):
    pass


class PolarAlignmentMathError(PolarAlignmentError):
    pass


@dataclasses.dataclass(frozen=True)
class PolarAlignmentSample:
    timestamp_utc: dt.datetime
    ra: LX200Ra
    dec: LX200Dec

    def __post_init__(self) -> None:
        if self.timestamp_utc.tzinfo is None:
            raise PolarAlignmentDataError("timestamp must include timezone")

    def with_ra(self, ra: LX200Ra) -> "PolarAlignmentSample":
        return PolarAlignmentSample(timestamp_utc=self.timestamp_utc, ra=ra, dec=self.dec)


@dataclasses.dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if self.norm() <= PolarAlignmentConstants.VECTOR_EPS:
            raise PolarAlignmentMathError("vector norm is too small")

    def norm(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self) -> "Vector3":
        n = self.norm()
        if n <= PolarAlignmentConstants.VECTOR_EPS:
            raise PolarAlignmentMathError("vector norm is too small to normalize")
        return Vector3(x=self.x / n, y=self.y / n, z=self.z / n)

    def dot(self, other: "Vector3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vector3") -> "Vector3":
        return Vector3(
            x=self.y * other.z - self.z * other.y,
            y=self.z * other.x - self.x * other.z,
            z=self.x * other.y - self.y * other.x,
        )

    def sub(self, other: "Vector3") -> "Vector3":
        return Vector3(x=self.x - other.x, y=self.y - other.y, z=self.z - other.z)

    def mul(self, scalar: float) -> "Vector3":
        return Vector3(x=self.x * scalar, y=self.y * scalar, z=self.z * scalar)


@dataclasses.dataclass(frozen=True)
class PolarAlignmentAxis:
    ra: LX200Ra
    dec: LX200Dec
    error_deg: float

    def __post_init__(self) -> None:
        if self.error_deg < PolarAlignmentConstants.VECTOR_EPS:
            return
        if self.error_deg > PolarAlignmentConstants.DEGREES_PER_CIRCLE:
            raise PolarAlignmentDataError("alignment error must be within 0..360 degrees")


@dataclasses.dataclass(frozen=True)
class PolarAlignmentResult:
    axis: PolarAlignmentAxis
    corrected_samples: dict[PolarAlignmentFileKey, PolarAlignmentSample]


def parse_samples(payload: Mapping[str, Mapping[str, str]]) -> dict[PolarAlignmentFileKey, PolarAlignmentSample]:
    samples: dict[PolarAlignmentFileKey, PolarAlignmentSample] = {}
    for key in PolarAlignmentFileKey:
        raw = payload.get(key.value)
        if raw is None:
            raise PolarAlignmentParseError(f"missing sample {key.value!r}")
        samples[key] = _parse_sample(raw, key)
    return samples


def _parse_sample(raw: Mapping[str, str], key: PolarAlignmentFileKey) -> PolarAlignmentSample:
    try:
        timestamp_raw = raw[PolarAlignmentFieldKey.TIME.value]
        ra_raw = raw[PolarAlignmentFieldKey.RA.value]
        dec_raw = raw[PolarAlignmentFieldKey.DEC.value]
    except KeyError as exc:
        raise PolarAlignmentParseError(f"missing field for {key.value!r}") from exc
    timestamp = _parse_timestamp(timestamp_raw)
    ra = LX200Ra.from_string(ra_raw)
    dec = LX200Dec.from_string(dec_raw)
    return PolarAlignmentSample(timestamp_utc=timestamp, ra=ra, dec=dec)


def _parse_timestamp(value: str) -> dt.datetime:
    try:
        parsed = dt.datetime.fromisoformat(value)
    except ValueError as exc:
        raise PolarAlignmentParseError(f"invalid timestamp {value!r}") from exc
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=dt.timezone.utc)
    return parsed.astimezone(dt.timezone.utc)


def apply_tracking_correction(
    samples: Mapping[PolarAlignmentFileKey, PolarAlignmentSample],
    *,
    tracking_sign: int,
) -> dict[PolarAlignmentFileKey, PolarAlignmentSample]:
    if tracking_sign not in (LX200Constants.SIGN_NEG_INT, LX200Constants.SIGN_POS_INT):
        raise PolarAlignmentDataError("tracking sign must be -1 or 1")
    t1 = samples[PolarAlignmentFileKey.T1].timestamp_utc
    corrected: dict[PolarAlignmentFileKey, PolarAlignmentSample] = {}
    for key, sample in samples.items():
        delta_s = (sample.timestamp_utc - t1).total_seconds()
        correction = tracking_sign * delta_s * PolarAlignmentConstants.SIDEREAL_HOURS_PER_SECOND
        corrected_ra = wrap_hours(sample.ra.hours - correction)
        corrected[key] = sample.with_ra(LX200Ra(hours=corrected_ra))
    return corrected


# TODO: prompt

def compute_axis(samples: Mapping[PolarAlignmentFileKey, PolarAlignmentSample]) -> PolarAlignmentAxis:
    v1 = _vector_from_sample(samples[PolarAlignmentFileKey.T1])
    v2 = _vector_from_sample(samples[PolarAlignmentFileKey.T2])
    v3 = _vector_from_sample(samples[PolarAlignmentFileKey.T3])
    axis = _axis_from_vectors(v1, v2, v3)
    axis = _normalize_axis(axis)
    ra, dec = _axis_to_ra_dec(axis)
    error_deg = _axis_error_deg(axis)
    return PolarAlignmentAxis(ra=ra, dec=dec, error_deg=error_deg)


def _vector_from_sample(sample: PolarAlignmentSample) -> Vector3:
    ra_rad = math.radians(sample.ra.hours * PolarAlignmentConstants.DEG_PER_HOUR)
    dec_rad = math.radians(sample.dec.degrees)
    cos_dec = math.cos(dec_rad)
    return Vector3(
        x=cos_dec * math.cos(ra_rad),
        y=cos_dec * math.sin(ra_rad),
        z=math.sin(dec_rad),
    )


def _axis_from_vectors(v1: Vector3, v2: Vector3, v3: Vector3) -> Vector3:
    delta_12 = v1.sub(v2)
    delta_13 = v1.sub(v3)
    axis = delta_12.cross(delta_13)
    return axis.normalized()


def _normalize_axis(axis: Vector3) -> Vector3:
    if axis.z < PolarAlignmentConstants.ZERO_FLOAT:
        axis = axis.mul(PolarAlignmentConstants.AXIS_SIGN_NEGATIVE)
    return axis.normalized()


def _axis_to_ra_dec(axis: Vector3) -> tuple[LX200Ra, LX200Dec]:
    ra_rad = math.atan2(axis.y, axis.x)
    if ra_rad < PolarAlignmentConstants.ZERO_FLOAT:
        ra_rad += PolarAlignmentConstants.RADIANS_PER_CIRCLE
    ra_hours = wrap_hours(
        (ra_rad / PolarAlignmentConstants.RADIANS_PER_CIRCLE) * PolarAlignmentConstants.HOURS_PER_DAY
    )
    dec_rad = math.asin(clamp(axis.z, -PolarAlignmentConstants.AXIS_SIGN_POSITIVE, PolarAlignmentConstants.AXIS_SIGN_POSITIVE))
    return LX200Ra(hours=ra_hours), LX200Dec(degrees=math.degrees(dec_rad))


def _axis_error_deg(axis: Vector3) -> float:
    dot = clamp(axis.z, -PolarAlignmentConstants.AXIS_SIGN_POSITIVE, PolarAlignmentConstants.AXIS_SIGN_POSITIVE)
    error_rad = math.acos(dot)
    return math.degrees(error_rad)
