from __future__ import annotations

from math import cos, radians, sin, tan
from dataclasses import dataclass
from threading import RLock
from time import monotonic

from .constants import (
    POLAR_AXIS_STOP_AFTER_SECONDS,
    POLAR_DEC_SPEED_TOLERANCE_PERCENT,
    POLAR_GUIDE_TIMEOUT_SECONDS,
    POLAR_MAX_SPEED_JUMP_DEGREES_PER_SECOND,
    POLAR_MIN_SAMPLES,
    POLAR_RA_SPEED_TOLERANCE_PERCENT,
    SIDEREAL_RATE_DEGREES_PER_SECOND,
    SIDEREAL_RATE_HOURS_PER_SECOND,
)
from .physics import Dec, DecPerSecond, Ha, HaPerSecond, PointCoordinates


@dataclass(frozen=True, slots=True)
class PoleOffset:
    ra_bias: Ha
    dec_bias: Dec
    reference_position: PointCoordinates
    samples: int


def compute_pole_offset(dec_drift: DecPerSecond, ra_drift: HaPerSecond, ha: Ha, dec: Dec) -> tuple[Ha, Dec]:
    ha_radians = radians(ha.hours * 15.0)
    dec_radians = radians(dec.degrees)
    tangent = tan(dec_radians)
    if abs(tangent) < 1e-6:
        raise ValueError("Declination is too close to 0 degrees for the RA equation")

    rhs_ra = ((ra_drift.hours_per_second - SIDEREAL_RATE_HOURS_PER_SECOND) / SIDEREAL_RATE_HOURS_PER_SECOND) / tangent
    rhs_dec = dec_drift.degrees_per_second / SIDEREAL_RATE_DEGREES_PER_SECOND
    eps_e_degrees = -rhs_dec * sin(ha_radians) + rhs_ra * cos(ha_radians)
    eps_n_degrees = rhs_dec * cos(ha_radians) + rhs_ra * sin(ha_radians)
    return Ha(eps_e_degrees / 15.0), Dec(eps_n_degrees)


def compute_guide_speeds(eps_east: Ha, eps_north: Dec, ha: Ha, dec: Dec) -> tuple[HaPerSecond, DecPerSecond]:
    ha_radians = radians(ha.hours * 15.0)
    dec_radians = radians(dec.degrees)
    eps_e_degrees = eps_east.hours * 15.0
    eps_n_degrees = eps_north.degrees
    dec_drift = SIDEREAL_RATE_DEGREES_PER_SECOND * (eps_n_degrees * cos(ha_radians) - eps_e_degrees * sin(ha_radians))
    ra_drift = SIDEREAL_RATE_HOURS_PER_SECOND * (1.0 + tan(dec_radians) * (eps_n_degrees * sin(ha_radians) + eps_e_degrees * cos(ha_radians)))
    return HaPerSecond(ra_drift), DecPerSecond(dec_drift)


class PolarCompensator:
    def __init__(
        self,
        min_samples: int = POLAR_MIN_SAMPLES,
        guide_timeout_seconds: float = POLAR_GUIDE_TIMEOUT_SECONDS,
        axis_stop_after_seconds: float = POLAR_AXIS_STOP_AFTER_SECONDS,
        max_speed_jump_degrees_per_second: float = POLAR_MAX_SPEED_JUMP_DEGREES_PER_SECOND,
        ra_speed_tolerance_percent: float = POLAR_RA_SPEED_TOLERANCE_PERCENT,
        dec_speed_tolerance_percent: float = POLAR_DEC_SPEED_TOLERANCE_PERCENT,
    ) -> None:
        self._min_samples = min_samples
        self._guide_timeout_seconds = guide_timeout_seconds
        self._axis_stop_after_seconds = axis_stop_after_seconds
        self._max_speed_jump_degrees_per_second = max_speed_jump_degrees_per_second
        self._ra_speed_tolerance_percent = ra_speed_tolerance_percent
        self._dec_speed_tolerance_percent = dec_speed_tolerance_percent
        self._lock = RLock()
        self._current_position = PointCoordinates(ra=Ha(0.0), dec=Dec(0.0))
        self._ra_speeds: list[HaPerSecond] = [HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND)]
        self._dec_speeds: list[DecPerSecond] = [DecPerSecond(0.0)]
        now = monotonic()
        self._last_guide_pulse = now
        self._last_ra_guide_pulse = now
        self._last_dec_guide_pulse = now
        self._stable_guide_ra_pulses = 0
        self._stable_guide_dec_pulses = 0
        self._offset: PoleOffset | None = None

    def reset(self) -> None:
        with self._lock:
            now = monotonic()
            self._ra_speeds = [HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND)]
            self._dec_speeds = [DecPerSecond(0.0)]
            self._last_guide_pulse = now
            self._last_ra_guide_pulse = now
            self._last_dec_guide_pulse = now
            self._stable_guide_ra_pulses = 0
            self._stable_guide_dec_pulses = 0
            self._offset = None

    def update_position(self, position: PointCoordinates) -> None:
        with self._lock:
            self._current_position = position

    def record_guide_speeds(
        self,
        ra_speed: HaPerSecond,
        dec_speed: DecPerSecond,
        position: PointCoordinates,
        timestamp: float | None = None,
    ) -> None:
        now = monotonic() if timestamp is None else timestamp
        with self._lock:
            self._current_position = position
            if abs(ra_speed.hours_per_second) > 0.0:
                self._guide_ra_locked(ra_speed, now)
            if abs(dec_speed.degrees_per_second) > 0.0:
                self._guide_dec_locked(dec_speed, now)

    def compute_pole_offset(self) -> PoleOffset | None:
        with self._lock:
            return self._compute_pole_offset_locked()

    def guide_ra(self, speed: HaPerSecond, timestamp: float | None = None) -> None:
        now = monotonic() if timestamp is None else timestamp
        with self._lock:
            self._guide_ra_locked(speed, now)

    def guide_dec(self, speed: DecPerSecond, timestamp: float | None = None) -> None:
        now = monotonic() if timestamp is None else timestamp
        with self._lock:
            self._guide_dec_locked(speed, now)

    def _compute_pole_offset_locked(self) -> PoleOffset | None:
        if self._stable_guide_ra_pulses < self._min_samples or self._stable_guide_dec_pulses < self._min_samples:
            return None

        eps_east, eps_north = compute_pole_offset(self.dec_speed(), self.ra_speed(), self._current_position.ra, self._current_position.dec)
        return PoleOffset(
            ra_bias=eps_east,
            dec_bias=eps_north,
            reference_position=self._current_position,
            samples=min(self._stable_guide_ra_pulses, self._stable_guide_dec_pulses),
        )

    def compute_guide_speeds(self, position: PointCoordinates) -> tuple[HaPerSecond, DecPerSecond]:
        with self._lock:
            if self._offset is None:
                return HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND), DecPerSecond(0.0)

            return compute_guide_speeds(self._offset.ra_bias, self._offset.dec_bias, position.ra, position.dec)

    def stable_offset(self) -> PoleOffset | None:
        with self._lock:
            return self._offset

    def takeover_speeds(self, position: PointCoordinates, timestamp: float | None = None) -> tuple[HaPerSecond, DecPerSecond] | None:
        now = monotonic() if timestamp is None else timestamp
        with self._lock:
            self._current_position = position
            speeds = self._get_guide_speeds_locked(now)
            if speeds is None:
                return None

            ra_speed, dec_speed = speeds
            return (
                ra_speed if ra_speed is not None else HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND),
                dec_speed if dec_speed is not None else DecPerSecond(0.0),
            )

    def ra_speed(self) -> HaPerSecond:
        return HaPerSecond(sum(item.hours_per_second for item in self._ra_speeds) / len(self._ra_speeds))

    def dec_speed(self) -> DecPerSecond:
        return DecPerSecond(sum(item.degrees_per_second for item in self._dec_speeds) / len(self._dec_speeds))

    def _guide_ra_locked(self, speed: HaPerSecond, now: float) -> None:
        previous = self._ra_speeds[-1]
        if now - self._last_guide_pulse > self._guide_timeout_seconds:
            self._stable_guide_ra_pulses = 0

        self._last_guide_pulse = now
        self._last_ra_guide_pulse = now
        if self._stable_guide_ra_pulses == 0 and previous == HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND):
            self._stable_guide_ra_pulses = 1
        elif self._speed_is_stable(previous.to_degrees_per_second(), speed.to_degrees_per_second(), self._ra_speed_tolerance_percent):
            self._stable_guide_ra_pulses += 1
        else:
            self._stable_guide_ra_pulses = 0

        self._ra_speeds.append(speed)
        self._ra_speeds = self._ra_speeds[-self._min_samples * 10 :]
        self._offset = self._compute_pole_offset_locked()

    def _guide_dec_locked(self, speed: DecPerSecond, now: float) -> None:
        previous = self._dec_speeds[-1]
        if now - self._last_guide_pulse > self._guide_timeout_seconds:
            self._stable_guide_dec_pulses = 0

        self._last_guide_pulse = now
        self._last_dec_guide_pulse = now
        if self._stable_guide_dec_pulses == 0 and previous == DecPerSecond(0.0):
            self._stable_guide_dec_pulses = 1
        elif self._speed_is_stable(previous.degrees_per_second, speed.degrees_per_second, self._dec_speed_tolerance_percent):
            self._stable_guide_dec_pulses += 1
        else:
            self._stable_guide_dec_pulses = 0

        self._dec_speeds.append(speed)
        self._dec_speeds = self._dec_speeds[-self._min_samples * 4 :]
        self._offset = self._compute_pole_offset_locked()

    def _get_guide_speeds_locked(self, now: float) -> tuple[HaPerSecond | None, DecPerSecond | None] | None:
        is_external_guide = now - self._last_guide_pulse < self._guide_timeout_seconds
        is_stable_guide = self._stable_guide_ra_pulses >= self._min_samples and self._stable_guide_dec_pulses >= self._min_samples
        is_external_guide_ra = now - self._last_ra_guide_pulse < self._axis_stop_after_seconds
        is_external_guide_dec = now - self._last_dec_guide_pulse < self._axis_stop_after_seconds

        if is_external_guide:
            if not is_external_guide_ra and is_external_guide_dec and self.ra_speed() != HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND):
                self._ra_speeds = [HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND)]
                return HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND), None

            if not is_external_guide_dec and is_external_guide_ra and self.dec_speed() != DecPerSecond(0.0):
                self._dec_speeds = [DecPerSecond(0.0)]
                return None, DecPerSecond(0.0)

        if not is_external_guide and is_stable_guide:
            self._offset = self._compute_pole_offset_locked()
            if self._offset is None:
                return None

            ra_speed, dec_speed = compute_guide_speeds(
                self._offset.ra_bias,
                self._offset.dec_bias,
                self._current_position.ra,
                self._current_position.dec,
            )
            self._ra_speeds = [ra_speed]
            self._dec_speeds = [dec_speed]
            return ra_speed, dec_speed

        if not is_stable_guide and not is_external_guide:
            had_compensation = self._offset is not None or self.ra_speed() != HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND) or self.dec_speed() != DecPerSecond(0.0)
            self._ra_speeds = [HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND)]
            self._dec_speeds = [DecPerSecond(0.0)]
            self._offset = None
            self._stable_guide_ra_pulses = 0
            self._stable_guide_dec_pulses = 0
            if had_compensation:
                return HaPerSecond(SIDEREAL_RATE_HOURS_PER_SECOND), DecPerSecond(0.0)

        return None

    def _speed_is_stable(self, previous: float, current: float, tolerance_percent: float) -> bool:
        if abs(current - previous) > self._max_speed_jump_degrees_per_second:
            return False

        denominator = previous if previous != 0.0 else (current if current != 0.0 else 1.0)
        delta_percent = ((current - previous) / denominator) * 100.0
        return abs(delta_percent) < tolerance_percent
