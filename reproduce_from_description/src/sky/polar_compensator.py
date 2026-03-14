from __future__ import annotations

from dataclasses import dataclass
from statistics import fmean
from threading import RLock
from time import monotonic

from .constants import (
    POLAR_GUIDE_TIMEOUT_SECONDS,
    POLAR_MAX_POSITION_DELTA_DEGREES,
    POLAR_MAX_SPEED_JUMP_DEGREES_PER_SECOND,
    POLAR_MIN_SAMPLES,
    POLAR_TAKEOVER_DELAY_SECONDS,
    POLAR_TAKEOVER_MAX_SECONDS,
)
from .physics import Dec, DecPerSecond, Ha, HaPerSecond, PointCoordinates


@dataclass(frozen=True, slots=True)
class GuideSample:
    timestamp: float
    ra_speed: HaPerSecond
    dec_speed: DecPerSecond
    position: PointCoordinates


@dataclass(frozen=True, slots=True)
class PoleOffset:
    ra_bias: HaPerSecond
    dec_bias: DecPerSecond
    reference_position: PointCoordinates
    samples: int


class PolarCompensator:
    def __init__(
        self,
        min_samples: int = POLAR_MIN_SAMPLES,
        guide_timeout_seconds: float = POLAR_GUIDE_TIMEOUT_SECONDS,
        takeover_delay_seconds: float = POLAR_TAKEOVER_DELAY_SECONDS,
        takeover_max_seconds: float = POLAR_TAKEOVER_MAX_SECONDS,
        max_position_delta_degrees: float = POLAR_MAX_POSITION_DELTA_DEGREES,
        max_speed_jump_degrees_per_second: float = POLAR_MAX_SPEED_JUMP_DEGREES_PER_SECOND,
    ) -> None:
        self._min_samples = min_samples
        self._guide_timeout_seconds = guide_timeout_seconds
        self._takeover_delay_seconds = takeover_delay_seconds
        self._takeover_max_seconds = takeover_max_seconds
        self._max_position_delta_degrees = max_position_delta_degrees
        self._max_speed_jump_degrees_per_second = max_speed_jump_degrees_per_second
        self._lock = RLock()
        self._samples: list[GuideSample] = []
        self._offset: PoleOffset | None = None

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()
            self._offset = None

    def record_guide_speeds(
        self,
        ra_speed: HaPerSecond,
        dec_speed: DecPerSecond,
        position: PointCoordinates,
        timestamp: float | None = None,
    ) -> None:
        now = monotonic() if timestamp is None else timestamp
        sample = GuideSample(timestamp=now, ra_speed=ra_speed, dec_speed=dec_speed, position=position)
        with self._lock:
            if self._samples:
                previous = self._samples[-1]
                if now - previous.timestamp > self._guide_timeout_seconds:
                    self._samples.clear()
                elif abs(previous.position.dec.delta_to(position.dec)) > self._max_position_delta_degrees:
                    self._samples.clear()
                elif abs(previous.position.ra.shortest_delta_to(position.ra) * 15.0) > self._max_position_delta_degrees:
                    self._samples.clear()
                elif abs(previous.ra_speed.to_degrees_per_second() - ra_speed.to_degrees_per_second()) > self._max_speed_jump_degrees_per_second:
                    self._samples.clear()
                elif abs(previous.dec_speed.degrees_per_second - dec_speed.degrees_per_second) > self._max_speed_jump_degrees_per_second:
                    self._samples.clear()

            self._samples.append(sample)
            if len(self._samples) > self._min_samples * 3:
                self._samples = self._samples[-self._min_samples * 3 :]
            self._offset = self.compute_pole_offset_locked()

    def compute_pole_offset(self) -> PoleOffset | None:
        with self._lock:
            return self.compute_pole_offset_locked()

    def compute_pole_offset_locked(self) -> PoleOffset | None:
        if len(self._samples) < self._min_samples:
            return None

        recent = self._samples[-self._min_samples :]
        ra_mean = fmean(sample.ra_speed.hours_per_second for sample in recent)
        dec_mean = fmean(sample.dec_speed.degrees_per_second for sample in recent)
        reference_ra = fmean(sample.position.ra.hours for sample in recent)
        reference_dec = fmean(sample.position.dec.degrees for sample in recent)
        return PoleOffset(
            ra_bias=HaPerSecond(ra_mean),
            dec_bias=DecPerSecond(dec_mean),
            reference_position=PointCoordinates(ra=Ha(reference_ra), dec=Dec(reference_dec)),
            samples=len(recent),
        )

    def compute_guide_speeds(self, position: PointCoordinates) -> tuple[HaPerSecond, DecPerSecond]:
        with self._lock:
            if self._offset is None:
                return HaPerSecond(0.0), DecPerSecond(0.0)

            return HaPerSecond(-self._offset.ra_bias.hours_per_second), DecPerSecond(-self._offset.dec_bias.degrees_per_second)

    def stable_offset(self) -> PoleOffset | None:
        with self._lock:
            return self._offset

    def takeover_speeds(self, position: PointCoordinates, timestamp: float | None = None) -> tuple[HaPerSecond, DecPerSecond] | None:
        now = monotonic() if timestamp is None else timestamp
        with self._lock:
            if self._offset is None or not self._samples:
                return None

            age = now - self._samples[-1].timestamp
            if age < self._takeover_delay_seconds:
                return None
            if age > self._takeover_max_seconds:
                self._samples.clear()
                self._offset = None
                return None

            return self.compute_guide_speeds(position)
