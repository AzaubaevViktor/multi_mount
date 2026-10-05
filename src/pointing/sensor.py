"""Raw DEC sensor capability; no calibration or sky coordinates cross this boundary."""

from dataclasses import dataclass
from enum import StrEnum
from math import isfinite
from typing import Protocol

from utils.polar_align_lib import Vec3


class SensorState(StrEnum):
    AVAILABLE = "available"
    NOT_CONNECTED = "not_connected"
    UNSUPPORTED = "unsupported"
    DEVICE_NOT_FOUND = "device_not_found"
    NO_DATA = "no_data"
    INVALID_DATA = "invalid_data"
    TRANSPORT_ERROR = "transport_error"


@dataclass(frozen=True)
class SensorReading:
    state: SensorState
    sequence: int | None = None
    age_ms: int | None = None
    gravity: Vec3 | None = None  # m/s², points down, sensor frame
    magnetic: Vec3 | None = None  # µT, same sensor frame

    def __post_init__(self) -> None:
        if self.state is not SensorState.AVAILABLE:
            if self.gravity is not None or self.magnetic is not None:
                raise ValueError("unavailable readings must not carry vectors")
            return
        if self.sequence is None or self.sequence < 0 or self.age_ms is None or self.age_ms < 0:
            raise ValueError("available readings need sequence and age")
        if self.gravity is None:
            raise ValueError("available readings need gravity")
        for vector in (self.gravity, self.magnetic):
            if vector is not None and (not all(isfinite(v) for v in vector.as_tuple()) or not isfinite(vector.norm())):
                raise ValueError("invalid sensor vector")


class SensorReader(Protocol):
    def read_orientation_sensor(self) -> SensorReading: ...


def sensor_basis(reading: SensorReading) -> tuple[Vec3, Vec3, Vec3]:
    """Return East, magnetic North and Up, all expressed in the sensor frame."""
    if reading.gravity is None or reading.magnetic is None:
        raise ValueError("gravity and magnetic measurements are required")
    up = reading.gravity.normalize() * -1
    horizontal = reading.magnetic - up * reading.magnetic.dot(up)
    if horizontal.norm() <= max(1e-9, reading.magnetic.norm() * 0.01):
        raise ValueError("magnetic heading is degenerate")
    north = horizontal.normalize()
    return north.cross(up).normalize(), north, up
