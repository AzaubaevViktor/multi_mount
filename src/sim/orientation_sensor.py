"""Independent sensor poses: changing the mount's SYNC never changes these readings."""

from math import cos, radians, sin
import threading

from pointing.sensor import SensorReading, SensorState
from sim.clock import Clock
from utils.polar_align_lib import Vec3, altaz_to_enu


class OrientationSensorSim:
    def __init__(self, clock: Clock) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._sequence = 0
        self._updated = clock.monotonic()
        self._reading = SensorReading(SensorState.NO_DATA)
        self._frozen = False
        self.set_pose(45, 0)

    def set_raw(self, reading: SensorReading, *, frozen: bool = False) -> None:
        with self._lock:
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF
            self._reading = reading
            self._updated = self._clock.monotonic()
            self._frozen = frozen

    def set_pose(
        self,
        altitude_deg: float,
        azimuth_deg: float,
        *,
        mounting_deg: float = 17,
        magnetic_bias: Vec3 = Vec3(0, 0, 0),
        magnetic_scale: Vec3 = Vec3(1, 1, 1),
    ) -> None:
        forward = altaz_to_enu(altitude_deg, azimuth_deg)
        left = altaz_to_enu(0, azimuth_deg - 90)
        up = forward.cross(left)
        angle = radians(mounting_deg)
        axes = (forward * cos(angle) - up * sin(angle), left, forward * sin(angle) + up * cos(angle))
        gravity = Vec3(*(Vec3(0, 0, -9.807).dot(axis) for axis in axes))
        magnetic = Vec3(*(Vec3(0, 20, -40).dot(axis) for axis in axes))
        magnetic = Vec3(
            magnetic.x * magnetic_scale.x + magnetic_bias.x,
            magnetic.y * magnetic_scale.y + magnetic_bias.y,
            magnetic.z * magnetic_scale.z + magnetic_bias.z,
        )
        self.set_raw(SensorReading(SensorState.AVAILABLE, 0, 0, gravity, magnetic))

    def read_orientation_sensor(self) -> SensorReading:
        now = self._clock.monotonic()
        with self._lock:
            if self._reading.state is not SensorState.AVAILABLE:
                return self._reading
            age_ms = int((now - self._updated) * 1000)
            if not self._frozen and age_ms >= 100:
                self._sequence = (self._sequence + age_ms // 100) & 0xFFFFFFFF
                self._updated = now
                age_ms = 0
            return SensorReading(
                self._reading.state,
                self._sequence,
                (self._reading.age_ms or 0) + age_ms,
                self._reading.gravity,
                self._reading.magnetic,
            )
