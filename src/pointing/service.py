"""Host-owned local calibration of a rigidly mounted gravity/magnetic sensor."""

from collections import deque
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime, timedelta
from hashlib import sha256
import json
from math import acos, degrees, isfinite
import os
from pathlib import Path
import tempfile
import threading
from typing import Any, Callable
from uuid import uuid4

from pointing.sensor import SensorReader, SensorReading, SensorState, sensor_basis
from utils.polar_align_lib import (
    AltAzCoord,
    ObserverSite,
    Vec3,
    altaz_to_enu,
    altaz_to_radec,
    enu_vec_to_altaz,
    radec_to_altaz,
)


@dataclass(frozen=True)
class Measurement:
    measurement_id: str
    timestamp: datetime
    reading: SensorReading


@dataclass(frozen=True)
class CalibrationPoint:
    measurement_id: str
    timestamp: str
    sequence: int
    gravity: Vec3
    magnetic: Vec3
    ra_hours: float
    dec_deg: float
    target: AltAzCoord
    boresight: Vec3

    @classmethod
    def from_measurement(
        cls,
        measurement: Measurement,
        ra_hours: float,
        dec_deg: float,
        site: ObserverSite,
    ) -> "CalibrationPoint":
        reading = measurement.reading
        if reading.gravity is None or reading.magnetic is None or reading.sequence is None:
            raise ValueError("calibration needs a complete measurement")
        validate_solve(ra_hours, dec_deg, measurement.timestamp)
        target = radec_to_altaz(ra_hours, dec_deg, measurement.timestamp, site)
        east, north, up = sensor_basis(reading)
        direction = altaz_to_enu(target.alt_deg, target.az_deg)
        return cls(
            measurement.measurement_id,
            measurement.timestamp.isoformat(),
            reading.sequence,
            reading.gravity,
            reading.magnetic,
            ra_hours,
            dec_deg,
            target,
            (east * direction.x + north * direction.y + up * direction.z).normalize(),
        )


@dataclass(frozen=True)
class Calibration:
    site: ObserverSite | None = None
    points: tuple[CalibrationPoint, ...] = ()
    latest_sync: dict[str, Any] | None = None
    drift: bool = False


def angular_distance(first: Vec3, second: Vec3) -> float:
    return degrees(acos(max(-1, min(1, first.normalize().dot(second.normalize())))))


def validate_site(latitude_deg: float, longitude_deg: float) -> ObserverSite:
    if (
        not isfinite(latitude_deg)
        or not isfinite(longitude_deg)
        or not -90 <= latitude_deg <= 90
        or not -180 <= longitude_deg <= 180
    ):
        raise ValueError("site requires finite latitude [-90,90] and east longitude [-180,180]")
    return ObserverSite(latitude_deg, longitude_deg)


def validate_solve(ra_hours: float, dec_deg: float, timestamp: datetime) -> None:
    if not isfinite(ra_hours) or not isfinite(dec_deg) or not 0 <= ra_hours < 24 or not -90 <= dec_deg <= 90:
        raise ValueError("solve requires finite RA [0,24) hours and declination [-90,90] degrees")
    if timestamp.tzinfo is None or timestamp.utcoffset() is None:
        raise ValueError("solve timestamp must be timezone-aware")


def predict(
    reading: SensorReading, calibration: Calibration, coverage_deg: float
) -> tuple[AltAzCoord | None, float | None]:
    if reading.gravity is None or reading.magnetic is None or not calibration.points:
        return None, None
    east, north, up = sensor_basis(reading)
    neighbours = sorted(
        (
            (
                max(
                    angular_distance(reading.gravity, point.gravity), angular_distance(reading.magnetic, point.magnetic)
                ),
                point,
            )
            for point in calibration.points
        ),
        key=lambda item: item[0],
    )
    nearest = neighbours[0][0]
    if nearest > coverage_deg:
        return None, nearest
    # Coincident measurements average vectors, including across azimuth 0/360.
    local = (
        [item for item in neighbours if item[0] <= 0.1]
        if nearest <= 0.1
        else [item for item in neighbours[:4] if item[0] <= coverage_deg]
    )
    mean = Vec3(0, 0, 0)
    total_weight = 0.0
    for distance, point in local:
        weight = 1 / (1 + distance * distance)
        mean += point.boresight * weight
        total_weight += weight
    if mean.norm() / total_weight < 0.95:
        return None, nearest
    boresight = mean.normalize()
    return enu_vec_to_altaz(Vec3(east.dot(boresight), north.dot(boresight), up.dot(boresight))), nearest


class PointingService:
    def __init__(
        self,
        reader: SensorReader,
        storage: Path | None = None,
        *,
        now: Callable[[], datetime] = lambda: datetime.now(UTC),
        max_age_ms: int = 2000,
        coverage_deg: float = 25,
        drift_deg: float = 2,
    ) -> None:
        if max_age_ms <= 0 or not 0 < coverage_deg <= 90 or not 0 < drift_deg <= 90:
            raise ValueError("invalid sensor calibration limits")
        self._reader, self._storage, self._now = reader, storage, now
        self._max_age_ms, self._coverage_deg, self._drift_deg = max_age_ms, coverage_deg, drift_deg
        self._lock = threading.Lock()
        self._generation = 0
        self._calibration = Calibration()
        self._calibrating = False
        self._measurements: deque[Measurement] = deque(maxlen=512)
        self._storage_error: str | None = None
        if storage is not None and storage.exists():
            try:
                document = json.loads(storage.read_text())
                if document["schema"] != 1:
                    raise ValueError("unsupported calibration schema")
                checksum = document.pop("sha256")
                if checksum != sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest():
                    raise ValueError("calibration checksum mismatch")
                site_data = document["site"]
                site = None if site_data is None else validate_site(**site_data)
                points = []
                for item in document["points"]:
                    gravity, magnetic = Vec3(**item["gravity"]), Vec3(**item["magnetic"])
                    reading = SensorReading(SensorState.AVAILABLE, item["sequence"], 0, gravity, magnetic)
                    if self._reading_state(reading) != "available":
                        raise ValueError("invalid calibration measurement")
                    if site is None:
                        raise ValueError("calibration points need a site")
                    points.append(
                        CalibrationPoint.from_measurement(
                            Measurement(item["measurement_id"], datetime.fromisoformat(item["timestamp"]), reading),
                            item["ra_hours"],
                            item["dec_deg"],
                            site,
                        )
                    )
                latest_sync, drift = document["latest_sync"], document["drift"]
                if not isinstance(drift, bool):
                    raise ValueError("invalid stored drift state")
                if latest_sync is not None:
                    validate_solve(
                        latest_sync["ra_hours"],
                        latest_sync["dec_deg"],
                        datetime.fromisoformat(latest_sync["timestamp"]),
                    )
                    residual = latest_sync["residual_deg"]
                    if residual is not None and (not isfinite(residual) or not 0 <= residual <= 180):
                        raise ValueError("invalid stored comparison")
                self._calibration = Calibration(site, tuple(points), latest_sync, drift)
            except (OSError, ValueError, KeyError, TypeError) as error:
                self._storage_error = f"invalid calibration storage: {error}"

    def _commit(self, calibration: Calibration, generation: int, *, reset: bool = False) -> None:
        # Prepare all bytes and disk I/O outside the state lock. Only publish a
        # still-current profile; a concurrent reset must never be overwritten.
        temporary: str | None = None
        try:
            if self._storage is not None:
                document: dict[str, Any] = {
                    "schema": 1,
                    "site": asdict(calibration.site) if calibration.site else None,
                    "points": [asdict(point) for point in calibration.points],
                    "latest_sync": calibration.latest_sync,
                    "drift": calibration.drift,
                }
                document["sha256"] = sha256(json.dumps(document, sort_keys=True, allow_nan=False).encode()).hexdigest()
                encoded = json.dumps(document, allow_nan=False, indent=2)
                self._storage.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(mode="w", dir=self._storage.parent, delete=False) as stream:
                    temporary = stream.name
                    stream.write(encoded)
                    stream.flush()
                    os.fsync(stream.fileno())
            with self._lock:
                if generation != self._generation:
                    raise ValueError("calibration changed concurrently; retry")
                if temporary is not None and self._storage is not None:
                    os.replace(temporary, self._storage)
                    temporary = None
                self._calibration = calibration
                self._storage_error = None
                if reset:
                    self._calibrating = False
                self._generation += 1
        finally:
            if temporary is not None:
                Path(temporary).unlink(missing_ok=True)

    def set_site(self, latitude_deg: float, longitude_deg: float) -> None:
        site = validate_site(latitude_deg, longitude_deg)
        with self._lock:
            calibration, generation = self._calibration, self._generation
            if calibration.points and site != calibration.site:
                raise ValueError("reset calibration before changing the site")
        self._commit(replace(calibration, site=site), generation)

    def start_calibration(self) -> None:
        with self._lock:
            if self._calibration.site is None:
                raise ValueError("configure the observer site first")
            self._calibrating = True
            self._generation += 1

    def stop_calibration(self) -> None:
        with self._lock:
            self._calibrating = False
            self._generation += 1

    def reset_calibration(self) -> None:
        with self._lock:
            calibration, generation = Calibration(self._calibration.site), self._generation
        self._commit(calibration, generation, reset=True)

    def capture(self) -> Measurement:
        try:
            reading = self._reader.read_orientation_sensor()
        except Exception:  # Transport errors are explicit API state, never zeros.
            reading = SensorReading(SensorState.TRANSPORT_ERROR)
        timestamp = self._now() - timedelta(milliseconds=reading.age_ms or 0)
        measurement = Measurement(str(uuid4()), timestamp, reading)
        with self._lock:
            self._measurements.append(measurement)
        return measurement

    def record_sync(
        self,
        ra_hours: float,
        dec_deg: float,
        *,
        timestamp: datetime | None = None,
        measurement_id: str | None = None,
    ) -> dict[str, Any]:
        if measurement_id is None:
            measurement = self.capture()
        else:
            with self._lock:
                saved = next((m for m in self._measurements if m.measurement_id == measurement_id), None)
            if saved is None:
                raise ValueError("unknown or expired measurement_id")
            measurement = saved
        timestamp = measurement.timestamp if timestamp is None else timestamp
        validate_solve(ra_hours, dec_deg, timestamp)
        if abs((timestamp - measurement.timestamp).total_seconds()) > 0.5:
            raise ValueError("solve time must match the captured sensor measurement within 0.5 seconds")
        with self._lock:
            calibration, generation, calibrating = self._calibration, self._generation, self._calibrating
        reading = measurement.reading
        result: dict[str, Any] = {
            "ra_hours": ra_hours,
            "dec_deg": dec_deg,
            "timestamp": timestamp.isoformat(),
            "measurement_id": measurement.measurement_id,
            "sample_added": False,
            "residual_deg": None,
        }
        reason = self._reading_state(reading)
        if reason == "available" and calibration.site is None:
            reason = "site_not_configured"
        if reason == "available" and calibration.site is not None:
            target = radec_to_altaz(ra_hours, dec_deg, timestamp, calibration.site)
            predicted, _ = predict(reading, calibration, self._coverage_deg)
            if predicted is not None:
                result["residual_deg"] = angular_distance(
                    altaz_to_enu(predicted.alt_deg, predicted.az_deg),
                    altaz_to_enu(target.alt_deg, target.az_deg),
                )
            if (
                calibrating
                and reading.gravity is not None
                and reading.magnetic is not None
                and reading.sequence is not None
            ):
                if any(
                    p.measurement_id == measurement.measurement_id
                    or (
                        p.sequence == reading.sequence
                        and p.gravity == reading.gravity
                        and p.magnetic == reading.magnetic
                        and abs((datetime.fromisoformat(p.timestamp) - timestamp).total_seconds())
                        < self._max_age_ms / 1000
                    )
                    for p in calibration.points
                ):
                    reason = "duplicate_measurement"
                else:
                    point = CalibrationPoint.from_measurement(
                        replace(measurement, timestamp=timestamp),
                        ra_hours,
                        dec_deg,
                        calibration.site,
                    )
                    calibration = replace(calibration, points=(*calibration.points, point))
                    result["sample_added"] = True
                    reason = "calibrating"
            elif predicted is None:
                reason = "cannot_validate" if calibration.points else "uncalibrated"
            else:
                reason = "calibration_drift" if result["residual_deg"] > self._drift_deg else "verified"
        result["status"] = reason
        drift = calibration.drift
        if reason == "calibration_drift":
            drift = True
        elif reason in ("verified", "calibrating"):
            drift = False
        self._commit(replace(calibration, latest_sync=dict(result), drift=drift), generation)
        return dict(result)

    def _reading_state(self, reading: SensorReading) -> str:
        if reading.state is not SensorState.AVAILABLE:
            return reading.state.value
        if reading.age_ms is None or reading.age_ms > self._max_age_ms:
            return "stale_data"
        if reading.gravity is None or not 5 <= reading.gravity.norm() <= 15:
            return "invalid_data"
        if reading.magnetic is None:
            return "heading_unavailable"
        try:
            sensor_basis(reading)
        except ValueError:
            return "invalid_data"
        return "available"

    def observer_site(self) -> ObserverSite | None:
        with self._lock:
            return self._calibration.site

    def status(self) -> dict[str, Any]:
        measurement = self.capture()
        with self._lock:
            calibration, calibrating = self._calibration, self._calibrating
            revision = self._generation
            storage_error = self._storage_error
        reading = measurement.reading
        state = self._reading_state(reading)
        data_status = state
        # These describe available data, not individual chips' physical
        # presence: the current DEC packet cannot distinguish absence from OVL.
        gravity_status = magnetic_status = reading.state.value
        if reading.state is SensorState.AVAILABLE:
            gravity_status = "invalid_data" if reading.gravity is None or not 5 <= reading.gravity.norm() <= 15 else "available"
            magnetic_status = "unavailable" if reading.magnetic is None else "available"
            if state == "stale_data":
                gravity_status = "stale_data"
                if reading.magnetic is not None:
                    magnetic_status = "stale_data"
            elif state == "invalid_data" and gravity_status == "available":
                magnetic_status = "invalid_data"
        altaz, nearest, equatorial = None, None, None
        if state == "available":
            altaz, nearest = predict(reading, calibration, self._coverage_deg)
            if not calibration.points:
                state = "uncalibrated"
            elif altaz is None:
                state = (
                    "calibration_inconsistent"
                    if nearest is not None and nearest <= self._coverage_deg
                    else "out_of_coverage"
                )
            elif calibration.drift:
                state, altaz = "calibration_drift", None
            else:
                state = "calibrating" if calibrating else "ready"
                if calibration.site is not None:
                    equatorial = altaz_to_radec(altaz.alt_deg, altaz.az_deg, measurement.timestamp, calibration.site)
        return {
            "status": state,
            "sensor": {
                "state": reading.state.value,
                "data_status": data_status,
                "channels": {"gravity": gravity_status, "magnetic": magnetic_status},
                "measurement_id": measurement.measurement_id,
                "timestamp": measurement.timestamp.isoformat(),
                "sequence": reading.sequence,
                "age_ms": reading.age_ms,
                "gravity_m_s2": asdict(reading.gravity) if reading.gravity else None,
                "magnetic_uT": asdict(reading.magnetic) if reading.magnetic else None,
            },
            "calibration": {
                "mode": calibrating,
                "revision": revision,
                "point_count": len(calibration.points),
                "drift": calibration.drift,
                "site": asdict(calibration.site) if calibration.site else None,
                "nearest_point_deg": nearest,
                "coverage_deg": self._coverage_deg,
                "drift_threshold_deg": self._drift_deg,
                "latest_sync": dict(calibration.latest_sync) if calibration.latest_sync else None,
                "latest_calibration": asdict(calibration.points[-1]) if calibration.points else None,
                "storage_error": storage_error,
            },
            "altaz": asdict(altaz) if altaz else None,
            "equatorial": asdict(equatorial) if equatorial else None,
        }
