"""Sensor scenarios use independent physical poses, never SYNC-derived measurements."""

from datetime import UTC, datetime, timedelta
import json
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from pointing.api import AgentAPI, AgentAPIServer
from pointing.sensor import SensorReading, SensorState
from pointing.service import PointingService, angular_distance
from sim.clock import Clock
from sim.orientation_sensor import OrientationSensorSim
from sim.realtime import RealtimeClock
from sky.physics import Dec, Ha
from tmc2209.motor import TMC2209MotorProtocolError, _Dialect
from tmc2209.protocol import Frame, Op, encode_frame, encode_sensor_payload, response_values
from tools.sim_stack import build_sim_stack
from utils.polar_align_lib import ObserverSite, Vec3, altaz_to_enu, altaz_to_radec, radec_to_altaz

TIME = datetime(2026, 10, 5, 12, tzinfo=UTC)
SITE = ObserverSite(43, 77)


@pytest.fixture
def sensor_stack():
    stack = build_sim_stack()
    stack.dec_motor.connect()
    try:
        yield stack
    finally:
        stack.dec_motor.disconnect()


def add_pose(service, sensor, altitude, azimuth, *, mounting=17, bias=Vec3(0, 0, 0), scale=Vec3(1, 1, 1), time=TIME):
    sensor.set_pose(altitude, azimuth, mounting_deg=mounting, magnetic_bias=bias, magnetic_scale=scale)
    solve = altaz_to_radec(altitude, azimuth, time, SITE)
    return service.record_sync(solve.ra_hours, solve.dec_deg)


@pytest.mark.parametrize("dialect", [_Dialect.FRAMED, _Dialect.LEGACY])
def test_raw_sensor_crosses_the_production_serial_protocol(sensor_stack, dialect):
    sensor_stack.dec_motor._dialect = dialect
    sensor_stack.orientation_sensor.set_raw(
        SensorReading(
            SensorState.AVAILABLE,
            0,
            7,
            Vec3(-1.234, 2.345, -9.001),
            Vec3(12.34, -23.45, 34.56),
        )
    )
    reading = sensor_stack.dec_motor.read_orientation_sensor()
    assert reading.gravity == Vec3(-1.234, 2.345, -9.001)
    assert reading.magnetic == Vec3(12.34, -23.45, 34.56)
    assert reading.age_ms == 7
    assert reading.sequence is not None


def test_absent_and_unconnected_devices_never_expose_zero_vectors():
    stack = build_sim_stack(dec_config={"orientation_sensor": None})
    assert stack.pointing.status()["status"] == "not_connected"
    stack.dec_motor.connect()
    try:
        service = PointingService(stack.dec_motor)
        state = service.status()
        assert state["status"] == "device_not_found"
        assert state["sensor"]["gravity_m_s2"] is None
        assert state["sensor"]["magnetic_uT"] is None
        assert state["altaz"] is None and state["equatorial"] is None
    finally:
        stack.dec_motor.disconnect()


def test_firmware_without_the_capability_is_reported_as_unsupported(sensor_stack, monkeypatch):
    from tmc2209.protocol import Response

    monkeypatch.setattr(sensor_stack.dec_motor, "_exchange", lambda *_: Response(False, {}, "unknown_cmd"))
    assert sensor_stack.dec_motor.read_orientation_sensor().state is SensorState.UNSUPPORTED


def test_protocol_rejects_inconsistent_flags_and_truncated_sensor_payload():
    for flags in (2, 4, 5, 6, 8, 255):
        with pytest.raises(TMC2209MotorProtocolError):
            response_values(Frame(Op.SENSOR, 0, bytes((flags,)) + bytes(18)))
    with pytest.raises(TMC2209MotorProtocolError):
        response_values(Frame(Op.SENSOR, 0, bytes(18)))


def test_sensor_golden_packet_and_wire_limits():
    payload = encode_sensor_payload(7, 42, 7, (-1234, 2345, -9001, 1234, -2345, 3456))
    assert payload.hex().upper() == "070000002A0007FB2E0929DCD704D2F6D70D80"
    assert encode_frame(Op.SENSOR, 42, payload) == "#13302A070000002A0007FB2E0929DCD704D2F6D70D802CBE\n"
    assert len(encode_frame(Op.SENSOR, 42, payload)) == 50
    assert response_values(Frame(Op.SENSOR, 0, bytes(19))) == {"sensor_flags": "0"}


def test_multi_point_calibration_persists_stops_reloads_and_resets(sensor_stack, tmp_path):
    storage = tmp_path / "sensor.json"
    service = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    assert service.status()["status"] == "uncalibrated"
    with pytest.raises(ValueError, match="site"):
        service.start_calibration()
    service.set_site(SITE.latitude_deg, SITE.longitude_deg)
    service.start_calibration()
    for alt, az in ((35, 0), (45, 10), (55, 20)):
        assert add_pose(service, sensor_stack.orientation_sensor, alt, az)["sample_added"]
    assert service.status()["calibration"]["point_count"] == 3
    service.stop_calibration()
    snapshot = storage.read_bytes()
    current = service.status()
    assert current["status"] == "ready"
    assert current["altaz"]["alt_deg"] == pytest.approx(55, abs=0.02)
    assert current["altaz"]["az_deg"] == pytest.approx(20, abs=0.02)
    solve = altaz_to_radec(55, 20, TIME, SITE)
    assert service.record_sync(solve.ra_hours, solve.dec_deg)["status"] == "verified"
    assert (
        json.loads(storage.read_bytes())["points"] == json.loads(snapshot)["points"]
    )  # Verification must never retrain.

    loaded = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    assert loaded.status()["calibration"]["point_count"] == 3
    assert loaded.status()["calibration"]["mode"] is False
    assert loaded.status()["calibration"]["latest_calibration"]["target"]["alt_deg"] == pytest.approx(55)
    loaded.reset_calibration()
    assert loaded.status()["status"] == "uncalibrated"
    assert PointingService(sensor_stack.dec_motor, storage).status()["calibration"]["point_count"] == 0


def test_same_raw_sample_is_not_counted_twice(sensor_stack):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    assert add_pose(service, sensor_stack.orientation_sensor, 45, 0)["sample_added"]
    eq = altaz_to_radec(45, 0, TIME, SITE)
    result = service.record_sync(eq.ra_hours, eq.dec_deg)
    assert result["status"] == "duplicate_measurement"
    assert service.status()["calibration"]["point_count"] == 1


def test_coincident_noisy_solves_average_as_vectors_at_azimuth_wrap(sensor_stack):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    for solve_az in (359.8, 0.2):
        sensor_stack.orientation_sensor.set_pose(45, 0)
        eq = altaz_to_radec(45, solve_az, TIME, SITE)
        service.record_sync(eq.ra_hours, eq.dec_deg)
    service.stop_calibration()
    current = service.status()
    assert current["calibration"]["point_count"] == 2
    assert abs((current["altaz"]["az_deg"] + 180) % 360 - 180) < 0.02


def test_arbitrary_mount_and_distorted_magnetic_field_are_corrected_locally(sensor_stack):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    bias, scale = Vec3(8, -4, 3), Vec3(1.2, 0.8, 1.1)
    for alt, az in ((40, 0), (40, 10), (50, 0), (50, 10)):
        add_pose(service, sensor_stack.orientation_sensor, alt, az, mounting=63, bias=bias, scale=scale)
    service.stop_calibration()
    sensor_stack.orientation_sensor.set_pose(45, 5, mounting_deg=63, magnetic_bias=bias, magnetic_scale=scale)
    current = service.status()
    assert current["status"] == "ready"
    predicted = altaz_to_enu(current["altaz"]["alt_deg"], current["altaz"]["az_deg"])
    assert angular_distance(predicted, altaz_to_enu(45, 5)) < 1


def test_held_out_plate_solve_detects_remount_without_overwriting_calibration(sensor_stack, tmp_path):
    storage = tmp_path / "sensor.json"
    service = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    add_pose(service, sensor_stack.orientation_sensor, 45, 0)
    service.stop_calibration()
    saved = storage.read_bytes()
    sensor_stack.orientation_sensor.set_pose(45, 0, mounting_deg=27)
    eq = altaz_to_radec(45, 0, TIME, SITE)
    result = service.record_sync(eq.ra_hours, eq.dec_deg)
    assert result["status"] == "calibration_drift"
    assert result["residual_deg"] > 9
    current = service.status()
    assert current["status"] == "calibration_drift"
    assert current["calibration"]["drift"] is True
    assert current["altaz"] is None and current["equatorial"] is None
    assert json.loads(saved)["points"] == json.loads(storage.read_bytes())["points"]
    assert current["calibration"]["point_count"] == 1
    restarted = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    assert restarted.status()["status"] == "calibration_drift"
    assert restarted.status()["calibration"]["latest_sync"]["residual_deg"] > 9


def test_outside_calibrated_area_is_unknown_and_cannot_verify(sensor_stack):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    add_pose(service, sensor_stack.orientation_sensor, 45, 0)
    service.stop_calibration()
    sensor_stack.orientation_sensor.set_pose(45, 180)
    current = service.status()
    assert current["status"] == "out_of_coverage"
    assert current["equatorial"] is None
    eq = altaz_to_radec(45, 180, TIME, SITE)
    assert service.record_sync(eq.ra_hours, eq.dec_deg)["residual_deg"] is None


@pytest.mark.parametrize(
    "reading, expected",
    [
        (SensorReading(SensorState.NO_DATA), "no_data"),
        (SensorReading(SensorState.AVAILABLE, 1, 2500, Vec3(0, 0, -9.8), Vec3(0, 20, -40)), "stale_data"),
        (SensorReading(SensorState.AVAILABLE, 1, 0, Vec3(0, 0, -9.8)), "heading_unavailable"),
        (SensorReading(SensorState.AVAILABLE, 1, 0, Vec3(0, 0, -1), Vec3(0, 20, -40)), "invalid_data"),
        (SensorReading(SensorState.AVAILABLE, 1, 0, Vec3(0, 0, -9.8), Vec3(0, 0, -40)), "invalid_data"),
    ],
)
def test_bad_or_partial_measurements_do_not_calibrate(sensor_stack, reading, expected):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    sensor_stack.orientation_sensor.set_raw(reading, frozen=True)
    status = service.status()
    assert status["status"] == expected
    assert status["altaz"] is None and status["equatorial"] is None
    result = service.record_sync(12, 20)
    assert result["status"] == expected and result["sample_added"] is False
    assert service.status()["calibration"]["point_count"] == 0


def test_measurement_pairing_uses_exposure_time_and_rejects_wrong_timestamp(sensor_stack):
    current_time = [TIME]
    service = PointingService(sensor_stack.dec_motor, now=lambda: current_time[0])
    service.set_site(43, 77)
    service.start_calibration()
    sample = service.status()["sensor"]
    eq = altaz_to_radec(45, 0, TIME, SITE)
    sensor_stack.orientation_sensor.set_pose(60, 90)
    current_time[0] += timedelta(minutes=3)
    with pytest.raises(ValueError, match="match"):
        service.record_sync(eq.ra_hours, eq.dec_deg, timestamp=current_time[0], measurement_id=sample["measurement_id"])
    result = service.record_sync(eq.ra_hours, eq.dec_deg, timestamp=TIME, measurement_id=sample["measurement_id"])
    assert result["sample_added"]
    assert result["timestamp"] == TIME.isoformat()


@pytest.mark.parametrize(
    "latitude, longitude, altitude, azimuth",
    [
        (43, 77, 45, 0),
        (0, 0, 30, 90),
        (-33, -70, 20, 359),
        (90, 20, 60, 240),
    ],
)
def test_horizontal_equatorial_round_trip_and_sidereal_time(latitude, longitude, altitude, azimuth):
    site = ObserverSite(latitude, longitude)
    eq = altaz_to_radec(altitude, azimuth, TIME, site)
    horizontal = radec_to_altaz(eq.ra_hours, eq.dec_deg, TIME, site)
    assert angular_distance(altaz_to_enu(altitude, azimuth), altaz_to_enu(horizontal.alt_deg, horizontal.az_deg)) < 1e-5
    later = altaz_to_radec(altitude, azimuth, TIME + timedelta(hours=1), site)
    assert (later.ra_hours - eq.ra_hours) % 24 == pytest.approx(1.002738, abs=1e-5)
    with pytest.raises(ValueError, match="timezone"):
        altaz_to_radec(altitude, azimuth, TIME.replace(tzinfo=None), site)


def test_failed_disk_commit_preserves_existing_profile(sensor_stack, tmp_path, monkeypatch):
    import pointing.service as module

    storage = tmp_path / "sensor.json"
    service = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    add_pose(service, sensor_stack.orientation_sensor, 45, 0)
    saved = storage.read_bytes()
    monkeypatch.setattr(module.os, "replace", lambda *_: (_ for _ in ()).throw(OSError("disk failure")))
    with pytest.raises(OSError):
        service.reset_calibration()
    assert service.status()["calibration"]["point_count"] == 1
    assert storage.read_bytes() == saved
    assert list(tmp_path.iterdir()) == [storage]


def test_corrupt_storage_never_activates_calibration(sensor_stack, tmp_path):
    storage = tmp_path / "sensor.json"
    storage.write_text('{"schema": 999, "points": []}')
    service = PointingService(sensor_stack.dec_motor, storage)
    assert service.status()["status"] == "uncalibrated"
    assert service.status()["calibration"]["storage_error"] is not None


def test_concurrent_reset_cannot_be_overwritten_by_a_prepared_sync(sensor_stack, tmp_path, monkeypatch):
    import pointing.service as module

    storage = tmp_path / "sensor.json"
    service = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    original = module.os.fsync
    blocked, resume = threading.Event(), threading.Event()
    errors = []

    def paused_fsync(fd):
        original(fd)
        if threading.current_thread().name == "calibration_writer":
            blocked.set()
            assert resume.wait(5)

    monkeypatch.setattr(module.os, "fsync", paused_fsync)

    def sync():
        try:
            add_pose(service, sensor_stack.orientation_sensor, 45, 0)
        except ValueError as error:
            errors.append(str(error))

    writer = threading.Thread(target=sync, name="calibration_writer")
    writer.start()
    assert blocked.wait(5)
    try:
        service.reset_calibration()
    finally:
        resume.set()
        writer.join(5)
    assert not writer.is_alive()
    assert errors == ["calibration changed concurrently; retry"]
    assert service.status()["calibration"]["point_count"] == 0
    assert json.loads(storage.read_text())["points"] == []


def test_live_http_and_lx200_sync_share_the_calibration_service(tmp_path):
    stack = build_sim_stack(RealtimeClock(), calibration_storage=tmp_path / "calibration.json")
    stack.sky_lx200.connect()
    server = AgentAPIServer(AgentAPI(stack.sky_lx200, stack.pointing, stack.orientation_sensor), port=0)
    server.start()
    host, port = server.address

    def request(path, data=None):
        body = None if data is None else json.dumps(data).encode()
        with urlopen(
            Request(f"http://{host}:{port}{path}", data=body, headers={"Content-Type": "application/json"}), timeout=5
        ) as response:
            return json.load(response)

    try:
        with urlopen(f"http://{host}:{port}/", timeout=5) as response:
            page = response.read().decode()
            assert "Зафиксировать измерение" in page
            assert 'id="telescope-model"' in page and '/model.mjs' in page
        with urlopen(f"http://{host}:{port}/model.mjs", timeout=5) as response:
            assert response.headers.get_content_type() == "text/javascript"
            assert "export class TelescopeModel" in response.read().decode()
        with urlopen(f"http://{host}:{port}/telemetry.mjs", timeout=5) as response:
            assert response.headers.get_content_type() == "text/javascript"
            assert "export class MountPanel" in response.read().decode()
        assert request("/v1/status")["status"] == "uncalibrated"
        request("/v1/site", {"latitude_deg": 43, "longitude_deg": 77})
        request("/v1/calibration/start", {})
        request("/v1/sim/sensor", {"alt_deg": 45, "az_deg": 0})
        captured = request("/v1/sensor")
        eq = altaz_to_radec(45, 0, datetime.fromisoformat(captured["timestamp"]), SITE)
        result = request(
            "/v1/sync", {"ra_hours": eq.ra_hours, "dec_deg": eq.dec_deg, "measurement_id": captured["measurement_id"]}
        )
        assert result["sample_added"]
        request("/v1/sim/sensor", {"alt_deg": 50, "az_deg": 5})
        captured = request("/v1/sensor")
        eq = altaz_to_radec(50, 5, datetime.fromisoformat(captured["timestamp"]), SITE)
        raw_before = stack.orientation_sensor.read_orientation_sensor().gravity
        stack.sky_lx200.handle(f"Sr{Ha(eq.ra_hours * 3600)}")
        stack.sky_lx200.handle(f"Sd{Dec(eq.dec_deg * 3600)}")
        assert stack.sky_lx200.handle("CM") == "OK"
        assert stack.orientation_sensor.read_orientation_sensor().gravity == raw_before
        assert request("/v1/calibration")["point_count"] == 2
        revision = request("/v1/status")["calibration"]["revision"]
        assert request("/v1/status")["calibration"]["revision"] == revision
        request("/v1/calibration/stop", {})
        status = request("/v1/status")
        assert status["status"] == "ready"
        assert status["calibration"]["revision"] > revision
        with pytest.raises(HTTPError) as error:
            request("/v1/sync", {"ra_hours": True, "dec_deg": 0})
        assert error.value.code == 400
        request("/v1/calibration/reset", {})
        assert request("/v1/status")["status"] == "uncalibrated"
    finally:
        server.stop()
        stack.sky_lx200.stop_all()
        stack.sky_lx200.stop()


def test_api_has_no_simulation_controls_for_real_mode(sensor_stack):
    api = AgentAPI(sensor_stack.sky_lx200, sensor_stack.pointing)
    assert api.handle_request("POST", "/v1/sim/sensor", {"alt_deg": 45, "az_deg": 0})[0] == 404


def test_checksum_detects_well_formed_but_altered_calibration(sensor_stack, tmp_path):
    storage = tmp_path / "sensor.json"
    service = PointingService(sensor_stack.dec_motor, storage, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    add_pose(service, sensor_stack.orientation_sensor, 45, 0)
    document = json.loads(storage.read_text())
    document["points"][0]["ra_hours"] += 0.1
    storage.write_text(json.dumps(document))
    restarted = PointingService(sensor_stack.dec_motor, storage)
    assert restarted.status()["calibration"]["point_count"] == 0
    assert "checksum" in restarted.status()["calibration"]["storage_error"]


def test_contradictory_points_and_zero_vectors_are_not_trusted(sensor_stack):
    service = PointingService(sensor_stack.dec_motor, now=lambda: TIME)
    service.set_site(43, 77)
    service.start_calibration()
    for altitude in (10, 80):
        sensor_stack.orientation_sensor.set_pose(45, 0)
        eq = altaz_to_radec(altitude, 0, TIME, SITE)
        service.record_sync(eq.ra_hours, eq.dec_deg)
    service.stop_calibration()
    status = service.status()
    assert status["status"] == "calibration_inconsistent"
    assert status["altaz"] is None
    sensor_stack.orientation_sensor.set_raw(SensorReading(SensorState.AVAILABLE, 0, 0, Vec3(0, 0, 0), Vec3(0, 20, -40)))
    assert service.status()["status"] == "invalid_data"


def test_normal_lx200_sync_survives_absent_sensor():
    stack = build_sim_stack(RealtimeClock(), dec_config={"orientation_sensor": None})
    stack.sky_lx200.connect()
    try:
        stack.sky_lx200.handle("Sr12:00:00")
        stack.sky_lx200.handle("Sd+40*00:00")
        assert stack.sky_lx200.handle("CM") == "OK"
        status = stack.pointing.status()
        assert status["status"] == "device_not_found"
        assert status["calibration"]["point_count"] == 0
        assert status["calibration"]["latest_sync"]["ra_hours"] == 12
    finally:
        stack.sky_lx200.stop_all()
        stack.sky_lx200.stop()


@pytest.mark.parametrize("dialect", [_Dialect.FRAMED, _Dialect.LEGACY])
def test_mpu_qmc_disconnect_partial_stale_transport_failure_and_recovery(sensor_stack, dialect, tmp_path, monkeypatch):
    sensor_stack.dec_motor._dialect = dialect
    sensor = sensor_stack.orientation_sensor
    service = PointingService(sensor_stack.dec_motor, tmp_path / "sensor.json", now=lambda: TIME)
    api = AgentAPI(sensor_stack.sky_lx200, service)
    service.set_site(43, 77)
    service.start_calibration()
    add_pose(service, sensor, 45, 0)
    service.stop_calibration()
    original = service.status()
    assert original["status"] == "ready"
    points = original["calibration"]["point_count"]
    reading = sensor.read_orientation_sensor()

    for raw, expected, channels in (
        (SensorReading(SensorState.DEVICE_NOT_FOUND), "device_not_found", {"gravity": "device_not_found", "magnetic": "device_not_found"}),
        (SensorReading(SensorState.NO_DATA), "no_data", {"gravity": "no_data", "magnetic": "no_data"}),
        (SensorReading(SensorState.AVAILABLE, 0, 0, reading.gravity), "heading_unavailable", {"gravity": "available", "magnetic": "unavailable"}),
        (SensorReading(SensorState.AVAILABLE, 0, 2500, reading.gravity, reading.magnetic), "stale_data", {"gravity": "stale_data", "magnetic": "stale_data"}),
        (SensorReading(SensorState.AVAILABLE, 0, 0, Vec3(0, 0, -1), reading.magnetic), "invalid_data", {"gravity": "invalid_data", "magnetic": "available"}),
        (SensorReading(SensorState.AVAILABLE, 0, 0, Vec3(0, 0, -9.807), Vec3(0, 0, -40)), "invalid_data", {"gravity": "available", "magnetic": "invalid_data"}),
    ):
        sensor.set_raw(raw)
        code, state = api.handle_request("GET", "/v1/status", {})
        assert code == 200 and state["status"] == expected
        assert state["sensor"]["data_status"] == expected
        assert state["sensor"]["channels"] == channels
        assert state["altaz"] is None and state["equatorial"] is None
        assert state["calibration"]["point_count"] == points
        code, raw_status = api.handle_request("GET", "/v1/sensor", {})
        assert code == 200 and raw_status["data_status"] == expected
        assert raw_status["channels"] == channels
        if raw.state is not SensorState.AVAILABLE:
            assert raw_status["gravity_m_s2"] is None and raw_status["magnetic_uT"] is None
        # The motion protocol stays live when orientation is unavailable.
        sensor_stack.dec_motor.status()
        service.start_calibration()
        result = service.record_sync(0, 0)
        assert result["status"] == expected and not result["sample_added"]
        assert service.status()["calibration"]["point_count"] == points
        service.stop_calibration()

    # An actual wire error is a status, never a cached coordinate or zero vector.
    with monkeypatch.context() as patch:
        patch.setattr(sensor_stack.dec_motor, "_exchange", lambda *_: (_ for _ in ()).throw(TMC2209MotorProtocolError("truncated sensor response")))
        code, raw_status = api.handle_request("GET", "/v1/sensor", {})
        assert code == 200 and raw_status["data_status"] == "transport_error"
        assert raw_status["channels"] == {"gravity": "transport_error", "magnetic": "transport_error"}
        assert raw_status["gravity_m_s2"] is None and raw_status["magnetic_uT"] is None

    sensor.set_raw(reading)
    recovered = service.status()
    assert recovered["status"] == "ready"
    assert recovered["sensor"]["channels"] == {"gravity": "available", "magnetic": "available"}
    assert recovered["calibration"]["point_count"] == points
    assert recovered["altaz"] == original["altaz"]
    assert PointingService(sensor_stack.dec_motor, tmp_path / "sensor.json", now=lambda: TIME).status()["status"] == "ready"


@pytest.mark.parametrize("field,value", [
    ("sample", "-1"), ("sample", "4294967296"),
    ("age_ms", "-1"), ("age_ms", "65536"),
    ("gx", "32768"), ("gx", "-32769"),
    ("my", "32768"), ("my", "-32769"),
])
def test_legacy_sensor_rejects_values_outside_binary_wire_bounds(sensor_stack, monkeypatch, field, value):
    from tmc2209.protocol import Response

    values = {"sensor_flags": "7", "sample": "42", "age_ms": "0", "gx": "0", "gy": "0", "gz": "-9807", "mx": "0", "my": "2000", "mz": "-4000"}
    values[field] = value
    monkeypatch.setattr(sensor_stack.dec_motor, "_exchange", lambda *_: Response(True, values, None))
    with pytest.raises(TMC2209MotorProtocolError, match="invalid raw sensor response"):
        sensor_stack.dec_motor.read_orientation_sensor()
    state = PointingService(sensor_stack.dec_motor, now=lambda: TIME).status()
    assert state["status"] == "transport_error" and state["altaz"] is None
