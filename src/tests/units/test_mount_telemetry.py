"""Monitor the production simulator; rates use explicit sampled observations."""

from copy import deepcopy
import json
from typing import Any

import pytest

from pointing.api import AgentAPI
from sky.telemetry import MountTelemetry
from tools.sim_stack import build_sim_stack
from utils.polar_align_lib import ObserverSite, radec_to_altaz
from datetime import datetime


class SampledMount:
    def __init__(self):
        self.data: dict[str, Any] = {
            "equatorial": {"ra_hours": 23.9999, "dec_deg": 20},
            "ra": {"available": True, "position_revision": 0, "observed_s": 10, "motor": {"position_native": 86399.64}},
            "dec": {"available": True, "position_revision": 0, "observed_s": 10, "motor": {"position_native": 72000}},
        }

    def monitor(self):
        return deepcopy(self.data)

    def advance(self, time, ra, dec):
        self.data["equatorial"] = {"ra_hours": ra, "dec_deg": dec}
        self.data["ra"]["observed_s"] = self.data["dec"]["observed_s"] = time
        self.data["ra"]["motor"]["position_native"] = ra * 3600
        self.data["dec"]["motor"]["position_native"] = dec * 3600


def test_native_rates_wrap_ra_and_keep_the_two_units_distinct():
    mount = SampledMount()
    telemetry = MountTelemetry(mount)
    assert telemetry.status()["ra"]["rates"]["mount_native_s"] is None
    mount.advance(11.2, .0001, 20.001)
    status = telemetry.status()
    assert status["ra"]["rates"]["mount_native_s"] == pytest.approx(.6)
    assert status["ra"]["rates"]["motor_native_s"] == pytest.approx(.6)
    assert status["dec"]["rates"]["mount_native_s"] == pytest.approx(3)
    assert status["dec"]["rates"]["motor_native_s"] == pytest.approx(3)
    assert status["ra"]["rates"]["interval_s"] == pytest.approx(1.2)


@pytest.mark.parametrize("reason", ["sync", "gap", "axis_lost", "old_observation", "old_revision"])
def test_discontinuities_are_unknown_instead_of_motion_or_zero(reason):
    mount = SampledMount()
    telemetry = MountTelemetry(mount)
    if reason == "old_revision":
        mount.data["ra"]["position_revision"] = 1
    telemetry.status()
    mount.advance(11.2, 6, 40)
    if reason == "sync":
        mount.data["dec"]["position_revision"] += 1
    elif reason == "gap":
        mount.advance(20, 6, 40)
    elif reason == "axis_lost":
        mount.data["ra"]["available"] = False
        mount.data["equatorial"] = None
    elif reason == "old_observation":
        mount.advance(9, 6, 40)
    else:
        mount.data["ra"]["position_revision"] = 0
    status = telemetry.status()
    assert status["ra"]["rates"]["mount_native_s"] is None
    assert status["ra"]["rates"]["motor_native_s"] is None
    assert status["dec"]["rates"]["mount_native_s"] is None


def test_site_is_required_for_the_horizontal_mount_model():
    telemetry = MountTelemetry(SampledMount())
    assert telemetry.status()["altaz"] is None
    site = ObserverSite(43, 77)
    status = telemetry.status(site)
    expected = radec_to_altaz(23.9999, 20, datetime.fromisoformat(status["timestamp"]), site)
    assert status["altaz"]["az_deg"] == pytest.approx(expected.az_deg)
    assert status["altaz"]["alt_deg"] == pytest.approx(expected.alt_deg)


@pytest.fixture
def monitor_stack():
    stack = build_sim_stack()
    # Read through the actual drivers, without starting asynchronous motion loops.
    stack.ra_motor.connect()
    stack.dec_motor.connect()
    stack.axis_ra._connected = stack.axis_dec._connected = True
    try:
        yield stack
    finally:
        stack.axis_ra._connected = stack.axis_dec._connected = False
        stack.ra_motor.disconnect()
        stack.dec_motor.disconnect()


def test_api_exposes_axes_boards_polar_and_lx200_without_a_sensor_solution(monitor_stack):
    stack = monitor_stack
    stack.sky_lx200.handle("MS")
    api = AgentAPI(stack.sky_lx200, stack.pointing, stack.orientation_sensor)
    code, status = api.handle_request("GET", "/v1/status", {})
    assert code == 200 and status["status"] == "uncalibrated"
    mount = status["mount"]
    assert mount["ra"]["available"] and mount["dec"]["available"]
    for axis in (mount["ra"], mount["dec"]):
        assert axis["motor"]["steps"] is not None
        assert axis["motor"]["direction"] in ("stop", "forward", "backward")
        assert axis["queue_size"] >= 0
        assert axis["rates"]["mount_native_s"] is None
    protocol = mount["ra"]["motor"]["protocol"]
    assert all(key in protocol for key in ("speed_mode", "highspeed_ratio", "initialized", "battery_v", "usb_v"))
    for name in ("ra", "dec"):
        assert all(key in mount["polar"][name] for key in ("average_native", "samples", "pulse_age_s", "external", "stable", "axis_external", "current_native", "eps_native"))
    assert mount["lx200"]["stats"][0]["command"] == "SLEW"
    assert mount["lx200"]["stats"][0]["count"] == 1
    assert mount["altaz"] is None
    json.dumps(status, allow_nan=False)


def test_one_failed_motor_does_not_hide_the_other_or_look_stopped(monitor_stack, monkeypatch):
    def fault():
        raise OSError("test board disconnected")

    monkeypatch.setattr(monitor_stack.dec_motor, "status", fault)
    api = AgentAPI(monitor_stack.sky_lx200, monitor_stack.pointing)
    code, mount = api.handle_request("GET", "/v1/mount", {})
    assert code == 200
    assert mount["ra"]["available"] and mount["ra"]["motor"] is not None
    assert not mount["dec"]["available"] and mount["dec"]["motor"] is None
    assert mount["dec"]["rates"]["motor_native_s"] is None
    assert "test board disconnected" in mount["dec"]["error"]
    assert mount["equatorial"] is None
    json.dumps(mount, allow_nan=False)


def test_unconnected_monitor_never_queries_boards_or_exposes_fake_positions(monkeypatch):
    stack = build_sim_stack()
    monkeypatch.setattr(stack.ra_motor, "status", lambda: pytest.fail("disconnected board queried"))
    code, mount = AgentAPI(stack.sky_lx200, stack.pointing).handle_request("GET", "/v1/mount", {})
    assert code == 200
    assert mount["equatorial"] is None
    assert mount["ra"]["motor"] is None and mount["dec"]["motor"] is None


def test_goto_has_direction_target_and_remaining_distance(monitor_stack):
    from sky.axis import AxisMotionMode
    from sky.physics import Ha, SkyDirection

    axis = monitor_stack.axis_ra
    axis._mode = AxisMotionMode.GOTO
    axis._goto_target = Ha(600)
    axis._goto_direction = SkyDirection.EAST
    status = axis.monitor()
    assert status["movement"]["direction"] == "east"
    assert status["movement"]["target_native"] == 600
    assert status["movement"]["remaining_native"] == 600


def test_lx200_server_state_is_unknown_without_provider_and_explicit_when_present(monitor_stack):
    from lx200.base_server import LX200SimpleServer

    sky, pointing = monitor_stack.sky_lx200, monitor_stack.pointing
    _, mount = AgentAPI(sky, pointing).handle_request("GET", "/v1/mount", {})
    assert mount["lx200"]["server"] is None
    server = LX200SimpleServer(sky, host="127.0.0.1", port=7624)
    server.last_error = OSError("test bind failed")
    _, mount = AgentAPI(sky, pointing, lx200_server=server).handle_request("GET", "/v1/mount", {})
    assert not mount["lx200"]["server"]["running"]
    assert not mount["lx200"]["server"]["listening"]
    assert not mount["lx200"]["server"]["client_connected"]
    assert mount["lx200"]["server"]["host"] == "127.0.0.1"
    assert "test bind failed" in mount["lx200"]["server"]["error"]


def test_reconnection_marks_a_discontinuity_even_if_polling_missed_the_offline_interval(monitor_stack):
    axis = monitor_stack.axis_ra
    revision = axis.monitor()["position_revision"]
    axis.disconnect()
    try:
        axis.connect()
        assert axis.monitor()["position_revision"] > revision
    finally:
        axis.disconnect()
