"""HTTP controls drive the production simulator without modifying sensor calibration."""

import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from pointing.api import AgentAPI, AgentAPIServer
from sim.realtime import RealtimeClock
from sky.axis import AxisMotionMode
from tools.sim_stack import build_sim_stack


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


def test_http_manual_axes_stop_tracking_and_goto(tmp_path):
    stack = build_sim_stack(RealtimeClock(), calibration_storage=tmp_path / "cal.json")
    stack.sky_lx200.connect()
    server = AgentAPIServer(AgentAPI(stack.sky_lx200, stack.pointing, stack.orientation_sensor), port=0)
    server.start()
    host, port = server.address

    def command(**payload):
        with urlopen(Request(f"http://{host}:{port}/v1/mount/control", data=json.dumps(payload).encode(),
                             headers={"Content-Type": "application/json"}), timeout=5) as response:
            return json.load(response)

    try:
        wait_for(lambda: all(item.is_ready for item in stack.combiner.motors_readiness()))
        raw = stack.orientation_sensor.read_orientation_sensor().gravity
        for direction, axis in (("north", stack.axis_dec), ("south", stack.axis_dec), ("east", stack.axis_ra), ("west", stack.axis_ra)):
            before = axis.monitor()["motor"]["position_native"]
            assert command(action="move", direction=direction, speed="center")["accepted"]
            wait_for(lambda axis=axis: axis.mode() == AxisMotionMode.SLEW)
            wait_for(lambda axis=axis, before=before: axis.monitor()["motor"]["position_native"] != before)
            assert command(action="halt")["accepted"]
            wait_for(lambda: stack.axis_ra.mode() == AxisMotionMode.TRACK and stack.axis_dec.monitor()["motor"]["direction"] == "stop")
        command(action="stop")
        wait_for(lambda: all(axis.monitor()["motor"]["direction"] == "stop" and axis.monitor()["sky_speed_native"] == 0 for axis in (stack.axis_ra, stack.axis_dec)))
        command(action="track")
        wait_for(lambda: stack.axis_ra.mode() == AxisMotionMode.TRACK)
        for direction, ra_direction, dec_direction in (("northwest", "west", "north"), ("northeast", "east", "north"), ("southwest", "west", "south"), ("southeast", "east", "south")):
            before = tuple(axis.monitor()["motor"]["position_native"] for axis in (stack.axis_ra, stack.axis_dec))
            command(action="move", direction=direction, speed="center")
            wait_for(lambda ra_direction=ra_direction, dec_direction=dec_direction: stack.axis_ra.monitor()["movement"]["direction"] == ra_direction and stack.axis_dec.monitor()["movement"]["direction"] == dec_direction)
            wait_for(lambda before=before: stack.axis_ra.monitor()["motor"]["position_native"] != before[0] and stack.axis_dec.monitor()["motor"]["position_native"] != before[1])
            command(action="halt")
            wait_for(lambda: stack.axis_ra.mode() == stack.axis_dec.mode() == AxisMotionMode.TRACK)
        command(action="speed", speed="find")
        command(action="goto", ra_hours=2.5, dec_deg=20)
        wait_for(lambda: stack.axis_ra.is_moving_to() and stack.axis_dec.is_moving_to())
        assert stack.axis_ra.monitor()["movement"]["target_native"] == pytest.approx(2.5 * 3600)
        assert stack.axis_dec.monitor()["movement"]["target_native"] == pytest.approx(20 * 3600)
        command(action="halt")
        wait_for(lambda: not stack.axis_ra.is_moving_to() and not stack.axis_dec.is_moving_to())
        assert stack.orientation_sensor.read_orientation_sensor().gravity == raw
        assert stack.pointing.status()["calibration"]["point_count"] == 0
        with pytest.raises(HTTPError) as error:
            command(action="goto", ra_hours=24, dec_deg=20)
        assert error.value.code == 400
    finally:
        server.stop()
        stack.sky_lx200.stop_all()
        stack.sky_lx200.stop()


@pytest.mark.parametrize("payload", [
    {}, {"action": "unknown"}, {"action": "move", "direction": "up"},
    {"action": "move", "direction": "north", "speed": "turbo"},
    {"action": "goto", "ra_hours": True, "dec_deg": 0},
    {"action": "goto", "ra_hours": 24, "dec_deg": 0},
    {"action": "goto", "ra_hours": 0, "dec_deg": 91},
    {"action": "goto", "ra_hours": float("nan"), "dec_deg": 0},
])
def test_invalid_controls_do_not_enqueue_motion(payload):
    stack = build_sim_stack()
    api = AgentAPI(stack.sky_lx200, stack.pointing)
    assert api.handle_request("POST", "/v1/mount/control", payload)[0] == 400
    assert stack.axis_ra.monitor()["queue_size"] == stack.axis_dec.monitor()["queue_size"] == 0


def test_offline_controls_are_rejected_but_stop_remains_available():
    stack = build_sim_stack()
    api = AgentAPI(stack.sky_lx200, stack.pointing)
    assert api.handle_request("POST", "/v1/mount/control", {"action": "move", "direction": "north"})[0] == 503
    assert api.handle_request("POST", "/v1/mount/control", {"action": "goto", "ra_hours": 1, "dec_deg": 2})[0] == 503
    assert api.handle_request("POST", "/v1/mount/control", {"action": "stop"})[0] == 200


@pytest.mark.parametrize("missing", ["ra", "dec"])
def test_diagonal_is_rejected_before_either_axis_moves(monkeypatch, missing):
    stack = build_sim_stack()
    monkeypatch.setattr(stack.sky_lx200, "is_connected", lambda: True)
    monkeypatch.setattr(stack.sky_lx200, "monitor", lambda: {axis: {"available": axis != missing} for axis in ("ra", "dec")})
    api = AgentAPI(stack.sky_lx200, stack.pointing)
    assert api.handle_request("POST", "/v1/mount/control", {"action": "move", "direction": "northeast"})[0] == 503
    assert stack.axis_ra.monitor()["queue_size"] == stack.axis_dec.monitor()["queue_size"] == 0


def test_failed_stop_is_reported_to_operator(monkeypatch):
    stack = build_sim_stack()

    def fail_stop():
        raise OSError("serial unavailable")

    monkeypatch.setattr(stack.sky_lx200, "stop_all", fail_stop)
    api = AgentAPI(stack.sky_lx200, stack.pointing)
    assert api.handle_request("POST", "/v1/mount/control", {"action": "stop"}) == (503, {"error": "mount_transport_unavailable"})
