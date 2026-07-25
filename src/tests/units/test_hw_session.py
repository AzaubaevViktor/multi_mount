"""The recorded-session script (``src/tools/hw_session.py``) against the simulator.

Two things are asserted here, and they are not the same thing:

1. the artifacts — a trace that reads back, a manifest that is actually filled in
   — because a session run on hardware is a one-shot window and an empty or
   half-written artifact is only discovered afterwards;
2. the shutdown — that a step blowing up in the middle leaves *both* axes
   stopped and *both* lines closed. That is the only property in this file whose
   failure costs hardware.
"""

import json
import logging
from collections.abc import Iterator

import pytest

from serial_wrapper.recorder import JsonlRecorder, TraceKind, read_trace
from serial_wrapper.wrapper import SerialLineState
from sky.motor import MotionMode, MotorDirection
from tools import hw_session
from tools.hw_session import LineConfig, SessionConfig, Session, SessionSetupError, Step

# Spelled out rather than derived from `hw_session.SCENARIO`: a test that reads
# the scenario out of the module under test cannot notice a step disappearing
# from it.
EXPECTED_STEPS = [
    "connect",
    "read_board_config",
    "track_sidereal",
    "goto_ra_lowspeed_forward",
    "goto_ra_lowspeed_backward",
    "goto_ra_highspeed_forward",
    "goto_ra_highspeed_backward",
    "goto_dec_slow",
    "goto_dec_fast",
    "sync_while_tracking",
    "guide_pulses",
    "halt_and_resume_tracking",
    "poll_status_and_voltage",
    "dec_status_and_full_status",
]


class _Boom(RuntimeError):
    pass


@pytest.fixture(autouse=True)
def restore_logging() -> Iterator[None]:
    """``main()`` installs the project logging; put the root logger back after."""
    root = logging.getLogger()
    handlers = list(root.handlers)
    level = root.level
    try:
        yield
    finally:
        for handler in root.handlers:
            if handler not in handlers:
                handler.close()
        root.handlers[:] = handlers
        root.setLevel(level)


def _run_main(tmp_path, *extra: str) -> tuple[dict, list]:
    exit_code = hw_session.main(
        [
            "--sim",
            "--out-dir", str(tmp_path / "out"),
            "--logs-root", str(tmp_path / "logs"),
            "--session-id", "session",
            *extra,
        ]
    )
    assert exit_code == 0
    manifest = json.loads((tmp_path / "out" / "session.manifest.json").read_text(encoding="utf-8"))
    events = read_trace(tmp_path / "out" / "session.jsonl")
    return manifest, events


def _sim_config(tmp_path) -> SessionConfig:
    return SessionConfig(
        mode="sim",
        session_id="poison",
        out_dir=tmp_path,
        ra=LineConfig("ra", "sim://ra", 115200, 0.2, "\r"),
        dec=LineConfig("dec", "sim://dec", 115200, 2.0, "\n"),
    )


def _step(name: str) -> Step:
    return next(step for step in hw_session.SCENARIO if step.name == name)


def _start_dec_drift(session: Session) -> None:
    dec = session.dec_motor
    dec.set_motion_mode(MotionMode.RUN)
    dec.set_acceleration(1000)
    dec.set_speed(500)
    dec.set_direction(MotorDirection.FORWARD)
    dec.run()
    session.clock.sleep(1.0)


def _both_axes_moving(session: Session) -> None:
    assert session.ra_device is not None and session.dec_device is not None
    assert session.ra_device.running is True, "RA is not moving, the shutdown test would prove nothing"
    assert session.dec_device.running is True, "DEC is not moving, the shutdown test would prove nothing"


def _run_until_boom(tmp_path, steps, expected_error) -> Session:
    recorder = JsonlRecorder(tmp_path / "poison.jsonl")
    session = hw_session.open_session(_sim_config(tmp_path), recorder)
    hw_session.init_manifest(session)
    try:
        with pytest.raises(expected_error):
            hw_session.run_scenario(session, steps)
    finally:
        recorder.close()
    return session


# --------------------------------------------------------------------------- #
# artifacts
# --------------------------------------------------------------------------- #


def test_sim_run_walks_the_whole_scenario_and_leaves_a_readable_trace(tmp_path) -> None:
    manifest, events = _run_main(tmp_path)

    assert events, "the trace is empty"
    assert [step["name"] for step in manifest["steps"]] == EXPECTED_STEPS
    assert {step["status"] for step in manifest["steps"]} == {"ok"}
    assert manifest["error"] is None


def test_every_step_is_marked_in_the_trace_between_its_bytes(tmp_path) -> None:
    """Without the markers a reader cannot tell which step a frame belongs to."""
    _, events = _run_main(tmp_path)

    marks = [event for event in events if event.kind is TraceKind.MARK]
    begins = [event.detail["step"] for event in marks if event.detail["phase"] == "begin"]
    ends = [event.detail["step"] for event in marks if event.detail["phase"] == "end"]
    assert begins == [*EXPECTED_STEPS, "shutdown"]
    assert ends == [*EXPECTED_STEPS, "shutdown"]

    # Bytes really do sit between two markers, i.e. the marks segment the trace.
    kinds = [event.kind for event in events]
    first_mark = kinds.index(TraceKind.MARK)
    assert TraceKind.TX in kinds[first_mark:]
    assert TraceKind.RX in kinds[first_mark:]


def test_manifest_carries_what_the_boards_said_about_themselves(tmp_path) -> None:
    manifest, events = _run_main(tmp_path)

    ra_board = manifest["ra_board"]
    assert ra_board["cpr"] > 0
    assert ra_board["timer_freq"] > 0
    assert ra_board["highspeed_ratio"] > 0
    assert ra_board["mount_code"] is not None
    assert ra_board["board_version"].startswith("0x")
    assert ra_board["voltage_supported"] is True
    assert ra_board["power_v"] == pytest.approx(12.6)

    dec_board = manifest["dec_board"]
    assert dec_board["status"]["ok"] is True
    assert dec_board["status"]["values"]["phase"]
    assert dec_board["full_status"]["command"] == "full_status"
    # Absent on the simulator and on pre-TX-ring-fix firmware; the key must be
    # there and null, not missing, and must not abort the session.
    assert "tx_overflow" in dec_board
    assert dec_board["tx_overflow"] is None
    assert "tx_overflow" in manifest["dec_final"]

    # `full_status` really went out on the wire, twice: once with the board
    # config and once at the end of the session.
    dec_tx = [event.data for event in events if event.kind is TraceKind.TX and event.name == "dec"]
    assert dec_tx.count(b"full_status\n") == 2
    assert b"status\n" in dec_tx


def test_manifest_records_the_connection_and_the_code_revision(tmp_path) -> None:
    manifest, _ = _run_main(tmp_path)

    for axis in ("ra", "dec"):
        line = manifest["lines"][axis]
        assert line["port"]
        assert line["baud"] == 115200
        assert line["timeout_s"] > 0
    assert manifest["code"]["commit"], "no git revision: the trace cannot be tied to a version"


def test_baud_is_a_cli_knob_and_reaches_both_the_manifest_and_the_trace(tmp_path) -> None:
    """The 112500 vs 115200 question is answered by comparing two traces, so the
    baud each one was taken at has to be readable from the artifacts."""
    manifest, events = _run_main(tmp_path, "--ra-baud", "112500")

    assert manifest["lines"]["ra"]["baud"] == 112500
    assert manifest["lines"]["dec"]["baud"] == 115200
    opens = {event.name: event.detail["baud"] for event in events if event.kind is TraceKind.OPEN}
    assert opens["ra"] == 112500
    assert opens["dec"] == 115200


def test_scenario_covers_both_speed_modes_and_both_directions_on_both_axes(tmp_path) -> None:
    manifest, _ = _run_main(tmp_path)

    moves = {move["label"]: move for move in manifest["goto"]}
    assert {"ra_lowspeed_forward", "ra_lowspeed_backward", "ra_highspeed_forward", "ra_highspeed_backward"} <= set(moves)
    assert {"dec_slow_forward", "dec_slow_backward", "dec_fast_forward", "dec_fast_backward"} <= set(moves)
    assert "lowspeed" in moves["ra_lowspeed_forward"]["speed_mode"]
    assert "highspeed" in moves["ra_highspeed_forward"]["speed_mode"]
    for label, move in moves.items():
        travelled = move["end"]["steps"] - move["start"]["steps"]
        assert travelled != 0, f"{label} did not move the axis at all"
        assert (travelled > 0) is ("forward" in label), f"{label} moved the wrong way"

    guides = {pulse["label"] for pulse in manifest["guide"]}
    assert {"ra_west", "ra_east", "dec_north", "dec_south"} <= guides


def test_manifest_states_the_p6_and_halt_verdicts(tmp_path) -> None:
    manifest, _ = _run_main(tmp_path)

    assert manifest["checks"]["sync_resumed_tracking"] is True
    assert manifest["checks"]["halt_stops_axis"] is True
    assert manifest["checks"]["tracking_resumed_after_halt"] is True
    assert manifest["sync"]["after"]["motion_mode"] == "run"


# --------------------------------------------------------------------------- #
# shutdown: the part that costs hardware
# --------------------------------------------------------------------------- #


def test_a_failing_step_stops_both_axes_and_closes_both_lines(tmp_path) -> None:
    def boom(session: Session) -> None:
        _both_axes_moving(session)
        raise _Boom("scripted failure in the middle of the session")

    session = _run_until_boom(
        tmp_path,
        (_step("connect"), _step("track_sidereal"), Step("dec_drift", _start_dec_drift), Step("boom", boom)),
        _Boom,
    )

    assert session.ra_device.running is False
    assert session.dec_device.running is False
    assert session.dec_device.actual_sps == 0.0
    assert session.ra_line.state is SerialLineState.CLOSED
    assert session.dec_line.state is SerialLineState.CLOSED
    assert session.manifest["checks"]["ra_stopped"] is True
    assert session.manifest["checks"]["dec_stopped"] is True
    assert session.manifest["error"]["step"] == "boom"


def test_ctrl_c_in_the_middle_stops_both_axes_too(tmp_path) -> None:
    """KeyboardInterrupt is not an Exception; a bare `except Exception` here
    would hand back a mount that is still slewing."""

    def interrupt(session: Session) -> None:
        _both_axes_moving(session)
        raise KeyboardInterrupt

    session = _run_until_boom(
        tmp_path,
        (_step("connect"), _step("track_sidereal"), Step("dec_drift", _start_dec_drift), Step("interrupt", interrupt)),
        KeyboardInterrupt,
    )

    assert session.ra_device.running is False
    assert session.dec_device.running is False
    assert session.ra_line.state is SerialLineState.CLOSED
    assert session.dec_line.state is SerialLineState.CLOSED
    # An interrupted run must still say where it was interrupted, otherwise the
    # trace ends mid-step with nothing explaining why.
    assert session.manifest["error"] == {"step": "interrupt", "type": "KeyboardInterrupt", "message": ""}
    marks = [event for event in read_trace(tmp_path / "poison.jsonl") if event.kind is TraceKind.MARK]
    assert [event.detail["step"] for event in marks if event.detail["phase"] == "failed"] == ["interrupt"]


def test_an_axis_whose_port_was_already_closed_is_still_stopped(tmp_path) -> None:
    """What an I/O error leaves behind: `SerialLine` closes itself, and the axis
    keeps slewing with no open handle to it. The shutdown has to reopen."""

    def close_the_port(session: Session) -> None:
        _both_axes_moving(session)
        session.ra_line.close(reason="simulated_io_error")
        raise _Boom("port died mid-slew")

    session = _run_until_boom(
        tmp_path,
        (_step("connect"), _step("track_sidereal"), Step("dec_drift", _start_dec_drift), Step("yank", close_the_port)),
        _Boom,
    )

    assert session.ra_device.running is False, "RA kept slewing after its port was closed"
    assert session.dec_device.running is False


def test_the_failed_step_is_marked_in_the_trace(tmp_path) -> None:
    def boom(session: Session) -> None:
        raise _Boom("scripted")

    _run_until_boom(tmp_path, (_step("connect"), Step("boom", boom)), _Boom)

    marks = [event for event in read_trace(tmp_path / "poison.jsonl") if event.kind is TraceKind.MARK]
    failed = [event for event in marks if event.detail["phase"] == "failed"]
    assert [event.detail["step"] for event in failed] == ["boom"]
    assert failed[0].detail["error_type"] == "_Boom"
    assert [event.detail["phase"] for event in marks if event.detail["step"] == "shutdown"][-1] == "end"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def test_a_missing_device_names_the_pattern_that_was_searched(tmp_path) -> None:
    parser = hw_session.build_parser()
    args = parser.parse_args(["--ra-pattern", "no-such-adapter-42"])

    with pytest.raises(SessionSetupError, match="no-such-adapter-42"):
        hw_session.config_from_args(args)


def test_sim_mode_needs_no_device_at_all(tmp_path) -> None:
    args = hw_session.build_parser().parse_args(["--sim", "--ra-pattern", "no-such-adapter-42"])

    config = hw_session.config_from_args(args)

    assert config.mode == "sim"
    assert config.ra.port.startswith("sim://")
