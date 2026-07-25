"""The RA conformance suite against the simulator.

This is the same case list, executed by the same runner, that
``src/tests/hw/test_ra_conformance_hw.py`` points at the live board. If the
simulator and the board ever disagree, one of the two runs goes red on the same
named case — which is the whole reason this file exists (RA_REWRITE_PLAN.md §3
Э2): before it, the simulator had its own tests and the board had others, and
they drifted for half a year without a single failure.

Time is virtual: a case that waits two seconds for a braking ramp costs nothing.
"""

import pytest

from ra_conformance import (
    CASES,
    SIMULATOR_SAFETY,
    Case,
    HardwareWire,
    Safety,
    SerialWire,
    Status,
    UnsafeOnHardware,
    is_allowed_on_hardware,
    render_markdown,
    run_case,
    run_cases,
    summarize,
)
from sim import Clock, SimSerialLine, SkyWatcherSim

# The read timeout the protocol document ran with (§1). It matters here too:
# every "the board stays silent" case burns exactly this much virtual time.
TIMEOUT_S = 0.5


def _fresh_wire() -> tuple[Clock, SkyWatcherSim, SerialWire]:
    """A board straight out of power-on, as §12.1 describes it."""
    clock = Clock()
    device = SkyWatcherSim(clock)
    line = SimSerialLine(device, clock, port="sim://ra", timeout_s=TIMEOUT_S, name="conformance", terminator="\r")
    line.connect()
    return clock, device, SerialWire(line, clock)


# Claims of RA_PROTOCOL.md that the simulator does not model, kept as cases on
# purpose and marked xfail(strict) so they stay visible in every run instead of
# being quietly dropped from the list. An xpass here means the gap was closed
# and this set has to shrink.
#
# `reply_time_is_milliseconds_not_zero` — §9: the board spends ~1 ms of its own
# on every command, and §12.3 leans on precisely that ("перезагрузка не
# удлиняет ответы") to tell a rebooted board from a dead link. The simulator
# answers synchronously inside the write, i.e. in exactly zero virtual
# milliseconds, so a driver that keys off response time cannot be tested here at
# all. RA_REWRITE_PLAN.md §3 Э1 listed the response-time model as in scope and
# it was not done; this is the case that says so out loud.
SIMULATOR_GAPS = frozenset({"reply_time_is_milliseconds_not_zero"})


@pytest.mark.parametrize("case", CASES, ids=[case.name for case in CASES])
def test_case_against_simulator(case: Case, request: pytest.FixtureRequest) -> None:
    """Every case, each on its own freshly booted simulated board.

    Cases are written to be self-contained and restoring, so a fresh board per
    case and one long hardware session must give the same verdicts. The next
    test checks the other half of that claim.
    """
    if case.name in SIMULATOR_GAPS:
        request.node.add_marker(pytest.mark.xfail(strict=True, reason="симулятор не моделирует это утверждение"))
    _, _, wire = _fresh_wire()
    result = run_case(wire, case)
    assert result.status is Status.MATCH, f"{case.name} ({case.section}): {result.detail}"


def test_whole_suite_in_one_session() -> None:
    """The list end to end on one board — the way hardware runs it.

    This is what proves the teardowns actually restore the board: without them
    the period, the direction or the position left behind by one case would fail
    the next one. Everything except the known gaps above has to match.
    """
    _, _, wire = _fresh_wire()
    results = run_cases(wire, CASES, SIMULATOR_SAFETY)
    diverged = {result.case.name for result in results if result.status is not Status.MATCH}
    assert diverged == SIMULATOR_GAPS, render_markdown(results, "Симулятор, один сеанс")
    assert summarize(results).matched == len(CASES) - len(SIMULATOR_GAPS)


def test_every_case_names_a_protocol_section() -> None:
    for case in CASES:
        assert case.section.startswith("§"), case.name
        assert case.note, case.name


def test_motion_cases_stop_the_axis() -> None:
    """No motion case may end with the axis still turning."""
    for case in CASES:
        if case.safety is not Safety.MOTION:
            continue
        assert b":K1\r" in case.teardown, case.name


class _RecordingLine:
    """A line that records instead of transmitting: what reached the port, and when."""

    def __init__(self) -> None:
        self.written: list[str | None] = []

    def query(self, payload: str | None, timeout: float | None = None) -> str:
        self.written.append(payload)
        return "=\r"


def test_hardware_wire_refuses_forbidden_case_before_writing() -> None:
    """The guard is mechanical: nothing reaches the port, not even the first byte."""
    line = _RecordingLine()
    wire = HardwareWire(line, Clock(), {Safety.READ, Safety.WRITE, Safety.MOTION, Safety.NEVER})
    with pytest.raises(UnsafeOnHardware):
        wire.exchange(b":W1000009\r", Safety.NEVER)
    assert line.written == []


def test_hardware_wire_refuses_unsafe_payload_even_under_a_safe_label() -> None:
    """A mislabelled case is caught by the payload whitelist, not by the label."""
    line = _RecordingLine()
    wire = HardwareWire(line, Clock(), {Safety.READ})
    with pytest.raises(UnsafeOnHardware):
        wire.exchange(b":R100\r", Safety.READ)
    assert line.written == []


def test_hardware_wire_refuses_motion_unless_asked() -> None:
    line = _RecordingLine()
    wire = HardwareWire(line, Clock(), {Safety.READ, Safety.WRITE})
    with pytest.raises(UnsafeOnHardware):
        wire.exchange(b":J1\r", Safety.MOTION)
    assert line.written == []


def test_never_cases_are_skipped_on_hardware() -> None:
    """Run the whole list through a hardware wire: nothing forbidden reaches it.

    The assertion is on what was written, not on what was labelled: every byte
    that made it to the port has to pass the same whitelist the wire enforces,
    and the destructive payloads must be absent from it entirely.
    """
    line = _RecordingLine()
    wire = HardwareWire(line, Clock(), {Safety.READ, Safety.WRITE, Safety.MOTION})
    results = run_cases(wire, CASES, {Safety.READ, Safety.WRITE, Safety.MOTION})

    skipped = {result.case.name for result in results if result.status is Status.SKIPPED}
    assert skipped == {case.name for case in CASES if case.safety is Safety.NEVER}

    written = [payload for payload in line.written if payload is not None]
    assert written
    for payload in written:
        for chunk in payload.encode("ascii").split(b"\r"):
            assert is_allowed_on_hardware(chunk), f"на порт ушло {chunk!r}"
    # No forbidden command letter on channel 1. `:` + 60 `A` (§8.2) is not one of
    # them: the board answers `!1` to it, which is precisely the claim it pins.
    forbidden = {f":{letter}1" for letter in "WNRQABOPTUVSLz"}
    for payload in written:
        assert payload[:3] not in forbidden, payload


def test_every_payload_of_a_hardware_case_is_whitelisted() -> None:
    """Static check of the same rule the wire enforces at runtime."""
    for case in CASES:
        if case.safety is Safety.NEVER:
            continue
        for payload in (*case.sent, *case.teardown):
            for chunk in payload.split(b"\r"):
                assert is_allowed_on_hardware(chunk), f"{case.name}: {chunk!r}"


def test_forbidden_commands_are_not_whitelisted() -> None:
    for payload in (b":W1000009", b":W10D8004", b":R100", b":N100", b":Q155AA", b":A100", b":z1", b":L1"):
        assert not is_allowed_on_hardware(payload)
