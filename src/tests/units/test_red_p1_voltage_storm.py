"""П1: the RA driver must never ask the board for a supply voltage.

`get_power_v()` used to send `:fL#`, a command that cannot exist on this board:
`L` sits where the hex channel belongs (`:fL\\r` -> `!3`) and `#` terminates
nothing (`:fL#` -> silence), so the probe burned three 507 ms timeouts and a
WARNING per attempt — every 60 s, forever, on every dashboard tick.
RA_PROTOCOL.md §6 records the exhaustive search that found no other query.

The contract these tests pin is therefore not "some backoff exists" (a backoff
of 1 ms would satisfy that) but the real one: **after the first refusal not a
single request byte for the voltage ever leaves the driver**, no matter how
much time passes.
"""

import logging

import pytest

from sim import Clock, SimSerialLine, SkyWatcherSim
from skywatcher.motor import SkyWatcherMotor


class _RecordingSerial:
    """SerialLine stand-in that records every payload the driver sends."""

    terminator = b"\r"

    def __init__(self) -> None:
        self.query_calls: list[str] = []

    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        self.query_calls.append(payload or "")
        return ""

    def drop_buffers(self) -> None:
        pass

    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        return None


def test_voltage_is_never_asked_of_the_board_however_long_the_session_runs() -> None:
    """No poll, ever: not the first one, not one after hours of dashboard ticks."""
    clock = Clock()
    serial = _RecordingSerial()
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]
    motor._is_connected = True

    assert motor.get_power_v() is None

    for _ in range(50):
        # An hour of dashboard ticks per iteration: any TTL or backoff, however
        # long, would have expired long ago.
        clock.advance(3600)
        assert motor.get_power_v() is None

    assert serial.query_calls == [], (
        f"the driver still probes the board for a voltage it cannot report: {serial.query_calls}"
    )


def test_voltage_probe_does_not_reach_the_simulated_board() -> None:
    """The same contract end to end: the board receives no bytes at all."""
    clock = Clock()
    sim = SkyWatcherSim(clock)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    motor = SkyWatcherMotor(line, clock)
    motor.connect()

    received = bytearray()
    original_feed = sim.feed

    def _recording_feed(data: bytes) -> None:
        received.extend(data)
        original_feed(data)

    sim.feed = _recording_feed  # type: ignore[method-assign]

    for _ in range(10):
        clock.advance(120)
        assert motor.get_power_v() is None

    assert bytes(received) == b"", f"the board was asked something after all: {bytes(received)!r}"


def test_unsupported_voltage_is_announced_once_and_never_as_an_error(caplog: pytest.LogCaptureFixture) -> None:
    """A permanent, known property of the board is not a per-tick failure."""
    clock = Clock()
    serial = _RecordingSerial()
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]
    motor._is_connected = True

    with caplog.at_level(logging.DEBUG):
        for _ in range(20):
            clock.advance(600)
            motor.get_power_v()

    voltage_records = [r for r in caplog.records if "voltage" in r.getMessage().lower()]

    assert len(voltage_records) == 1, (
        f"expected exactly one note about the missing voltage query, got "
        f"{[r.getMessage() for r in voltage_records]}"
    )
    assert voltage_records[0].levelno < logging.WARNING, (
        "a board property that will never change is not a warning"
    )
