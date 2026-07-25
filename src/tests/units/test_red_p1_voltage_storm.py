"""П1: reading the RA supply voltage must not turn into a stream of commands.

The original defect was `:fL#`, a command that cannot exist on this board (`L`
sits where the hex channel belongs, `#` terminates nothing), polled on every
dashboard tick: three 507 ms timeouts and a WARNING per attempt, forever. The
command was removed — and then found for real: the `:C`/`:n` memory window,
disassembled out of the vendor's app and confirmed on the wire
(RA_PROTOCOL.md §6.8).

That makes the storm question live again, and worse than before: one reading is
**eight** commands (`:C1` + `:n1` per byte, four bytes for the two channels),
and `stdout_dashboard` asks for the voltage on every tick. So the contract
pinned here is no longer "nothing is ever sent" but the real one:

* frequent polling costs one reading per TTL, not one per call;
* a board that refuses to answer is asked again after a backoff, not in a loop;
* and neither case ever raises.
"""

import logging

import pytest

from sim import Clock, SimSerialLine, SkyWatcherSim
from skywatcher.motor import SkyWatcherMotor
from skywatcher.session import SkyWatcherSession

_READING_COMMANDS = 8


class _RecordingSerial:
    """SerialLine stand-in that records every payload the driver sends."""

    terminator = b"\r"

    def __init__(self, answer: str = "") -> None:
        self.answer = answer
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
        return self.answer

    def drop_buffers(self) -> None:
        pass

    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        return None


def _connected_motor(sim: SkyWatcherSim, clock: Clock) -> tuple[SkyWatcherMotor, list[bytes]]:
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    motor = SkyWatcherMotor(line, clock)
    motor.connect()

    received: list[bytes] = []
    original_feed = sim.feed

    def _recording_feed(data: bytes) -> None:
        received.append(data)
        original_feed(data)

    sim.feed = _recording_feed  # type: ignore[method-assign]
    return motor, received


def test_frequent_polling_costs_one_reading_per_ttl_not_one_per_call() -> None:
    """A dashboard tick every 100 ms must not be a command every 25 ms."""
    clock = Clock()
    sim = SkyWatcherSim(clock)
    motor, received = _connected_motor(sim, clock)

    ticks = 100
    tick_s = 0.1
    for _ in range(ticks):
        assert motor.get_power_v() == pytest.approx(6.04)
        clock.advance(tick_s)

    elapsed_s = ticks * tick_s
    allowed_readings = int(elapsed_s / SkyWatcherSession._POWER_CACHE_TTL_S) + 1
    commands = [frame for frame in received if frame.startswith(b":")]

    assert len(commands) <= allowed_readings * _READING_COMMANDS, (
        f"{ticks} ticks over {elapsed_s:.0f}s sent {len(commands)} commands; "
        f"a {SkyWatcherSession._POWER_CACHE_TTL_S:.0f}s cache allows at most "
        f"{allowed_readings * _READING_COMMANDS}"
    )
    # And the cache is a cache, not a mute: over ten TTLs it did refresh.
    assert len(commands) >= _READING_COMMANDS


def test_a_stale_reading_is_refreshed_once_the_ttl_expires() -> None:
    """The other half of the same contract: the number must not freeze."""
    clock = Clock()
    sim = SkyWatcherSim(clock)
    motor, _received = _connected_motor(sim, clock)

    assert motor.get_power_v() == pytest.approx(6.04)

    sim.battery_volt_hundredths = 512
    clock.advance(SkyWatcherSession._POWER_CACHE_TTL_S / 2)
    assert motor.get_power_v() == pytest.approx(6.04)

    clock.advance(SkyWatcherSession._POWER_CACHE_TTL_S)
    assert motor.get_power_v() == pytest.approx(5.12)


def test_a_board_that_refuses_the_query_is_not_hammered(caplog: pytest.LogCaptureFixture) -> None:
    """An unreadable voltage is None and a backoff, never an exception or a storm."""
    clock = Clock()
    serial = _RecordingSerial(answer="!0\r")
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]
    motor._is_connected = True

    with caplog.at_level(logging.DEBUG):
        for _ in range(600):
            assert motor.get_power_v() is None
            clock.advance(0.1)

    elapsed_s = 600 * 0.1
    attempts = len([call for call in serial.query_calls if call.startswith(":C1")])
    allowed_attempts = int(elapsed_s / SkyWatcherSession._POWER_FAILURE_BACKOFF_S) + 1

    assert attempts <= allowed_attempts, (
        f"a refusing board was probed {attempts} times in {elapsed_s:.0f}s, "
        f"the backoff allows {allowed_attempts}"
    )
    assert attempts >= 1
    # The complaint is one line per attempt at most, not one per tick.
    voltage_warnings = [r for r in caplog.records if "supply voltage" in r.getMessage()]
    assert len(voltage_warnings) == attempts


def test_garbage_from_the_window_is_not_a_voltage_and_not_a_crash() -> None:
    """`=` with a body that is not two hex chars is a damaged frame, not 0 V."""
    clock = Clock()
    serial = _RecordingSerial(answer="=ZZ\r")
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]
    motor._is_connected = True

    assert motor.get_power_v() is None


def test_a_disconnected_motor_reports_nothing_and_sends_nothing() -> None:
    clock = Clock()
    serial = _RecordingSerial(answer="=5C\r")
    motor = SkyWatcherMotor(serial, clock)  # type: ignore[arg-type]

    assert motor.get_power_v() is None
    assert serial.query_calls == []
