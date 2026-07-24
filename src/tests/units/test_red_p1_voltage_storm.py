"""Red tests for П1 (INQUIRE_VOLTAGE storm), see PLAN.md §1 П1.

The dashboard polls ``get_power_v()`` every ~2s. A board that does not support
``:fL#`` answers with an empty response. Today each poll costs 3 ``_transact``
retries plus one ``get_power_v`` log = ~4 error records with tracebacks per
failure, and the "unsupported" verdict is never remembered, producing a storm
(27k+ tracebacks in one real session).

Desired contract:
  (a) After the first full failure the command is marked unsupported: further
      ``get_power_v()`` calls issue no new serial queries (or issue them only
      after a >= 60s virtual backoff).
  (b) A single failure produces at most one error-level log record.
"""

import logging

import pytest

import skywatcher.motor
from skywatcher.motor import SkyWatcherMotor


@pytest.fixture(autouse=True)
def _no_real_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    # _transact sleeps 0.1s between its internal retries; keep tests off real time.
    monkeypatch.setattr(skywatcher.motor.time, "sleep", lambda seconds: None)


class _DeadVoltageSerial:
    """Minimal SerialLine stand-in: ``:fL#`` always returns an empty string."""

    def __init__(self) -> None:
        self.query_calls: list[str] = []

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


def _make_connected_motor(serial: _DeadVoltageSerial) -> SkyWatcherMotor:
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]
    motor._is_connected = True
    return motor


def test_voltage_failure_is_remembered_and_stops_new_queries() -> None:
    """П1(a): a dead ``:fL#`` is probed once, not on every poll.

    See PLAN.md §1 П1 / §3 item 7.
    """
    serial = _DeadVoltageSerial()
    motor = _make_connected_motor(serial)

    # First poll: real probe, allowed to fail.
    assert motor.get_power_v() is None

    voltage_queries_after_first = len(serial.query_calls)

    # Many further polls without advancing the 2s cache TTL: a fixed, dead board
    # must not be re-probed on every dashboard tick.
    for _ in range(5):
        motor._last_power_v_updated = 0.0  # force cache to look stale
        motor.get_power_v()

    new_voltage_queries = len(serial.query_calls) - voltage_queries_after_first

    assert new_voltage_queries == 0, (
        f"expected no new serial queries after the first voltage failure, "
        f"got {new_voltage_queries} (command not marked unsupported / no backoff)"
    )


def test_single_voltage_failure_emits_at_most_one_error_log(caplog: pytest.LogCaptureFixture) -> None:
    """П1(b): one failure => at most one error record, not ~4.

    See PLAN.md §1 П1.
    """
    serial = _DeadVoltageSerial()
    motor = _make_connected_motor(serial)

    with caplog.at_level(logging.ERROR):
        motor.get_power_v()

    error_records = [r for r in caplog.records if r.levelno >= logging.ERROR]

    assert len(error_records) <= 1, (
        f"one voltage failure produced {len(error_records)} error records "
        f"(expected <= 1): {[r.getMessage() for r in error_records]}"
    )
