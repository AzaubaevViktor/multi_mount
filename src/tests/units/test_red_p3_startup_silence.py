"""Red tests for П3 (mount silent on startup => instant opaque crash), PLAN.md §1 П3.

Real sessions died in <0.5s when ``INITIALIZE :F1`` got an empty answer: a bare
``SkyWatcherMotorProtocolError("empty response: '')`` with no app-level retry or
backoff and no operator-readable "mount not responding on <port>". A manual
third launch succeeded, i.e. the failure was transient and worth retrying.

Desired contract for ``connect()`` against a permanently silent mount:
  (a) It fails with a message naming both the unavailability of the mount and
      the serial port, not a raw protocol error about an empty string.
  (b) It performs a bounded application-level retry: the total number of serial
      attempts is limited but strictly greater than the 3 fast in-transaction
      retries of a single transaction (today it dies after exactly those 3).
"""

import pytest

from skywatcher.motor import SkyWatcherMotor, SkyWatcherMotorError
from skywatcher.protocol import Protocol


class _SilentMount:
    """SerialLine stand-in for a mount that answers every query with ''."""

    def __init__(self, port: str) -> None:
        self.port = port
        self.terminator = Protocol.ANSWER_END_BYTE
        self.connect_calls = 0
        self.query_calls: list[str] = []

    def connect(self) -> None:
        self.connect_calls += 1

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

    def close(self) -> None:
        pass


def test_connect_error_names_mount_unavailability_and_port() -> None:
    """П3(a): the failure message mentions the mount being unreachable and the port.

    See PLAN.md §1 П3 / §3 item 9.
    """
    serial = _SilentMount("/dev/ttyUSB-RA")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorError) as excinfo:
        motor.connect()

    message = str(excinfo.value).lower()
    assert "/dev/ttyusb-ra" in message, (
        f"connect() error must name the serial port, got: {str(excinfo.value)!r}"
    )
    assert any(word in message for word in ("responding", "unavailable", "unreachable", "not respond")), (
        f"connect() error must state the mount is unavailable/not responding, "
        f"got: {str(excinfo.value)!r}"
    )
    assert "empty response" not in message, (
        f"connect() leaked the raw low-level protocol error instead of a "
        f"diagnostic message: {str(excinfo.value)!r}"
    )


def test_connect_does_bounded_retry_with_more_than_three_attempts() -> None:
    """П3(b): connect retries at the app level, more than one transaction's 3 tries.

    See PLAN.md §1 П3.
    """
    serial = _SilentMount("/dev/ttyUSB-RA")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorError):
        motor.connect()

    # A single transaction already retries 3 times internally. An app-level
    # bounded retry must exceed that before giving up.
    assert len(serial.query_calls) > SkyWatcherMotor.REPEATS, (
        f"connect() made only {len(serial.query_calls)} serial attempts "
        f"(<= one transaction's {SkyWatcherMotor.REPEATS} retries): no app-level "
        f"retry/backoff on startup silence"
    )
