from serial.serialutil import SerialException

import pytest

from serial_wrapper.wrapper import SerialLine, SerialLineState


class _FakeSerial:
    def __init__(self, *, reset_error: BaseException | None = None) -> None:
        self.is_open = True
        self.timeout = 0.25
        self._reset_error = reset_error
        self.written = bytearray()
        self.flush_calls = 0

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        if self._reset_error is not None:
            raise self._reset_error

    def reset_output_buffer(self) -> None:
        return None

    def write(self, data: bytes) -> None:
        self.written.extend(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def read_until(self, expected: bytes, size: int = 1024) -> bytes:
        return b"OK\r"

    def read_all(self) -> bytes:
        return b""


def test_close_tracks_state_and_last_close_meta() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "serial-state-test", terminator="\r")
    line.serial = _FakeSerial()

    line.close()

    assert line.state == SerialLineState.CLOSED
    assert line.serial is None
    assert line.last_close_meta is not None
    assert line.last_close_meta.reason == "manual_close"
    assert "test_close_tracks_state_and_last_close_meta" in line.last_close_meta.closed_from
    assert "test_close_tracks_state_and_last_close_meta" in line.last_close_meta.caller_frame


def test_query_on_closed_line_returns_default_response() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "serial-state-test", terminator="\r")
    line.close()

    response = line.query(":GR#")

    assert response == ""
    assert line.state == SerialLineState.CLOSED
    assert line.last_close_meta is not None
    assert line.last_close_meta.reason == "manual_close"


def test_query_error_closes_line_with_reason_and_error_meta() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "serial-state-test", terminator="\r")
    line.serial = _FakeSerial(reset_error=SerialException("broken serial"))

    with pytest.raises(SerialException, match="broken serial"):
        line.query(":GR#")

    assert line.state == SerialLineState.CLOSED
    assert line.serial is None
    assert line.last_close_meta is not None
    assert line.last_close_meta.reason == "error_in_query"
    assert line.last_close_meta.closed_from == "SerialLine.query"
    assert "test_query_error_closes_line_with_reason_and_error_meta" in line.last_close_meta.caller_frame
    assert line.last_close_meta.error_type == "SerialException"
    assert line.last_close_meta.error_message == "broken serial"


class _SlowFakeSerial(_FakeSerial):
    """A device that answers only after the caller has waited a while.

    ``read_all`` hands out one scripted chunk per poll, so the script doubles as a
    timeline: ``[b"", b"", b"hi\r"]`` is "silent for two polls, then answers".
    """

    def __init__(self, script: list[bytes]) -> None:
        super().__init__()
        self._script = list(script)
        self.poll_count = 0

    def read_all(self) -> bytes:
        self.poll_count += 1
        return self._script.pop(0) if self._script else b""


def test_read_all_data_waits_for_a_device_that_answers_late(virtual_clock) -> None:
    """`pyserial`'s read_all() is read(in_waiting) — non-blocking. The wait is ours.

    Before this, `read_all_data(timeout=.5)` returned instantly no matter what the
    signature promised: 20 601 records of `['']` in the March logs came from exactly
    this. The device here stays quiet for two polls and then speaks.
    """
    line = SerialLine("/dev/null", 9600, 0.25, "read-all-wait", terminator="\r", clock=virtual_clock)
    line.serial = _SlowFakeSerial([b"", b"", b"late\r"])

    started = virtual_clock.monotonic()
    lines = line.read_all_data(timeout=.5)

    assert lines is not None
    assert "late" in lines
    assert virtual_clock.monotonic() > started, "a wait that costs no time is not a wait"


def test_read_all_data_returns_as_soon_as_the_device_goes_quiet(virtual_clock) -> None:
    """Having got an answer, do not sit on the lock until the timeout expires."""
    line = SerialLine("/dev/null", 9600, 0.25, "read-all-quiet", terminator="\r", clock=virtual_clock)
    fake = _SlowFakeSerial([b"one\r", b""])
    line.serial = fake

    started = virtual_clock.monotonic()
    lines = line.read_all_data(timeout=10.0)

    assert lines is not None
    assert "one" in lines
    assert virtual_clock.monotonic() - started < 1.0, "returned only after the full timeout"


def test_read_all_data_without_timeout_stays_non_blocking(virtual_clock) -> None:
    """`timeout=None` keeps the old meaning: drain what is already buffered, once.

    That is what post-connect boot-garbage cleanup wants, and it must not start waiting.
    """
    line = SerialLine("/dev/null", 9600, 0.25, "read-all-drain", terminator="\r", clock=virtual_clock)
    fake = _SlowFakeSerial([b"", b"too late\r"])
    line.serial = fake

    started = virtual_clock.monotonic()
    lines = line.read_all_data()

    assert lines == [""]
    assert fake.poll_count == 1
    assert virtual_clock.monotonic() == started
