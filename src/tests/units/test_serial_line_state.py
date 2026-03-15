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
    line.serial = _FakeSerial()  # type: ignore[assignment]

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
    line.serial = _FakeSerial(reset_error=SerialException("broken serial"))  # type: ignore[assignment]

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
