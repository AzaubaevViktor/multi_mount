"""Parsing of TMC2209 controller replies.

Covers two things the DEC firmware change touches:

* the new ``tx_overflow`` key in ``status`` must be optional, because the board in the
  field still runs firmware without it;
* a failed transaction must say *why* it failed. Previously silence, a cut-off line and
  two glued lines all surfaced as ``invalid response: ''`` in the logs.
"""

import logging

import pytest
from tmc2209.motor import (
    TMC2209Motor,
    TMC2209MotorConcatenatedResponseError,
    TMC2209MotorProtocolError,
    TMC2209MotorTimeoutError,
    TMC2209MotorTruncatedResponseError,
    _Mode,
    _Phase,
    _Response,
    _Status,
)

# Exactly what the firmware emits today, minus the key added together with the TX-ring fix.
_LEGACY_STATUS = (
    "1;initialised=1;enabled=1;mode=target;position=123456;phase=running;"
    "target=200000;target_set=1;speed=1000.00;actual_speed=980.00;"
    "accel_per_s=1000.00;power_v=12.34;\n"
)
_NEW_STATUS = _LEGACY_STATUS.replace("power_v=12.34;", "power_v=12.34;tx_overflow=7;")


def test_status_without_tx_overflow_key_still_parses() -> None:
    status = _Status.from_response(_Response.from_line(_LEGACY_STATUS))

    assert status.tx_overflow is None
    assert status.position == 123456
    assert status.mode == _Mode.TARGET
    assert status.phase == _Phase.RUNNING
    assert status.power_v == pytest.approx(12.34)


def test_status_with_tx_overflow_key_parses_the_counter() -> None:
    status = _Status.from_response(_Response.from_line(_NEW_STATUS))

    assert status.tx_overflow == 7
    assert status.position == 123456
    assert status.power_v == pytest.approx(12.34)


def test_firmware_overflow_marker_is_a_plain_error_response() -> None:
    response = _Response.from_line("0;error=tx_overflow;\n")

    assert response.ok is False
    assert response.error == "tx_overflow"


def test_empty_response_is_reported_as_silence_not_as_garbage() -> None:
    with pytest.raises(TMC2209MotorTimeoutError, match="no response"):
        _Response.from_line("")


def test_response_cut_off_mid_token_is_reported_as_truncated() -> None:
    with pytest.raises(TMC2209MotorTruncatedResponseError, match="cut off"):
        _Response.from_line("1;initialised=1;enabled=1;mode=tar")


def test_response_without_trailing_delimiter_is_reported_as_truncated() -> None:
    with pytest.raises(TMC2209MotorTruncatedResponseError, match="before its terminator"):
        _Response.from_line("1;initialised=1;position=42")


def test_two_glued_responses_are_reported_as_concatenated() -> None:
    with pytest.raises(TMC2209MotorConcatenatedResponseError, match="glued"):
        _Response.from_line("1;initialised=1;1;initialised=1;enabled=1;\n")


def test_unrecognised_line_is_reported_separately() -> None:
    with pytest.raises(TMC2209MotorProtocolError, match="no `0;`/`1;` status prefix"):
        _Response.from_line("ready\n")


@pytest.mark.parametrize(
    "line",
    ["", "1;initialised=1;mode=tar", "1;initialised=1;1;initialised=1;", "ready"],
)
def test_all_failure_modes_stay_protocol_errors(line: str) -> None:
    # Callers (and the hardware tests) catch the base class; splitting the reasons must not
    # change what escapes from_line.
    with pytest.raises(TMC2209MotorProtocolError):
        _Response.from_line(line)


def test_failure_modes_produce_distinct_messages() -> None:
    messages = set()
    for line in ("", "1;initialised=1;mode=tar", "1;initialised=1;1;initialised=1;", "ready"):
        with pytest.raises(TMC2209MotorProtocolError) as excinfo:
            _Response.from_line(line)
        messages.add(str(excinfo.value))

    assert len(messages) == 4


def test_growing_overflow_counter_is_logged_once_per_increment(caplog: pytest.LogCaptureFixture) -> None:
    lines = iter([_NEW_STATUS, _NEW_STATUS, _NEW_STATUS.replace("tx_overflow=7", "tx_overflow=9")])

    class _Line:
        def query(self, payload: str, **kwargs: object) -> str:
            assert payload == "status\n"
            return next(lines)

    motor = TMC2209Motor(_Line())  # type: ignore[arg-type]
    with caplog.at_level(logging.WARNING):
        motor._status()
        motor._status()
        motor._status()

    warnings = [record.getMessage() for record in caplog.records if record.levelno == logging.WARNING]
    assert warnings == [
        "TMC2209 dropped 7 response(s) to TX ring overflow, 7 since controller boot",
        "TMC2209 dropped 2 response(s) to TX ring overflow, 9 since controller boot",
    ]
