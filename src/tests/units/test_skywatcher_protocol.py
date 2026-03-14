from types import MethodType

import pytest

from serial_wrapper.wrapper import SerialLine
from sky.physics import Ha
from skywatcher.motor import (
    SkyWatcherMotor,
    SkyWatcherMotorCommandError,
    SkyWatcherMotorProtocolError,
    _Command,
    _Direction,
    _SlewMode,
    _SpeedMode,
    _Status,
)
from skywatcher.protocol import Protocol


class _FakePySerial:
    def __init__(self, prefix_stream: bytes, line_after_prefix: bytes) -> None:
        self.timeout = 0.25
        self._prefix_stream = list(prefix_stream)
        self._line_after_prefix = line_after_prefix
        self.written = bytearray()
        self.flush_calls = 0
        self.reset_input_buffer_calls = 0

    def reset_input_buffer(self) -> None:
        self.reset_input_buffer_calls += 1

    def write(self, data: bytes) -> None:
        self.written.extend(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def read(self, size: int = 1) -> bytes:
        assert size == 1
        if not self._prefix_stream:
            return b""
        return bytes([self._prefix_stream.pop(0)])

    def read_until(self, expected: bytes, size: int = 1024) -> bytes:
        return self._line_after_prefix


class _FakeSkyWatcherSerial:
    def __init__(self, response: str, terminator: bytes = b"\r", responses_by_payload: dict[str, str] | None = None) -> None:
        self.response = response
        self.terminator = terminator
        self.responses_by_payload = responses_by_payload or {}
        self.calls: list[tuple[str, tuple[bytes, ...] | None, bytes | str | None]] = []

    def connect(self) -> None:
        raise AssertionError("connect() should not be called")

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        self.calls.append((payload or "", response_prefixes, response_terminator))
        if payload is not None and payload in self.responses_by_payload:
            return self.responses_by_payload[payload]
        return self.response

    def drop_buffers(self) -> None:
        raise AssertionError("drop_buffers() should not be called")

    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        raise AssertionError("read_all_data() should not be called")


def test_serial_line_query_waits_for_prefix_then_reads_until_terminator() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "skywatcher-test", terminator="\r")
    line.serial = _FakePySerial(b"noise\r:ignored=", b"ABC123\r")  # type: ignore[assignment]

    response = line.query(
        ":a1\r",
        timeout=0.5,
        response_prefixes=(Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE),
    )

    assert response == "=ABC123\r"
    assert line.serial.timeout == 0.25
    assert line.serial.written == b":a1\r"
    assert line.serial.flush_calls == 1
    assert line.serial.reset_input_buffer_calls == 1


def test_serial_line_query_can_read_with_custom_terminator() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "skywatcher-test", terminator="\r")
    line.serial = _FakePySerial(b"", b"7E#")  # type: ignore[assignment]

    response = line.query(":fL#", timeout=0.5, response_terminator="#")

    assert response == "7E#"
    assert line.serial.written == b":fL#"


def test_skywatcher_connect_rejects_wrong_serial_terminator() -> None:
    serial = _FakeSkyWatcherSerial("=000000\r", terminator=b"\n")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorProtocolError, match="invalid SerialLine terminator"):
        motor.connect()


def test_skywatcher_transact_requests_prefixed_response_and_strips_answer_end() -> None:
    serial = _FakeSkyWatcherSerial("=010203\r")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    response = motor._transact(_Command.INQUIRE_CPR)

    assert response == "010203"
    assert serial.calls == [
        (
            ":a1\r",
            (Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE),
            None,
        )
    ]


def test_skywatcher_transact_raises_command_error_on_error_prefix() -> None:
    serial = _FakeSkyWatcherSerial("!02\r")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorCommandError, match="command error"):
        motor._transact(_Command.INQUIRE_CPR)


def test_skywatcher_transact_reads_voltage_from_hash_response() -> None:
    serial = _FakeSkyWatcherSerial("7E#")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    response = motor._transact(_Command.INQUIRE_VOLTAGE)

    assert response == "7E"
    assert serial.calls == [
        (
            ":fL#",
            None,
            "#",
        )
    ]


def test_skywatcher_get_power_v_reads_voltage_via_transact() -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._is_connected = True
    motor._steps_360 = 86400
    motor._get_status = MethodType(
        lambda self: _Status(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=_SlewMode.SLEW,
            direction=_Direction.FORWARD,
            speed_mode=_SpeedMode.LOWSPEED,
        ),
        motor,
    )
    motor._get_position = MethodType(lambda self: Ha(0), motor)
    transact_calls: list[_Command] = []
    motor._transact = MethodType(
        lambda self, command, arg=None: transact_calls.append(command) or ("7E" if command == _Command.INQUIRE_VOLTAGE else ""),
        motor,
    )

    power_v = motor.get_power_v()

    assert power_v == pytest.approx(12.6)
    assert transact_calls == [_Command.INQUIRE_VOLTAGE]


def test_skywatcher_status_does_not_fetch_voltage() -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._is_connected = True
    motor._steps_360 = 86400
    motor._get_status = MethodType(
        lambda self: _Status(
            raw=0,
            running=False,
            initialized=True,
            slew_mode=_SlewMode.SLEW,
            direction=_Direction.FORWARD,
            speed_mode=_SpeedMode.LOWSPEED,
        ),
        motor,
    )
    motor._get_position = MethodType(lambda self: Ha(0), motor)
    transact_calls: list[_Command] = []
    motor._transact = MethodType(lambda self, command, arg=None: transact_calls.append(command) or "", motor)

    status = motor.status()

    assert status.power_v is None
    assert _Command.INQUIRE_VOLTAGE not in transact_calls
