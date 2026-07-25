import pytest

from serial_wrapper.wrapper import SerialLine
from skywatcher.board import DEFAULT_MIN_PERIOD, SkyWatcherBoard
from skywatcher.codec import (
    Command,
    Direction,
    SkyWatcherCodec,
    SkyWatcherMotorCommandError,
    SkyWatcherMotorProtocolError,
    SlewMode,
    SpeedMode,
    Status,
)
from skywatcher.motor import SkyWatcherMotor
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


_IDLE_STATUS = Status(
    raw=0,
    running=False,
    initialized=True,
    slew_mode=SlewMode.SLEW,
    direction=Direction.FORWARD,
    speed_mode=SpeedMode.LOWSPEED,
)


def test_serial_line_query_waits_for_prefix_then_reads_until_terminator() -> None:
    line = SerialLine("/dev/null", 9600, 0.25, "skywatcher-test", terminator="\r")
    line.serial = _FakePySerial(b"noise\r:ignored=", b"ABC123\r")

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
    """Transport-level feature only: no SkyWatcher command uses `#` (§8).

    The RA board terminates everything with `\\r`; this checks that a device
    speaking some other framing can still be driven through the same transport.
    """
    line = SerialLine("/dev/null", 9600, 0.25, "skywatcher-test", terminator="\r")
    line.serial = _FakePySerial(b"", b"7E#")

    response = line.query(":X1#", timeout=0.5, response_terminator="#")

    assert response == "7E#"
    assert line.serial.written == b":X1#"


def test_skywatcher_connect_rejects_wrong_serial_terminator() -> None:
    serial = _FakeSkyWatcherSerial("=000000\r", terminator=b"\n")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    with pytest.raises(SkyWatcherMotorProtocolError, match="invalid SerialLine terminator"):
        motor.connect()


class _ConnectSerial(_FakeSkyWatcherSerial):
    """The handshake of the live board, including the step-period clamp of §10.5.

    `:I1` is accepted with any value and stores `max(value, clamp)`; `:i1` reads
    back what was stored. That pair is the whole of the driver's min-period
    probe, so the fake has to own a period, not answer from a table.
    """

    def __init__(self, clamp: int = 1103, initial_period: int = 110_359, period_error: str | None = None) -> None:
        super().__init__(
            "=000000\r",
            responses_by_payload={
                ":F1\r": "=000000\r",
                ":e1\r": "=03110A\r",
                ":a1\r": "=12A7BE\r",
                ":b1\r": "=2400F4\r",
                ":g1\r": "=010000\r",
                ":f1\r": "=101\r",
            },
        )
        self.connected = False
        self.clamp = clamp
        self.step_period = initial_period
        self.period_error = period_error

    def connect(self) -> None:
        self.connected = True

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        if payload is not None and payload.startswith((":I1", ":i1")):
            self.calls.append((payload, response_prefixes, response_terminator))
            if self.period_error is not None:
                return self.period_error
            if payload.startswith(":I1"):
                self.step_period = max(self.clamp, SkyWatcherCodec.decode_revu24(payload[3:-1]))
                return "=\r"
            return f"={SkyWatcherCodec.encode_revu24(self.step_period)}\r"
        return super().query(payload, timeout, response_prefixes, response_terminator)


def test_skywatcher_connect_parses_mcversion_with_board_byte_order() -> None:
    serial = _ConnectSerial()
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    motor.connect()

    assert serial.connected is True
    board = motor.board
    assert board is not None
    assert board.mount_code == 0x0A
    assert board.firmware_version == 0x0311
    assert board.min_period == 1103


def test_skywatcher_connect_probes_the_clamp_with_the_documented_command_pair() -> None:
    """§10.5 on the wire: read, write a value below any clamp, read back, put back.

    Pinned as payloads on purpose. The probe is only honest if the write really
    goes below the clamp (a value the board raises) and the restore really
    carries the period that was there before.
    """
    serial = _ConnectSerial()
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    motor.connect()

    period_payloads = [call[0] for call in serial.calls if call[0].startswith((":I1", ":i1"))]
    assert period_payloads == [
        ":i1\r",
        f":I1{SkyWatcherCodec.encode_revu24(1)}\r",
        ":i1\r",
        f":I1{SkyWatcherCodec.encode_revu24(110_359)}\r",
    ]
    assert serial.step_period == 110_359


def test_skywatcher_connect_survives_a_board_that_refuses_the_probe() -> None:
    """A board that will not talk about its period is not a failed connect.

    The fallback is the reference `CUSTOM` value of 6: it clamps nothing the
    board does not clamp itself, so the axis still runs — only the speed
    reported upwards may be optimistic, which is what the log says.
    """
    serial = _ConnectSerial(period_error="!0\r")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    motor.connect()

    board = motor.board
    assert board is not None
    assert board.min_period == DEFAULT_MIN_PERIOD
    assert motor._is_connected is True


def test_skywatcher_transact_requests_prefixed_response_and_strips_answer_end() -> None:
    serial = _FakeSkyWatcherSerial("=010203\r")
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]

    response = motor._session.transact(Command.INQUIRE_CPR)

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
        motor._session.transact(Command.INQUIRE_CPR)


class _VoltageWindowSerial(_FakeSkyWatcherSerial):
    """The `:C`/`:n` window byte by byte, as recorded on the wire (§6.8).

    The map is keyed by address so a driver that gets the address byte order
    wrong reads a different cell, not the same number by accident.
    """

    _MEMORY = {0x0004: "5C", 0x0005: "02", 0x001C: "D6", 0x001D: "01"}

    def __init__(self) -> None:
        super().__init__("=\r")
        self.address = 0

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        self.calls.append((payload or "", response_prefixes, response_terminator))
        assert payload is not None
        if payload.startswith(":C1"):
            data = payload[3:-1]
            self.address = int(data[2:4] + data[0:2], 16)
            return "=\r"
        if payload.startswith(":n1"):
            return f"={self._MEMORY.get(self.address, '00')}\r"
        return self.response


def test_skywatcher_voltage_is_read_through_the_memory_window_verbatim() -> None:
    """§6.8: `:C1<lo><hi>` then `:n1`, four times, low byte of each pair first.

    Pinned as payloads because every one of these details has a wrong variant
    that still parses: `:C10004` reads address 0x0400, a big-endian assembly
    turns 0x025C into 0x5C02, and skipping the `:C1` before the second `:n1`
    reads the low byte twice.
    """
    serial = _VoltageWindowSerial()
    motor = SkyWatcherMotor(serial)  # type: ignore[arg-type]
    motor._is_connected = True

    assert motor.get_power_v() == pytest.approx(6.04)

    assert [call[0] for call in serial.calls] == [
        ":C10400\r",
        ":n1\r",
        ":C10500\r",
        ":n1\r",
        ":C11C00\r",
        ":n1\r",
        ":C11D00\r",
        ":n1\r",
    ]


def test_skywatcher_status_does_not_fetch_voltage(monkeypatch: pytest.MonkeyPatch) -> None:
    motor = SkyWatcherMotor(object())  # type: ignore[arg-type]
    motor._is_connected = True
    motor._session._board = SkyWatcherBoard(cpr=86400, timer_freq=16_000_000, highspeed_ratio=1)
    monkeypatch.setattr(motor._session, "status", lambda: _IDLE_STATUS)
    monkeypatch.setattr(motor._session, "position_ticks", lambda fresh=False: 0)
    transact_calls: list[Command] = []

    def _transact(command: Command, arg: str | None = None) -> str:
        transact_calls.append(command)
        return ""

    monkeypatch.setattr(motor._session, "transact", _transact)

    status = motor.status()

    # Nothing has been read yet, so there is nothing to report — and status()
    # must not go and fetch it: it is called far more often than the voltage
    # moves, and one reading is eight commands.
    assert status.power_v is None
    assert transact_calls == []
