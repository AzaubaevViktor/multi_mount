from serial_wrapper.wrapper import SerialLineState

from sim import Clock, FakeSerial, SimSerialLine, Transport


class _EchoDevice:
    def __init__(self) -> None:
        self.received = bytearray()
        self.out = bytearray()
        self.dtr_events: list[bool] = []

    def feed(self, data: bytes) -> None:
        self.received.extend(data)
        self.out.extend(data.upper())

    def drain(self) -> bytes:
        data = bytes(self.out)
        self.out.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        self.dtr_events.append(level)


def test_fake_serial_write_read_roundtrip() -> None:
    device = _EchoDevice()
    port = FakeSerial(device, Clock())

    port.write(b"ab\r")

    assert port.read_until(b"\r") == b"AB\r"
    assert port.read_all() == b""


def test_fake_serial_read_single_bytes_and_reset_input_buffer() -> None:
    device = _EchoDevice()
    port = FakeSerial(device, Clock())
    port.write(b"xy\r")

    assert port.read(1) == b"X"
    assert port.read_all() == b"Y\r"

    port.write(b"z\r")
    port.reset_input_buffer()
    assert port.read_all() == b""


def test_fake_serial_dtr_edges_reach_device() -> None:
    device = _EchoDevice()
    port = FakeSerial(device, Clock())

    port.dtr = False
    port.dtr = False
    port.dtr = True

    assert device.dtr_events == [False, True]


def test_transport_seam_intercepts_write_and_read() -> None:
    device = _EchoDevice()
    transport = Transport(on_write=lambda data: data[:-1], on_read=lambda data: data.lower())
    port = FakeSerial(device, Clock(), transport)

    port.write(b"ab!")

    assert bytes(device.received) == b"ab"
    assert port.read_all() == b"ab"


def test_sim_serial_line_connect_installs_fake_port() -> None:
    device = _EchoDevice()
    line = SimSerialLine(device, Clock(), port="sim://test", timeout_s=0.5, name="sim-test", terminator="\r")

    line.connect()

    assert line.state == SerialLineState.OPEN
    assert isinstance(line.serial, FakeSerial)
    assert line.serial.timeout == 0.5
    assert line.query("ab\r") == "AB\r"
