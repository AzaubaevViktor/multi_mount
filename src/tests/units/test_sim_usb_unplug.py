"""Transport-level failure semantics of the simulated port (PLAN.md §1 П2, §2.4).

`test_red_p2_usb_unplug.py` checks the axis layer with a stub motor; this file
checks the layer underneath it — that the fake port fails the way pyserial
fails, so `EXCEPTIONS_TO_CLOSE` -> `SerialLine._disconnect_when_error` ->
`SerialLine.close()` is exercised for real instead of being assumed.
"""

import errno

import pytest
from serial.serialutil import SerialException

from serial_wrapper.wrapper import SerialLineClosedError, SerialLineState
from sim import Clock, FakeSerial, SimSerialLine, SkyWatcherSim
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection
from skywatcher.motor import SkyWatcherMotor

_RA_CPR = 8_000_000


class _EchoDevice:
    def __init__(self) -> None:
        self.dtr_events: list[bool] = []
        self._out = bytearray()

    def feed(self, data: bytes) -> None:
        self._out.extend(data.upper())

    def drain(self) -> bytes:
        data = bytes(self._out)
        self._out.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        self.dtr_events.append(level)


def test_closed_port_raises_like_pyserial() -> None:
    """Every I/O on a closed port is `SerialException`, as in serialposix.

    A port that keeps answering after `close()` would hide the whole
    degradation path, since that path is triggered by these exceptions.
    """
    port = FakeSerial(_EchoDevice(), Clock())
    port.close()

    with pytest.raises(SerialException):
        port.write(b"ab\r")
    with pytest.raises(SerialException):
        port.flush()
    with pytest.raises(SerialException):
        port.read(1)
    with pytest.raises(SerialException):
        port.read_until(b"\r")
    with pytest.raises(SerialException):
        port.read_all()
    with pytest.raises(SerialException):
        port.reset_input_buffer()
    with pytest.raises(SerialException):
        port.reset_output_buffer()


def test_closed_port_caches_dtr_without_touching_the_device() -> None:
    """pyserial's `dtr` setter only stores the level while the port is closed."""
    device = _EchoDevice()
    port = FakeSerial(device, Clock())
    port.close()

    port.dtr = False

    assert port.dtr is False
    assert device.dtr_events == []


def test_unplugged_port_raises_enxio_on_every_operation() -> None:
    """The device vanishes; the port object does not notice until the next call.

    ENXIO / "Device not configured" is verbatim what ended both real sessions
    (PLAN.md §1 П2).
    """
    port = FakeSerial(_EchoDevice(), Clock())
    port.write(b"ok\r")

    port.unplug()

    assert port.is_open is True
    for operation in (
        lambda: port.write(b"ab\r"),
        lambda: port.read_all(),
        lambda: port.read_until(b"\r"),
        lambda: port.reset_input_buffer(),
        lambda: setattr(port, "dtr", False),
    ):
        with pytest.raises(OSError) as unplugged:
            operation()
        assert unplugged.value.errno == errno.ENXIO
        assert not isinstance(unplugged.value, SerialException)

    # Closing a dead device still has to work: it is what the error path runs.
    port.close()
    assert port.is_open is False


def test_serial_line_closes_itself_when_the_device_is_unplugged() -> None:
    line = SimSerialLine(_EchoDevice(), Clock(), port="sim://unplug", timeout_s=0.1, name="sim-unplug")
    line.connect()
    assert line.query("ab\r") == "AB\r"

    assert line.serial is not None
    line.serial.unplug()

    with pytest.raises(OSError) as unplugged:
        line.query("cd\r")

    assert unplugged.value.errno == errno.ENXIO
    assert line.state == SerialLineState.CLOSED
    assert line.serial is None

    meta = line.last_close_meta
    assert meta is not None
    assert meta.reason == "error_in_query"
    assert meta.error_type == "OSError"
    assert "Device not configured" in (meta.error_message or "")


def test_unplug_mid_tracking_degrades_the_skywatcher_axis() -> None:
    """Full stack: unplug during tracking must degrade, not hang or lie.

    The first command after the unplug surfaces the ENXIO and closes the line;
    every later command then fails fast with a named error carrying the close
    reason, instead of retrying into a silent zero-length answer.
    """
    clock = Clock()
    sim = SkyWatcherSim(clock, cpr=_RA_CPR)
    line = SimSerialLine(sim, clock, port="sim://ra", timeout_s=0, name="sim-ra", terminator="\r")
    motor = SkyWatcherMotor(line, clock)
    motor.connect()

    motor.set_speed(motor.convert_speed_to_steps_per_second(STELLAR_SPEED))
    motor.set_direction(MotorDirection.FORWARD)
    motor.set_motion_mode(MotionMode.RUN)
    motor.run()
    clock.advance(60)
    assert motor.status().motion_mode == MotionMode.RUN

    assert line.serial is not None
    line.serial.unplug()

    with pytest.raises(OSError) as unplugged:
        motor.status()
    assert unplugged.value.errno == errno.ENXIO
    assert line.state == SerialLineState.CLOSED

    with pytest.raises(SerialLineClosedError) as closed:
        motor.status()
    assert "reason=error_in_query" in str(closed.value)
    assert "Device not configured" in str(closed.value)
