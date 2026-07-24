"""A fake ``pyserial.Serial`` compatible object driven by a simulated device.

Only the surface actually used by ``src/serial_wrapper/wrapper.py`` is
implemented. That surface is:

- ``is_open`` (attribute, read in ``connect``/``close``/``_require_open_serial``)
- ``close()``
- ``dtr`` (read+write attribute, toggled in ``SerialLine.reset``)
- ``timeout`` (read+write attribute)
- ``reset_input_buffer()`` / ``reset_output_buffer()`` (``drop_buffers``)
- ``write(bytes)`` / ``flush()``
- ``read(size)`` / ``read_until(expected, size)`` / ``read_all()``

The port owns two byte buffers (host->device input, device->host output) and a
:class:`Device`. Every command the host writes is fed to the device, which
synchronously appends its reply to the output buffer, so reads never block and
the real timeouts inside ``wrapper.py`` never fire (time is virtual).

Chaos seam
----------
All host<->device byte flow passes through :class:`Transport`, whose ``on_write``
and ``on_read`` hooks are the injection points for the future chaos stage
(connection drop mid-response, partial writes, byte loss). They default to
identity. Cheap scripted faults required *now* by the red P1/P3 tests live on
the device via :class:`FaultScript`, not here.
"""

from typing import Callable, Protocol

from serial_wrapper.wrapper import SerialLine, SerialLineState

from sim.clock import Clock


class Device(Protocol):
    """Contract a simulated endpoint (SkyWatcher / TMC) exposes to the port.

    The device is a pure byte pump: bytes written by the host go in via
    :meth:`feed`, and whatever the device has produced is drained via
    :meth:`drain`. All timing is driven by the shared :class:`Clock`, so the
    port never needs to know how the device schedules its answers.
    """

    def feed(self, data: bytes) -> None:
        """Consume bytes written by the host and enqueue any responses."""
        ...

    def drain(self) -> bytes:
        """Return and clear all bytes the device has ready for the host."""
        ...

    def on_dtr(self, level: bool) -> None:
        """React to a DTR line change (TMC reset / firmware ``ready``)."""
        ...


class Transport:
    """Byte-level seam between host writes/reads and the device buffers.

    ``on_write`` sees each chunk the host writes before it reaches the device;
    ``on_read`` sees each chunk handed back to the host. Both are identity by
    default. The chaos stage will replace them to drop/split/lose bytes without
    touching the device or the protocol layers.
    """

    def __init__(
        self,
        on_write: Callable[[bytes], bytes] | None = None,
        on_read: Callable[[bytes], bytes] | None = None,
    ) -> None:
        self.on_write = on_write or (lambda data: data)
        self.on_read = on_read or (lambda data: data)


class FakeSerial:
    """Drop-in replacement for ``pyserial.Serial`` backed by a :class:`Device`."""

    def __init__(
        self,
        device: Device,
        clock: Clock,
        transport: Transport | None = None,
    ) -> None:
        self._device = device
        self._clock = clock
        self._transport = transport or Transport()

        self.is_open = True
        self.timeout: float | None = 0
        self._dtr = True
        self._out_buffer = bytearray()

    @property
    def dtr(self) -> bool:
        return self._dtr

    @dtr.setter
    def dtr(self, value: bool) -> None:
        previous = self._dtr
        self._dtr = bool(value)
        if self._dtr != previous:
            self._device.on_dtr(self._dtr)

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        self._pull_from_device()
        self._out_buffer.clear()

    def reset_output_buffer(self) -> None:
        # Host->device direction has no queue: writes are delivered synchronously.
        return None

    def write(self, data: bytes) -> int:
        payload = self._transport.on_write(bytes(data))
        self._device.feed(payload)
        return len(data)

    def flush(self) -> None:
        return None

    def read(self, size: int = 1) -> bytes:
        self._pull_from_device()
        chunk = bytes(self._out_buffer[:size])
        del self._out_buffer[:size]
        return self._transport.on_read(chunk)

    def read_until(self, expected: bytes = b"\n", size: int = 1024) -> bytes:
        self._pull_from_device()
        index = self._out_buffer.find(expected)
        if index >= 0:
            end = min(index + len(expected), size)
        else:
            end = min(len(self._out_buffer), size)
        chunk = bytes(self._out_buffer[:end])
        del self._out_buffer[:end]
        return self._transport.on_read(chunk)

    def read_all(self) -> bytes:
        self._pull_from_device()
        chunk = bytes(self._out_buffer)
        self._out_buffer.clear()
        return self._transport.on_read(chunk)

    def _pull_from_device(self) -> None:
        self._out_buffer.extend(self._device.drain())


class SimSerialLine(SerialLine):
    """``SerialLine`` whose ``connect()`` opens a :class:`FakeSerial` port.

    This is the single injection point: the production ``SerialLine.connect``
    unconditionally constructs a real ``serial.Serial``, so the simulator
    subclasses it and installs the fake port instead. Everything else —
    query/read/reset/close, locking, state and close-meta bookkeeping — is the
    untouched production code, and the motors on top notice no difference.
    """

    def __init__(
        self,
        device: Device,
        clock: Clock,
        *,
        transport: Transport | None = None,
        port: str = "sim://device",
        baud: int = 115200,
        timeout_s: float = 0,
        name: str = "sim",
        terminator: str = "\r",
        encoding: str = "ascii",
    ) -> None:
        super().__init__(port, baud, timeout_s, name, terminator=terminator, encoding=encoding)
        self._sim_device = device
        self._sim_clock = clock
        self._sim_transport = transport

    def connect(self):
        with self._lock:
            serial_obj = self.serial
            if serial_obj is not None and getattr(serial_obj, "is_open", False):
                serial_obj.close()

            self.serial = FakeSerial(self._sim_device, self._sim_clock, self._sim_transport)
            self.serial.timeout = self.timeout_s
            self._state = SerialLineState.OPEN

            self.logger.info("Port: %s (simulated)", self.port)
