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
synchronously appends its reply to the output buffer, so reads never block in
real time. A read that cannot be satisfied instead spends the port ``timeout``
on the virtual :class:`~sim.clock.Clock`, which is what keeps the timeout-driven
wait loops in ``SerialLine`` and in the drivers terminating.

Failure fidelity
----------------
The whole degradation path of the project keys off the exceptions a real port
raises (``EXCEPTIONS_TO_CLOSE`` -> ``SerialLine._disconnect_when_error`` ->
``SerialLine.close()``), so the fake reproduces them exactly:

- every I/O on a closed port raises ``PortNotOpenError`` ("Attempting to use a
  port that is not open"), like ``serial.serialposix.Serial``;
- assigning ``dtr`` on a closed port is *not* an error in pyserial (the level is
  only cached and applied on open), so the fake caches it too and, crucially,
  does not deliver the edge to the device;
- :meth:`FakeSerial.unplug` models the USB-serial device vanishing mid-session
  (PLAN.md §1 П2): ``is_open`` stays true — the host cannot notice until the
  next syscall — and from then on every I/O raises
  ``OSError(ENXIO, "Device not configured")``, the error that ended both real
  sessions.

Chaos seam
----------
All host<->device byte flow passes through :class:`Transport`, whose ``on_write``
and ``on_read`` hooks are the injection points for the future chaos stage
(partial writes, byte loss). They default to identity, and combining them with
:meth:`FakeSerial.unplug` gives "connection drop mid-response". Cheap scripted
faults required *now* by the red P1/P3 tests live on the device via
:class:`FaultScript`, not here.
"""

import errno
from typing import Callable, Protocol

from serial.serialutil import PortNotOpenError
from serial_wrapper.recorder import Recorder
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
        self._unplugged = False

    @property
    def dtr(self) -> bool:
        return self._dtr

    @dtr.setter
    def dtr(self, value: bool) -> None:
        previous = self._dtr
        self._dtr = bool(value)
        if not self.is_open:
            # pyserial caches the level while the port is closed and drives the
            # line on open, so the device sees no edge here.
            return
        if self._unplugged:
            raise OSError(errno.ENXIO, "Device not configured")
        if self._dtr != previous:
            self._device.on_dtr(self._dtr)

    def unplug(self) -> None:
        """Make the device vanish from the OS the way a yanked USB cable does.

        ``is_open`` deliberately stays true: the host learns about the unplug
        only from the next failing syscall, which is what the drivers and
        ``SerialLine`` have to survive.
        """
        self._unplugged = True

    @property
    def is_unplugged(self) -> bool:
        """Whether the device behind this port is gone (no pyserial counterpart).

        A real port cannot answer this — that is the whole point of the failure
        mode — but a test needs it to state the half-open invariant: a line whose
        state is OPEN must never be holding a port that is already dead.
        """
        return self._unplugged

    def close(self) -> None:
        # Closing is the one operation that must stay silent even on a dead
        # device: it is exactly what SerialLine runs while handling the error.
        self.is_open = False

    def reset_input_buffer(self) -> None:
        self._require_usable()
        self._pull_from_device()
        self._out_buffer.clear()

    def reset_output_buffer(self) -> None:
        # Host->device direction has no queue: writes are delivered synchronously.
        self._require_usable()

    def write(self, data: bytes) -> int:
        self._require_usable()
        payload = self._transport.on_write(bytes(data))
        self._device.feed(payload)
        # pyserial reports how many bytes actually went out, so a partial-write
        # hook shows up here rather than being silently rounded up to len(data).
        return len(payload)

    def flush(self) -> None:
        self._require_usable()

    def read(self, size: int = 1) -> bytes:
        self._require_usable()
        self._pull_from_device()
        if len(self._out_buffer) < size:
            self._wait_out_timeout()
        chunk = bytes(self._out_buffer[:size])
        del self._out_buffer[:size]
        return self._transport.on_read(chunk)

    def read_until(self, expected: bytes = b"\n", size: int = 1024) -> bytes:
        self._require_usable()
        self._pull_from_device()
        if self._out_buffer.find(expected) < 0:
            self._wait_out_timeout()
        index = self._out_buffer.find(expected)
        if index >= 0:
            end = min(index + len(expected), size)
        else:
            end = min(len(self._out_buffer), size)
        chunk = bytes(self._out_buffer[:end])
        del self._out_buffer[:end]
        return self._transport.on_read(chunk)

    def read_all(self) -> bytes:
        self._require_usable()
        self._pull_from_device()
        chunk = bytes(self._out_buffer)
        self._out_buffer.clear()
        return self._transport.on_read(chunk)

    def _require_usable(self) -> None:
        if not self.is_open:
            raise PortNotOpenError()
        if self._unplugged:
            raise OSError(errno.ENXIO, "Device not configured")

    def _pull_from_device(self) -> None:
        self._out_buffer.extend(self._device.drain())

    def _wait_out_timeout(self) -> None:
        # A real port returns a short read only after it has blocked for the
        # whole timeout, and that elapsed time is exactly what makes the wait
        # loops in the drivers terminate. Burn it on the virtual clock, then
        # look again: the device may have produced something meanwhile.
        self._clock.advance(self.timeout or 0.0)
        self._pull_from_device()


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
        recorder: Recorder | None = None,
    ) -> None:
        super().__init__(
            port, baud, timeout_s, name, terminator=terminator, encoding=encoding, clock=clock, recorder=recorder
        )
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
            self._record_open()

            self.logger.info("Port: %s (simulated)", self.port)
