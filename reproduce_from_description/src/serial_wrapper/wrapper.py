from __future__ import annotations

import re
from dataclasses import dataclass
from threading import RLock
from time import monotonic, sleep
from typing import Any, Callable

try:
    import serial as pyserial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised indirectly through tests
    pyserial = None
    list_ports = None


class SerialTransportError(RuntimeError):
    pass


class SerialTimeoutError(SerialTransportError):
    pass


class SerialDisconnectedError(SerialTransportError):
    pass


@dataclass(frozen=True, slots=True)
class SerialDeviceInfo:
    device: str
    description: str
    hwid: str


def list_serial_ports() -> list[SerialDeviceInfo]:
    if list_ports is None:
        return []

    return [
        SerialDeviceInfo(device=port.device, description=getattr(port, "description", ""), hwid=getattr(port, "hwid", ""))
        for port in list_ports.comports()
    ]


class SerialLine:
    def __init__(
        self,
        port: str | None = None,
        search_pattern: str | None = None,
        baudrate: int = 9600,
        timeout: float = 1.0,
        terminator: bytes = b"\n",
        encoding: str = "ascii",
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.port = port
        self.search_pattern = search_pattern
        self.baudrate = baudrate
        self.timeout = timeout
        self.terminator = terminator
        self.encoding = encoding
        self.serial_factory = serial_factory or self._default_serial_factory
        self._lock = RLock()
        self._serial: Any | None = None

    @staticmethod
    def search(pattern: str) -> list[SerialDeviceInfo]:
        regex = re.compile(pattern)
        matches: list[SerialDeviceInfo] = []
        for port in list_serial_ports():
            candidate = " ".join(part for part in (port.device, port.description, port.hwid) if part)
            if regex.search(candidate):
                matches.append(port)
        return matches

    @staticmethod
    def find_first(pattern: str) -> str | None:
        matches = SerialLine.search(pattern)
        return matches[0].device if matches else None

    def connect(self) -> None:
        with self._lock:
            if self._serial is not None and getattr(self._serial, "is_open", True):
                return

            port = self.port or (self.find_first(self.search_pattern) if self.search_pattern else None)
            if not port:
                raise SerialTransportError("Serial device not found")

            self._serial = self.serial_factory(port=port, baudrate=self.baudrate, timeout=self.timeout)
            self.port = port

    def reconnect(self) -> None:
        with self._lock:
            self.close()
            self.connect()

    def reset(self, sleep_seconds: float = 0.1) -> None:
        with self._lock:
            serial_handle = self._require_connected()
            if hasattr(serial_handle, "reset_input_buffer"):
                serial_handle.reset_input_buffer()
            if hasattr(serial_handle, "reset_output_buffer"):
                serial_handle.reset_output_buffer()
            if hasattr(serial_handle, "dtr"):
                serial_handle.dtr = False
                sleep(sleep_seconds)
                serial_handle.dtr = True

    def drain(self) -> bytes:
        with self._lock:
            serial_handle = self._require_connected()
            buffer = bytearray()
            while True:
                waiting = getattr(serial_handle, "in_waiting", 0)
                if waiting:
                    buffer.extend(serial_handle.read(waiting))
                    continue

                chunk = serial_handle.read(1)
                if not chunk:
                    break
                buffer.extend(chunk)
            return bytes(buffer)

    def read(self, size: int = 1) -> bytes:
        with self._lock:
            serial_handle = self._require_connected()
            return bytes(serial_handle.read(size))

    def read_until(self, terminator: bytes | None = None, response_prefix: bytes | None = None, timeout: float | None = None) -> bytes:
        terminator = self.terminator if terminator is None else terminator
        deadline = monotonic() + (self.timeout if timeout is None else timeout)
        buffer = bytearray()
        with self._lock:
            serial_handle = self._require_connected()
            while monotonic() < deadline:
                chunk = serial_handle.read(1)
                if not chunk:
                    continue

                buffer.extend(chunk)
                if response_prefix:
                    prefix_index = buffer.find(response_prefix)
                    if prefix_index < 0:
                        continue
                    if prefix_index > 0:
                        del buffer[:prefix_index]

                if terminator is None:
                    return bytes(buffer)
                if buffer.endswith(terminator):
                    return bytes(buffer)

        raise SerialTimeoutError("Timed out while reading from serial device")

    def query(self, payload: bytes | str, terminator: bytes | None = None, response_prefix: bytes | None = None, timeout: float | None = None) -> bytes:
        with self._lock:
            serial_handle = self._require_connected()
            packet = payload.encode(self.encoding) if isinstance(payload, str) else payload
            serial_handle.write(packet)
            if hasattr(serial_handle, "flush"):
                serial_handle.flush()

        return self.read_until(terminator=terminator, response_prefix=response_prefix, timeout=timeout)

    def query_text(self, payload: str, terminator: bytes | None = None, response_prefix: str | None = None, timeout: float | None = None) -> str:
        prefix = response_prefix.encode(self.encoding) if response_prefix is not None else None
        raw = self.query(payload, terminator=terminator, response_prefix=prefix, timeout=timeout)
        return raw.decode(self.encoding).rstrip((terminator or self.terminator).decode(self.encoding))

    def close(self) -> None:
        with self._lock:
            if self._serial is None:
                return
            if hasattr(self._serial, "close"):
                self._serial.close()
            self._serial = None

    def _require_connected(self) -> Any:
        if self._serial is None:
            raise SerialDisconnectedError("Serial device is not connected")
        return self._serial

    @staticmethod
    def _default_serial_factory(**kwargs: Any) -> Any:
        if pyserial is None:
            raise SerialTransportError("pyserial is required to open a serial connection")
        return pyserial.Serial(**kwargs)
