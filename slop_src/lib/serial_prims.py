from __future__ import annotations
import logging
import threading
import time
from typing import Optional

try:
    import serial
except Exception as e:
    serial = None


class SerialLineDeviceConstants:
    READ_SIZE = 1
    DEFAULT_TIMEOUT_S = 1.0
    MIN_IDLE_TIMEOUT_S = 0.001
    EMPTY_READ = b""


class SerialLineDevice:
    def __init__(self, port: str, baud: int, timeout_s: float, name: str):
        self.log = logging.getLogger(name)
        self.log.setLevel(logging.DEBUG)
        self.lock = threading.Lock()
        if serial is None:
            self.log.error("pyserial not available: install with 'pip install pyserial'")
            raise ImportError("pyserial is required for SerialLineDevice")
        try:
            self.log.info("Opening serial port %s @ %d baud (timeout=%.3fs)", port, baud, timeout_s)
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout_s)
            self.log.info("Serial port %s opened", port)
        except Exception:
            self.log.exception("Failed to open serial port %s", port)
            raise

    def close(self) -> None:
        with self.lock:
            try:
                self.ser.close()
            except Exception:
                pass

    def transact(self, payload: bytes, terminator: bytes) -> bytes:
        """Write payload, then read until terminator (inclusive)."""
        with self.lock:
            self.log.debug("TX %r", payload)
            self.ser.reset_input_buffer()
            self.ser.write(payload)
            self.ser.flush()

            buf = bytearray()
            deadline = time.time() + (self.ser.timeout or SerialLineDeviceConstants.DEFAULT_TIMEOUT_S)
            while True:
                b = self.ser.read(SerialLineDeviceConstants.READ_SIZE)
                if b:
                    buf += b
                    if buf.endswith(terminator):
                        self.log.debug("RX %r", bytes(buf))
                        return bytes(buf)
                else:
                    if time.time() >= deadline:
                        self.log.debug(
                            "RX TIMEOUT after %.3fs, got=%r",
                            (self.ser.timeout or 0.0),
                            bytes(buf),
                        )
                        raise TimeoutError(f"serial timeout, got={bytes(buf)!r}")

    def transact_lines(self, payload: bytes, terminator: bytes, idle_timeout_s: float) -> list[bytes]:
        """Write payload, then read lines until idle timeout (inclusive terminator on each line)."""
        if idle_timeout_s < SerialLineDeviceConstants.MIN_IDLE_TIMEOUT_S:
            raise ValueError("idle_timeout_s must be positive")
        with self.lock:
            self.log.debug("TX %r", payload)
            self.ser.reset_input_buffer()
            self.ser.write(payload)
            self.ser.flush()
            return self._read_lines_until_idle_locked(terminator, idle_timeout_s)

    def read_lines_until_idle(self, terminator: bytes, idle_timeout_s: float) -> list[bytes]:
        """Read lines until idle timeout without writing."""
        if idle_timeout_s < SerialLineDeviceConstants.MIN_IDLE_TIMEOUT_S:
            raise ValueError("idle_timeout_s must be positive")
        with self.lock:
            return self._read_lines_until_idle_locked(terminator, idle_timeout_s)

    def _read_lines_until_idle_locked(self, terminator: bytes, idle_timeout_s: float) -> list[bytes]:
        lines: list[bytes] = []
        buf = bytearray()
        last_rx = time.monotonic()
        while True:
            b = self.ser.read(SerialLineDeviceConstants.READ_SIZE)
            now = time.monotonic()
            if b and b != SerialLineDeviceConstants.EMPTY_READ:
                last_rx = now
                buf += b
                if buf.endswith(terminator):
                    lines.append(bytes(buf))
                    buf.clear()
            else:
                if (now - last_rx) >= idle_timeout_s:
                    if buf:
                        lines.append(bytes(buf))
                    return lines
