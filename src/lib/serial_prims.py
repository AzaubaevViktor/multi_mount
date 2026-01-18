from __future__ import annotations
import logging
import threading
import time
from typing import Optional

try:
    import serial
except Exception as e:
    serial = None


class SerialLineDevice:
    def __init__(self, port: str, baud: int, timeout_s: float, name: str):
        self.log = logging.getLogger(name)
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
            deadline = time.time() + (self.ser.timeout or 1.0)
            while True:
                b = self.ser.read(1)
                if b:
                    buf += b
                    if buf.endswith(terminator):
                        self.log.debug("RX %r", bytes(buf))
                        return bytes(buf)
                else:
                    if time.time() >= deadline:
                        self.log.debug("RX TIMEOUT after %.3fs, got=%r", (self.ser.timeout or 0.0), bytes(buf))
                        raise TimeoutError(f"serial timeout, got={bytes(buf)!r}")
