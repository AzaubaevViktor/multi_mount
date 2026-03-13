import functools
import logging
import os
import re
import threading
import time
from typing import Any, Callable

import serial
from serial.serialutil import SerialException


class SerialLineError(Exception):
    pass


class SerialLineSearchError(SerialLineError):
    pass


class SerialLineSearchInvalidPattern(SerialLineSearchError):
    pass


class SerialLineSearchDirectoryError(SerialLineSearchError):
    pass


class SerialLineSearchNotFound(SerialLineSearchError):
    pass


EXCEPTIONS_TO_CLOSE = (SerialException, SerialLineError)


def _disconnect_when_error(default: Any) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @functools.wraps(func)
        def wrapper(self: "SerialLine", *args: Any, **kwargs: Any) -> Any:
            try:
                return func(self, *args, **kwargs)
            except EXCEPTIONS_TO_CLOSE:
                self.logger.exception("Error in %s, closing connection", func.__name__)
                self.close()
                raise
            except AttributeError as exc:
                if "object has no attribute 'serial'" in str(exc):
                    self.logger.debug("Serial connection is closed, skip error: %s", exc)
                    return default
                raise

        return wrapper

    return decorator


class SerialLine:
    def __init__(self, port: str, baud: int, timeout_s: float, name: str, terminator: str = "\r", encoding: str ='ascii') -> None:
        self.logger = logging.getLogger(f"serial.{name}")
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.encoding = encoding
        self.terminator = terminator.encode(self.encoding)

        self._lock = threading.Lock()

        self.serial: serial.Serial

    @classmethod
    def search(cls, pattern: str, directory: str = "/dev") -> str:
        if not pattern:
            raise SerialLineSearchError("pattern is required")
        if not directory:
            raise SerialLineSearchError("directory is required")

        try:
            regex = re.compile(pattern)
        except re.error as exc:
            raise SerialLineSearchInvalidPattern(
                f"invalid search pattern: {pattern!r}"
            ) from exc

        try:
            with os.scandir(directory) as entries:
                for entry in entries:
                    if regex.search(entry.name):
                        return entry.path
        except OSError as exc:
            raise SerialLineSearchDirectoryError(
                f"cannot read directory: {directory!r}"
            ) from exc

        raise SerialLineSearchNotFound(
            f"no match for pattern {pattern!r} in {directory!r}"
        )
    
    @_disconnect_when_error(default=None)
    def reset(self):
        with self._lock:
            self.serial.dtr = False
            time.sleep(0.1)
            self.drop_buffers()
            self.serial.dtr = True
            time.sleep(0.5)
    
    def drop_buffers(self):
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def connect(self):
        serial_obj = getattr(self, "serial", None)
        if serial_obj is not None and serial_obj.is_open:
            serial_obj.close()
        self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
        self.logger.info("Connected to %s:%s (timeout=%d)", self.port, self.baud, self.timeout_s)

    @_disconnect_when_error(default="")
    def query(self, payload: str | None, timeout: int | None = None) -> str:
        with self._lock:
            if payload is not None:
                # self.logger.debug("Send `%r`", payload)
                self.serial.reset_input_buffer()
                self.serial.write(payload.encode(self.encoding))
                self.serial.flush()
            else:
                # self.logger.debug("Just wait for answer")
                pass

            return self._read_line(timeout=timeout)
    
    @_disconnect_when_error(default="")
    def _read_line(self, timeout: int | None = None) -> str:
        _timeout = self.serial.timeout
        if timeout is not None:
            self.serial.timeout = timeout
        try:
            line = self.serial.read_until(self.terminator, 1024)
        finally:
            if timeout is not None:
                self.serial.timeout = _timeout

        responce = line.decode(self.encoding, errors="ignore")
        # self.logger.debug("Receive `%r`", responce)

        return responce
    
    @_disconnect_when_error(default=None)
    def read_all_data(self, timeout: int | None = None) -> list[str] | None:
        with self._lock:
            _timeout = self.serial.timeout
            if timeout is not None:
                self.serial.timeout = timeout
            try:
                if (data := self.serial.read_all()) is None:
                    return None
            finally:
                if timeout is not None:
                    self.serial.timeout = _timeout
            
            lines = [line.decode(self.encoding, errors="ignore") for line in data.split(self.terminator)]

            self.logger.info("Receive all data from input:\n%s", lines)

        return lines

    def close(self):
        if hasattr(self, "serial"):
            serial_obj = self.serial

            if serial_obj is not None and serial_obj.is_open:
                self.logger.debug("Close serial connection")
                serial_obj.close()

            del self.serial
