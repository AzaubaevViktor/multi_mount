import functools
import logging
import os
import re
import threading
import time
import sys
import traceback
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Callable, TypeVar

import serial
from serial.serialutil import SerialException

from utils.method_call_chain import format_stack_frame, log_method_call_chain


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


class SerialLineClosedError(SerialLineError):
    pass


class SerialLineState(StrEnum):
    NEW = "new"
    OPEN = "open"
    CLOSED = "closed"


@dataclass(frozen=True)
class SerialLineCloseMeta:
    reason: str
    closed_from: str
    caller_frame: str | None
    error_type: str | None
    error_message: str | None
    at_monotonic_s: float


EXCEPTIONS_TO_CLOSE = (SerialException, SerialLineError)


T = TypeVar("T")


def _disconnect_when_error(default: T) -> Callable[[Callable[..., T]], Callable[..., T]]:
    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @functools.wraps(func)
        def wrapper(self: "SerialLine", *args: Any, **kwargs: Any) -> T:
            caller_frame = format_stack_frame(sys._getframe(1))
            try:
                return func(self, *args, **kwargs)
            except SerialLineClosedError as exc:
                self.logger.debug("Serial connection is closed, skip %s: %s", func.__name__, exc)
                return default
            except EXCEPTIONS_TO_CLOSE as exc:
                self.logger.exception("Error in %s, closing connection", func.__name__)
                self.close(
                    reason=f"error_in_{func.__name__}",
                    error=exc,
                    closed_from=f"{type(self).__name__}.{func.__name__}",
                    caller_frame=caller_frame,
                )
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

        self._lock = threading.RLock()

        self.serial: serial.Serial | Any | None = None
        self._state = SerialLineState.NEW
        self._last_close_meta: SerialLineCloseMeta | None = None

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
            serial_obj = self._require_open_serial()
            serial_obj.dtr = False
            time.sleep(0.1)
            self.drop_buffers()
            serial_obj.dtr = True
            time.sleep(0.5)
    
    def drop_buffers(self):
        with self._lock:
            serial_obj = self._require_open_serial()
            serial_obj.reset_input_buffer()
            serial_obj.reset_output_buffer()

    def connect(self):
        with self._lock:
            serial_obj = self.serial
            if serial_obj is not None and getattr(serial_obj, "is_open", False):
                serial_obj.close()

            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
            self._state = SerialLineState.OPEN

            self.logger.info("Port: %s", self.port)
            self.logger.info("Baudrate: %d", self.baud)
            self.logger.info("Timeout: %d", self.timeout_s)
            self.logger.info("Encoding: %s", self.encoding)
            self.logger.info("Terminator: %s", self.terminator)

    @_disconnect_when_error(default="")
    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        with self._lock:
            serial_obj = self._require_open_serial()
            if payload is not None:
                # self.logger.debug("Send `%r`", payload)
                serial_obj.reset_input_buffer()
                serial_obj.write(payload.encode(self.encoding))
                serial_obj.flush()
            else:
                # self.logger.debug("Just wait for answer")
                pass

            return self._read_line(
                timeout=timeout,
                response_prefixes=response_prefixes,
                response_terminator=response_terminator,
            )
    
    @_disconnect_when_error(default="")
    def _read_line(
        self,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        serial_obj = self._require_open_serial()
        terminator = response_terminator.encode(self.encoding) if isinstance(response_terminator, str) else response_terminator
        if terminator is None:
            terminator = self.terminator

        _timeout = serial_obj.timeout
        if timeout is not None:
            serial_obj.timeout = timeout
        try:
            if response_prefixes is None:
                line = serial_obj.read_until(terminator, 1024)
            else:
                skipped = bytearray()
                prefix = b""
                while True:
                    byte = serial_obj.read(1)
                    if not byte:
                        line = bytes(skipped)
                        break
                    if byte in response_prefixes:
                        prefix = byte
                        break
                    skipped.extend(byte)

                if prefix:
                    line = prefix + serial_obj.read_until(terminator, 1024)
        finally:
            if timeout is not None:
                serial_obj.timeout = _timeout

        responce = line.decode(self.encoding, errors="ignore")
        # self.logger.debug("Receive `%r`", responce)

        return responce
    
    @_disconnect_when_error(default=None)
    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        with self._lock:
            serial_obj = self._require_open_serial()
            _timeout = serial_obj.timeout
            if timeout is not None:
                serial_obj.timeout = timeout
            try:
                if (data := serial_obj.read_all()) is None:
                    return None
            finally:
                if timeout is not None:
                    serial_obj.timeout = _timeout
            
            lines = [line.decode(self.encoding, errors="ignore") for line in data.split(self.terminator)]

            self.logger.info("Receive all data from input:\n%s", lines)

        return lines

    @property
    def state(self) -> SerialLineState:
        return self._state

    @property
    def last_close_meta(self) -> SerialLineCloseMeta | None:
        return self._last_close_meta

    @log_method_call_chain(depth=None)
    def close(
        self,
        reason: str = "manual_close",
        error: BaseException | None = None,
        closed_from: str | None = None,
        caller_frame: str | None = None,
    ):
        with self._lock:
            if closed_from is None:
                caller = None
                for frame in reversed(traceback.extract_stack()[:-1]):
                    if frame.filename != __file__ and os.path.basename(frame.filename) != "method_call_chain.py":
                        caller = frame
                        break
                if caller is None:
                    closed_from = f"{type(self).__name__}.close"
                else:
                    closed_from = f"{os.path.basename(caller.filename)}:{caller.lineno} {caller.name}"

            if caller_frame is None:
                caller = None
                for frame in reversed(traceback.extract_stack()[:-1]):
                    if frame.filename != __file__ and os.path.basename(frame.filename) != "method_call_chain.py":
                        caller = frame
                        break
                if caller is None:
                    caller_frame = None
                else:
                    caller_frame = f"{caller.name} ({caller.filename}:{caller.lineno})"

            self._last_close_meta = SerialLineCloseMeta(
                reason=reason,
                closed_from=closed_from,
                caller_frame=caller_frame,
                error_type=type(error).__name__ if error is not None else None,
                error_message=str(error) if error is not None else None,
                at_monotonic_s=time.monotonic(),
            )

            serial_obj = self.serial
            if serial_obj is not None and getattr(serial_obj, "is_open", False):
                self.logger.debug("Close serial connection")
                serial_obj.close()

            self.serial = None
            self._state = SerialLineState.CLOSED

    def _require_open_serial(self) -> serial.Serial | Any:
        serial_obj = self.serial
        if serial_obj is None:
            raise SerialLineClosedError(self._format_closed_message())
        if hasattr(serial_obj, "is_open") and not serial_obj.is_open:
            self._state = SerialLineState.CLOSED
            raise SerialLineClosedError(self._format_closed_message())
        if self._state != SerialLineState.OPEN:
            self._state = SerialLineState.OPEN
        return serial_obj

    def _format_closed_message(self) -> str:
        if self._last_close_meta is None:
            return f"{self.port} is {self._state.value}"
        return (
            f"{self.port} is {self._state.value}; "
            f"reason={self._last_close_meta.reason}, "
            f"from={self._last_close_meta.closed_from}, "
            f"caller={self._last_close_meta.caller_frame}, "
            f"error={self._last_close_meta.error_type}:{self._last_close_meta.error_message}"
        )
