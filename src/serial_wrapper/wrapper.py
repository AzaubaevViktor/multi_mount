import functools
import logging
import os
import re
import threading
import sys
import traceback
from dataclasses import dataclass
from enum import StrEnum
from typing import Any, Callable, Concatenate, ParamSpec, TypeVar

import serial
from serial.serialutil import SerialException

from clock import REAL_CLOCK, Clock
from serial_wrapper.recorder import Recorder, TraceEvent, TraceKind
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


# A raw OSError (ENXIO/EIO/ENODEV on USB-serial unplug) means the device is gone just like a
# SerialException does: the line has to be closed and the caller has to degrade, not to die.
EXCEPTIONS_TO_CLOSE = (SerialException, SerialLineError, OSError)


_P = ParamSpec("_P")
_R = TypeVar("_R")
_D = TypeVar("_D")


# The decorated method keeps its own parameters (`Concatenate` pins the `self` the
# wrapper needs), and its result widens to `_R | _D`: on a closed line the wrapper
# answers with `default` instead of the value the method would have returned.
def _disconnect_when_error(
    default: _D, reraise_closed: bool = False
) -> Callable[[Callable[Concatenate["SerialLine", _P], _R]], Callable[Concatenate["SerialLine", _P], _R | _D]]:
    def decorator(
        func: Callable[Concatenate["SerialLine", _P], _R],
    ) -> Callable[Concatenate["SerialLine", _P], _R | _D]:
        @functools.wraps(func)
        def wrapper(self: "SerialLine", *args: _P.args, **kwargs: _P.kwargs) -> _R | _D:
            caller_frame = format_stack_frame(sys._getframe(1))
            try:
                return func(self, *args, **kwargs)
            except SerialLineClosedError as exc:
                if reraise_closed:
                    # Buffer draining is only ever called from a caller's own error path,
                    # where "the line is already closed" is the answer it needs, not a
                    # silent no-op that sends it into another retry round.
                    raise
                self.logger.debug("Serial connection is closed, skip %s: %s", func.__name__, exc)
                return default
            except EXCEPTIONS_TO_CLOSE as exc:
                self.logger.exception("Error in %s, closing connection", func.__name__)
                if self._recorder is not None:
                    self._record(
                        TraceKind.ERROR,
                        method=func.__name__,
                        error_type=type(exc).__name__,
                        error_message=str(exc),
                    )
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
    def __init__(self, port: str, baud: int, timeout_s: float, name: str, terminator: str = "\r", encoding: str ='ascii', clock: Clock = REAL_CLOCK, recorder: Recorder | None = None) -> None:
        self.logger = logging.getLogger(f"serial.{name}")
        self.name = name
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.encoding = encoding
        self.terminator = terminator.encode(self.encoding)
        self._clock = clock
        self._recorder = recorder

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
    
    def _record(self, kind: TraceKind, data: bytes = b"", **detail: Any) -> None:
        # Callers guard with `if self._recorder is not None` so that a line
        # without a recorder pays one attribute load per I/O and nothing else;
        # the check is repeated here only to keep the method safe on its own.
        recorder = self._recorder
        if recorder is None:
            return
        recorder.record(
            TraceEvent(
                at_s=self._clock.monotonic(),
                name=self.name,
                port=self.port,
                kind=kind,
                data=data,
                detail=detail,
            )
        )

    @_disconnect_when_error(default=None)
    def reset(self) -> None:
        with self._lock:
            serial_obj = self._require_open_serial()
            serial_obj.dtr = False
            if self._recorder is not None:
                self._record(TraceKind.RESET, dtr=False)
            self._clock.sleep(0.1)
            self.drop_buffers()
            serial_obj.dtr = True
            if self._recorder is not None:
                self._record(TraceKind.RESET, dtr=True)
            self._clock.sleep(0.5)
    
    # Without the decorator this was the one I/O path that could leave the line half-open:
    # an unplug landing on a buffer drain (both drivers call it from their retry handlers)
    # raised a bare OSError past the caller while `state` stayed OPEN — the same defect as
    # `d04b9b8`, one method further down.
    @_disconnect_when_error(default=None, reraise_closed=True)
    def drop_buffers(self) -> None:
        with self._lock:
            serial_obj = self._require_open_serial()
            serial_obj.reset_input_buffer()
            serial_obj.reset_output_buffer()
            if self._recorder is not None:
                self._record(TraceKind.DROP_BUFFERS)

    def connect(self) -> None:
        with self._lock:
            serial_obj = self.serial
            if serial_obj is not None and getattr(serial_obj, "is_open", False):
                serial_obj.close()

            self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
            self._state = SerialLineState.OPEN
            self._record_open()

            self.logger.info("Port: %s", self.port)
            self.logger.info("Baudrate: %d", self.baud)
            self.logger.info("Timeout: %d", self.timeout_s)
            self.logger.info("Encoding: %s", self.encoding)
            self.logger.info("Terminator: %s", self.terminator)

    def _record_open(self) -> None:
        """Shared by the real ``connect`` and by the simulator's override."""
        if self._recorder is not None:
            self._record(
                TraceKind.OPEN,
                baud=self.baud,
                timeout_s=self.timeout_s,
                encoding=self.encoding,
                terminator=self.terminator.hex(),
            )

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
                raw = payload.encode(self.encoding)
                serial_obj.reset_input_buffer()
                serial_obj.write(raw)
                serial_obj.flush()
                # Recorded after the flush: the trace must claim only bytes that
                # actually left the host, so a write that raises leaves an
                # `error` event here instead of a phantom `tx`.
                if self._recorder is not None:
                    self._record(TraceKind.TX, raw)
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
        # The port object is untyped by design (a real `serial.Serial` or the
        # simulator's fake), so the bytes it hands back are pinned down here
        # rather than leaking as `Any` into the decoded answer.
        line: bytes
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

        # The raw bytes, not `responce`: the decode drops anything the encoding
        # cannot represent, and a replay has to see exactly what came back.
        if self._recorder is not None:
            self._record(TraceKind.RX, line)

        responce = line.decode(self.encoding, errors="ignore")
        # self.logger.debug("Receive `%r`", responce)

        return responce
    
    # How often the drain loop below asks the OS whether more bytes have arrived. Short
    # enough that a device answering mid-poll is not made to wait noticeably, long enough
    # that a full-timeout drain is a handful of syscalls rather than a spin.
    _READ_ALL_POLL_S = 0.01

    @_disconnect_when_error(default=None)
    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        """Drain the input buffer, optionally waiting up to `timeout` for something to arrive.

        The waiting is done here, by hand, because `pyserial` cannot do it: `read_all()` is
        literally `read(self.in_waiting)`, so it returns whatever the OS has buffered *at this
        instant* and never looks at `serial.timeout`. The previous implementation set that
        attribute and then called a method that ignores it — `read_all_data(timeout=.5)`
        returned immediately, and when called right after `drop_buffers()` it could only ever
        return `['']`. That is the source of the 20 601 bare `['']` records in the March
        sessions (PLAN.md П8): not a device that stayed silent, but a signature that promised
        a wait nobody implemented.

        `timeout=None` keeps the old non-blocking meaning — a single drain of what is already
        there, which is what post-connect boot-garbage cleanup wants.
        """
        with self._lock:
            serial_obj = self._require_open_serial()
            deadline = self._clock.monotonic() + (timeout or 0.0)
            chunks = bytearray()
            while True:
                chunk: bytes | None = serial_obj.read_all()
                if chunk is None:
                    return None
                if chunk:
                    chunks.extend(chunk)
                elif chunks:
                    # Something arrived and then the line went quiet: the device has finished
                    # talking, so there is nothing to be gained by holding the lock longer.
                    break
                if self._clock.monotonic() >= deadline:
                    break
                self._clock.sleep(self._READ_ALL_POLL_S)

            data = bytes(chunks)
            if self._recorder is not None:
                self._record(TraceKind.RX, data)

            lines = [line.decode(self.encoding, errors="ignore") for line in data.split(self.terminator)]

            self.logger.info("Receive all data from input:\n%s", lines)

        return lines

    # How long a post-error drain waits for a device that may still be talking. Half a
    # second is what both drivers used, and it is above the slowest answer either board
    # was measured giving (RA: 4.7 ms, §9; DEC: one 255-byte TX ring at 115 200 baud).
    _DRAIN_TIMEOUT_S = 0.5

    def drain_after_error(self, logger: logging.Logger, reason: str) -> None:
        """Read whatever is left on the line, log it, and only then drop the buffers.

        The order is the whole point, and it belongs here rather than in the
        drivers: it is a property of the transport, and both drivers had their
        own copy of it. The wrong order -- `drop_buffers()` and then a read that
        can only come back empty -- is what produced the 20 601 records of `['']`
        in the March sessions: the bytes that confused the parser were thrown
        away before anyone looked at them, and the read that followed could only
        wait half a second for a board that had already said everything. It was
        found and fixed in the RA driver, and the copy in the DEC driver kept it
        for months afterwards, long after the two protocols had stopped having
        anything else in common. One copy cannot drift from the other.

        The caller's logger is passed in on purpose: the line that comes out
        names the driver that hit the error, not the port that carried it.

        Not decorated with :func:`_disconnect_when_error`: the two calls below
        carry their own decorators, and their difference is deliberate -- a
        closed line makes the read a logged no-op, while ``drop_buffers``
        re-raises, which is the answer a retry loop needs.
        """
        data = self.read_all_data(timeout=self._DRAIN_TIMEOUT_S)
        if data:
            logger.info("Discarding %d leftover byte-groups after %s: %s", len(data), reason, data)
        self.drop_buffers()

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
    ) -> None:
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
                at_monotonic_s=self._clock.monotonic(),
            )

            serial_obj = self.serial
            if serial_obj is not None and getattr(serial_obj, "is_open", False):
                self.logger.debug("Close serial connection")
                try:
                    serial_obj.close()
                except EXCEPTIONS_TO_CLOSE:
                    # close() is what every error path runs, and the port object is being
                    # dropped anyway: raising from here would replace the failure that
                    # caused the close *and* leave the line half-open, which is the one
                    # state this class exists to prevent.
                    self.logger.exception("Error while closing the port, dropping it anyway")

            self.serial = None
            self._state = SerialLineState.CLOSED

            if self._recorder is not None:
                meta = self._last_close_meta
                self._record(
                    TraceKind.CLOSE,
                    reason=meta.reason,
                    closed_from=meta.closed_from,
                    caller_frame=meta.caller_frame,
                    error_type=meta.error_type,
                    error_message=meta.error_message,
                )

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
