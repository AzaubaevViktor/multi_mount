"""Machine-readable byte trace of a serial session.

The text log in ``SerialLine`` is written for a human reading a session
afterwards; this module is the other half — a stream meant to be replayed by a
program. A trace taken from the real mount is the only evidence that
``src/sim`` (which was reconstructed *from logs*, i.e. from a guess about the
protocol) answers the way the hardware does: replay the recorded ``tx`` bytes
into the simulator and compare its answers with the recorded ``rx`` bytes.

Format: JSON Lines, one object per event, appended in chronological order.

    {"at_s":0.0,"name":"sw","port":"/dev/ttyUSB0","kind":"open","data":"","detail":{...}}
    {"at_s":0.01,"name":"sw","port":"/dev/ttyUSB0","kind":"tx","data":"3a663130310d"}
    {"at_s":0.03,"name":"sw","port":"/dev/ttyUSB0","kind":"rx","data":"3d0d"}

``data`` is hex rather than an escaped text encoding on purpose. The two lines
carry different alphabets (SkyWatcher is ASCII-framed, the TMC firmware is
line-based text) and a broken line delivers arbitrary garbage, including NUL
and bytes above 0x7F; hex is the only encoding that is fixed-width, unambiguous
for all 256 values and free of any decode/normalisation step on the way back,
so ``bytes.fromhex(row["data"])`` is a lossless inverse by construction.

Events are not only about bytes: ``open`` / ``close`` / ``reset`` /
``drop_buffers`` / ``error`` mark where the session was interrupted, which is
exactly what makes a replay reproduce a *failed* run and not just a happy one.

``mark`` carries no bytes at all and is written by the *caller*, not by
``SerialLine``: a recorded scenario (``src/tools/hw_session.py``) needs the
reader to be able to say which step a given frame belongs to, and byte
boundaries alone cannot answer that — the same ``:f1\\r`` appears in every step.
"""

import json
import threading
from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any, BinaryIO, Protocol, runtime_checkable


class TraceKind(StrEnum):
    TX = "tx"
    RX = "rx"
    OPEN = "open"
    CLOSE = "close"
    RESET = "reset"
    DROP_BUFFERS = "drop_buffers"
    ERROR = "error"
    MARK = "mark"


@dataclass(frozen=True)
class TraceEvent:
    """One thing that happened on one line, at one instant of the line's clock.

    ``at_s`` comes from the ``Clock`` injected into ``SerialLine``, never from
    ``time.monotonic()`` directly, so a trace produced under a virtual clock is
    replayable on that same virtual timeline.
    """

    at_s: float
    name: str
    port: str
    kind: TraceKind
    data: bytes = b""
    detail: dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class Recorder(Protocol):
    """Sink for trace events, injected into ``SerialLine`` like ``Clock``."""

    def record(self, event: TraceEvent) -> None:
        """Append one event to the trace."""
        ...

    def flush(self) -> None:
        """Push everything buffered so far out of the process."""
        ...

    def close(self) -> None:
        """Finish the trace; further ``record`` calls are ignored."""
        ...


class NullRecorder:
    """Explicit "record nothing" sink, for call sites that want a non-optional
    recorder. ``SerialLine`` defaults to ``None`` instead and skips the call
    altogether, so recording costs nothing at all when it is switched off."""

    def record(self, event: TraceEvent) -> None:
        pass

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


NULL_RECORDER = NullRecorder()

# Session-ending events are flushed immediately: they are the ones a crashed or
# killed process must still leave on disk, and there are only a handful of them
# per session, so the cost is irrelevant. Everything else rides the file
# buffer.
DEFAULT_FLUSH_KINDS: frozenset[TraceKind] = frozenset(
    {TraceKind.OPEN, TraceKind.CLOSE, TraceKind.ERROR}
)

_BUFFER_BYTES = 1 << 16


class JsonlRecorder:
    """JSON Lines trace writer, safe to share between the RA and DEC lines.

    Buffering: writes go into an ordinary 64 KiB file buffer and are *not*
    fsync'd. A tracked session is minutes long and every write happens inside
    ``SerialLine``'s lock, so an fsync per line would inject millisecond stalls
    into the exact inter-byte timing the trace exists to capture — the
    measurement would change what it measures. Flushing (a userspace->kernel
    copy, no disk wait) on session-ending events keeps the interesting tail
    intact across a process crash; only a machine-level power loss can eat the
    buffered middle, and a trace is reproducible evidence, not a ledger.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        flush_kinds: Iterable[TraceKind] = DEFAULT_FLUSH_KINDS,
        buffer_bytes: int = _BUFFER_BYTES,
    ) -> None:
        self.path = Path(path)
        self._flush_kinds = frozenset(flush_kinds)
        # Binary append: json.dumps(ensure_ascii=True) already yields pure ASCII,
        # and a binary handle cannot re-translate newlines inside the payload.
        # `None` is the closed state: `close()` drops the handle and every later
        # call becomes a no-op instead of raising on the serial path.
        self._stream: BinaryIO | None = self.path.open("ab", buffering=buffer_bytes)
        self._lock = threading.Lock()

    def record(self, event: TraceEvent) -> None:
        row = encode_event(event)
        with self._lock:
            stream = self._stream
            if stream is None:
                # Recording is diagnostics: a trace closed early (or twice) must
                # never turn into an exception on the serial path.
                return
            stream.write(row)
            if event.kind in self._flush_kinds:
                stream.flush()

    def flush(self) -> None:
        with self._lock:
            if self._stream is not None:
                self._stream.flush()

    def close(self) -> None:
        with self._lock:
            if self._stream is None:
                return
            self._stream.close()
            self._stream = None

    def __enter__(self) -> "JsonlRecorder":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def encode_event(event: TraceEvent) -> bytes:
    row: dict[str, Any] = {
        "at_s": event.at_s,
        "name": event.name,
        "port": event.port,
        "kind": str(event.kind),
        "data": event.data.hex(),
    }
    if event.detail:
        row["detail"] = event.detail
    # `default=str` so an unexpected value in `detail` degrades to its repr
    # instead of aborting the write in the middle of a session.
    return (json.dumps(row, ensure_ascii=True, separators=(",", ":"), default=str) + "\n").encode("ascii")


def decode_event(row: dict[str, Any]) -> TraceEvent:
    return TraceEvent(
        at_s=float(row["at_s"]),
        name=row["name"],
        port=row["port"],
        kind=TraceKind(row["kind"]),
        data=bytes.fromhex(row["data"]),
        detail=dict(row.get("detail", {})),
    )


def iter_trace(path: str | Path) -> Iterator[TraceEvent]:
    """Stream a trace back in write order — the entry point for a replay run."""
    with Path(path).open("rb") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line:
                continue
            yield decode_event(json.loads(line))


def read_trace(path: str | Path) -> list[TraceEvent]:
    return list(iter_trace(path))
