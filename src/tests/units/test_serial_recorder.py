"""Byte-level session recorder on `SerialLine` (see src/serial_wrapper/recorder.py).

The trace exists to be replayed against the simulator, so what is asserted here
is exactly what a replay depends on: the same bytes, in the same order, with the
right direction, on a timeline taken from the line's injected clock — plus the
events that say where the session broke.

Everything runs on the simulated port; no hardware is involved.
"""

import json
import threading

import pytest

from serial_wrapper.recorder import (
    NULL_RECORDER,
    JsonlRecorder,
    TraceEvent,
    TraceKind,
    iter_trace,
    read_trace,
)
from sim import Clock, SimSerialLine


class _EchoDevice:
    """Answers every write with its upper-cased copy."""

    def __init__(self) -> None:
        self._out = bytearray()

    def feed(self, data: bytes) -> None:
        self._out.extend(data.upper())

    def drain(self) -> bytes:
        data = bytes(self._out)
        self._out.clear()
        return data

    def on_dtr(self, level: bool) -> None:
        pass


class _RawEchoDevice(_EchoDevice):
    """Answers with the bytes verbatim, whatever they are."""

    def feed(self, data: bytes) -> None:
        self._out.extend(data)


def _line(device=None, *, clock=None, encoding="ascii", name="rec", recorder=None):
    return SimSerialLine(
        device if device is not None else _EchoDevice(),
        clock if clock is not None else Clock(),
        port=f"sim://{name}",
        timeout_s=0,
        name=name,
        terminator="\r",
        encoding=encoding,
        recorder=recorder,
    )


def test_trace_holds_the_exact_bytes_in_order_with_directions(tmp_path) -> None:
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    line = _line(recorder=recorder)

    line.connect()
    assert line.query("ab\r") == "AB\r"
    assert line.query("cd\r") == "CD\r"
    line.close()
    recorder.close()

    events = read_trace(path)
    assert [(event.kind, event.data) for event in events] == [
        (TraceKind.OPEN, b""),
        (TraceKind.TX, b"ab\r"),
        (TraceKind.RX, b"AB\r"),
        (TraceKind.TX, b"cd\r"),
        (TraceKind.RX, b"CD\r"),
        (TraceKind.CLOSE, b""),
    ]
    assert {event.name for event in events} == {"rec"}
    assert {event.port for event in events} == {"sim://rec"}


def test_timestamps_come_from_the_injected_clock(tmp_path) -> None:
    """Wall-clock stamps would make a trace unreplayable on a virtual clock."""
    path = tmp_path / "trace.jsonl"
    clock = Clock(start_s=100.0)
    recorder = JsonlRecorder(path)
    line = _line(clock=clock, recorder=recorder)

    line.connect()
    line.query("ab\r")
    clock.advance(5.5)
    line.query("cd\r")
    recorder.close()

    stamps = [event.at_s for event in read_trace(path) if event.kind is TraceKind.TX]
    assert stamps == [100.0, 105.5]


def test_bytes_survive_the_trace_unchanged(tmp_path) -> None:
    """Every value 0x00..0xFF, through the line and back out of the file."""
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    # latin-1: the only str<->bytes mapping that carries 0x80..0xFF through
    # `SerialLine`'s own encode/decode, so the payload reaches the port intact.
    line = _line(_RawEchoDevice(), encoding="latin-1", recorder=recorder)

    payload = bytes(value for value in range(256) if value != 0x0D) + b"\r"
    line.connect()
    line.query(payload.decode("latin-1"))
    recorder.close()

    events = read_trace(path)
    assert [event.data for event in events if event.kind is TraceKind.TX] == [payload]
    assert [event.data for event in events if event.kind is TraceKind.RX] == [payload]
    assert b"\x00" in payload and b"\xff" in payload


def test_recorder_roundtrips_an_event_verbatim(tmp_path) -> None:
    path = tmp_path / "trace.jsonl"
    event = TraceEvent(
        at_s=12.25,
        name="dec",
        port="/dev/ttyUSB1",
        kind=TraceKind.RX,
        data=bytes(range(256)),
        detail={"note": "binary garbage"},
    )

    with JsonlRecorder(path) as recorder:
        recorder.record(event)

    assert read_trace(path) == [event]
    assert list(iter_trace(path)) == [event]


def test_on_disk_format_is_one_json_object_per_event_with_hex_data(tmp_path) -> None:
    """Pins the format a replay tool will parse."""
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    line = _line(recorder=recorder)

    line.connect()
    line.query("ab\r")
    recorder.close()

    rows = [json.loads(raw) for raw in path.read_text(encoding="ascii").splitlines()]
    assert [row["kind"] for row in rows] == ["open", "tx", "rx"]
    assert rows[1]["data"] == "61620d"
    assert rows[2]["data"] == "41420d"
    assert rows[1]["name"] == "rec"
    assert rows[1]["port"] == "sim://rec"


def test_reset_and_buffer_drops_are_recorded(tmp_path) -> None:
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    line = _line(recorder=recorder)

    line.connect()
    line.reset()
    line.read_all_data()
    recorder.close()

    events = read_trace(path)
    assert [event.kind for event in events] == [
        TraceKind.OPEN,
        TraceKind.RESET,
        TraceKind.DROP_BUFFERS,
        TraceKind.RESET,
        TraceKind.RX,
    ]
    assert events[1].detail["dtr"] is False
    assert events[3].detail["dtr"] is True


def test_a_line_without_a_recorder_writes_nothing(tmp_path) -> None:
    line = _line(recorder=None)

    line.connect()
    assert line.query("ab\r") == "AB\r"
    line.reset()
    line.read_all_data()
    line.close()

    assert list(tmp_path.iterdir()) == []


def test_null_recorder_changes_nothing(tmp_path) -> None:
    line = _line(recorder=NULL_RECORDER)

    line.connect()
    assert line.query("ab\r") == "AB\r"
    line.close()

    assert list(tmp_path.iterdir()) == []


def test_the_break_is_in_the_trace_and_on_disk_without_closing_it(tmp_path) -> None:
    """An unplug ends the session; a trace that lost its tail proves nothing."""
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    line = _line(recorder=recorder)

    line.connect()
    line.query("ab\r")
    line.serial.unplug()

    with pytest.raises(OSError, match="Device not configured"):
        line.query("cd\r")

    # Deliberately not closed: session-ending events are flushed as they happen.
    events = read_trace(path)
    assert [event.kind for event in events] == [
        TraceKind.OPEN,
        TraceKind.TX,
        TraceKind.RX,
        TraceKind.ERROR,
        TraceKind.CLOSE,
    ]
    assert events[3].detail["method"] == "query"
    assert events[3].detail["error_type"] == "OSError"
    assert "Device not configured" in events[3].detail["error_message"]
    assert events[4].detail["reason"] == "error_in_query"

    recorder.close()


def test_recording_never_breaks_the_session_after_the_trace_is_closed(tmp_path) -> None:
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    line = _line(recorder=recorder)
    line.connect()

    recorder.close()

    assert line.query("ab\r") == "AB\r"
    assert [event.kind for event in read_trace(path)] == [TraceKind.OPEN]


def test_one_recorder_shared_by_two_lines_never_interleaves_a_row(tmp_path) -> None:
    """The RA and DEC lines write into one trace from their own axis threads.

    Nothing else in this file runs concurrently, so without this the guarantee
    the recorder advertises -- that it is safe to share -- would rest on a lock
    no test defends. Removing that lock and splitting the row into two writes
    corrupts ~10% of the rows here while every other test stays green.
    """
    path = tmp_path / "trace.jsonl"
    recorder = JsonlRecorder(path)
    writers = 4
    rows_each = 2000
    # A payload long enough that a non-atomic write is very likely to be
    # preempted mid-row rather than slipping through by luck.
    payload = bytes(range(256)) * 4

    def hammer(name: str) -> None:
        for index in range(rows_each):
            recorder.record(
                TraceEvent(at_s=float(index), name=name, port="p", kind=TraceKind.TX, data=payload)
            )

    threads = [threading.Thread(target=hammer, args=(f"line{n}",)) for n in range(writers)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    recorder.close()

    events = read_trace(path)
    assert len(events) == writers * rows_each
    assert all(event.data == payload for event in events)
    assert {event.name for event in events} == {f"line{n}" for n in range(writers)}
