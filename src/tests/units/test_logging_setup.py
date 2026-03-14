import io
import logging

import logging_setup
from logging_setup import LockedStreamHandler


class _TrackingLock:
    def __init__(self) -> None:
        self.enter_count = 0
        self.exit_count = 0

    def __enter__(self) -> "_TrackingLock":
        self.enter_count += 1
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.exit_count += 1


def test_locked_stream_handler_uses_terminal_output_lock(monkeypatch) -> None:
    stream = io.StringIO()
    handler = LockedStreamHandler(stream)
    handler.setFormatter(logging.Formatter("%(message)s"))
    lock = _TrackingLock()

    monkeypatch.setattr(logging_setup, "TERMINAL_OUTPUT_LOCK", lock)

    record = logging.LogRecord(
        name="test",
        level=logging.INFO,
        pathname=__file__,
        lineno=1,
        msg="hello",
        args=(),
        exc_info=None,
    )

    handler.emit(record)

    assert stream.getvalue() == "hello\n"
    assert lock.enter_count == 1
    assert lock.exit_count == 1
