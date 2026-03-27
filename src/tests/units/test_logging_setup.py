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


def test_setup_logging_can_disable_stream_handler(tmp_path) -> None:
    root_logger = logging.getLogger()
    existing_handlers = list(root_logger.handlers)

    try:
        logging_setup.setup_logging(
            logs_root=str(tmp_path),
            base_path=str(tmp_path),
            stream_level=None,
        )

        assert not any(
            handler.name == f"{logging_setup.HANDLER_NAME_PREFIX}_stream"
            for handler in root_logger.handlers
        )
        assert any(
            handler.name == f"{logging_setup.HANDLER_NAME_PREFIX}_debug"
            for handler in root_logger.handlers
        )
    finally:
        for handler in list(root_logger.handlers):
            if handler not in existing_handlers:
                root_logger.removeHandler(handler)
                handler.close()
