import logging
import os
import time
from datetime import datetime
from typing import Iterable


LOG_DIR_FORMAT = "%Y-%m-%d_%H-%M-%S"
HANDLER_NAME_PREFIX = "multi_mount_logging_setup"


class LoggingSetupError(Exception):
    pass


def _normalize_subsystem(name: str) -> str:
    return name.strip().lower()


def _subsystem_from_logger(logger_name: str) -> str:
    subsystem = logger_name.split(".", 1)[0].strip().lower()
    return subsystem or "root"


def _safe_relpath(path: str, base_path: str) -> str:
    try:
        return os.path.relpath(path, base_path)
    except ValueError:
        return path


class SubsystemFileHandler(logging.Handler):
    def __init__(self, log_dir: str, formatter: logging.Formatter) -> None:
        super().__init__(level=logging.DEBUG)
        if not log_dir or not str(log_dir).strip():
            raise LoggingSetupError("log_dir must not be empty.")
        self._log_dir = log_dir
        self._formatter = formatter
        self._handlers: dict[str, logging.FileHandler] = {}

    def emit(self, record: logging.LogRecord) -> None:
        subsystem = _subsystem_from_logger(record.name)
        handler = self._handlers.get(subsystem)
        if handler is None:
            handler = logging.FileHandler(
                os.path.join(self._log_dir, f"{subsystem}.log"), encoding="utf-8"
            )
            handler.setLevel(logging.DEBUG)
            handler.setFormatter(self._formatter)
            self._handlers[subsystem] = handler
        handler.emit(record)

    def close(self) -> None:
        for handler in self._handlers.values():
            handler.close()
        self._handlers.clear()
        super().close()


class RelativeFormatter(logging.Formatter):
    def __init__(self, *, start_time: float, base_path: str) -> None:
        super().__init__(
            fmt=(
                "%(asctime_date)s [%(levelname)6s] %(elapsed)s %(asctime_time)s %(name)s  "
                " %(relpath)s:%(lineno)d %(funcName)s: %(message)s"
            )
        )
        self._start_time = start_time
        self._base_path = base_path

    def format(self, record: logging.LogRecord) -> str:
        record.elapsed = f"{time.monotonic() - self._start_time:.3f}"
        created = datetime.fromtimestamp(record.created)
        record.asctime_date = created.strftime("%Y-%m-%d")
        record.asctime_time = created.strftime("%H:%M:%S")
        record.relpath = _safe_relpath(record.pathname, self._base_path)
        return super().format(record)


def setup_logging(
    *,
    subsystems: Iterable[str] | None = None,
    logs_root: str = "logs",
    base_path: str | None = None,
) -> dict[str, logging.Logger]:
    if not logs_root or not str(logs_root).strip():
        raise LoggingSetupError("logs_root must not be empty.")

    if subsystems is not None:
        normalized_subsystems = {_normalize_subsystem(name) for name in subsystems}
    else:
        normalized_subsystems = set()

    start_time = time.monotonic()
    base_path = os.path.abspath(base_path or os.getcwd())
    formatter = RelativeFormatter(start_time=start_time, base_path=base_path)

    timestamp_dir = datetime.now().strftime(LOG_DIR_FORMAT)
    log_dir = os.path.join(logs_root, timestamp_dir)
    os.makedirs(log_dir, exist_ok=True)

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    for handler in list(root_logger.handlers):
        if handler.name and handler.name.startswith(HANDLER_NAME_PREFIX):
            root_logger.removeHandler(handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.INFO)
    stream_handler.setFormatter(formatter)
    stream_handler.name = f"{HANDLER_NAME_PREFIX}_stream"
    root_logger.addHandler(stream_handler)

    debug_handler = logging.FileHandler(
        os.path.join(log_dir, "debug.log"), encoding="utf-8"
    )
    debug_handler.setLevel(logging.DEBUG)
    debug_handler.setFormatter(formatter)
    debug_handler.name = f"{HANDLER_NAME_PREFIX}_debug"
    root_logger.addHandler(debug_handler)

    subsystem_handler = SubsystemFileHandler(log_dir, formatter)
    subsystem_handler.name = f"{HANDLER_NAME_PREFIX}_subsystem"
    root_logger.addHandler(subsystem_handler)

    return {name: logging.getLogger(name) for name in sorted(normalized_subsystems)}
