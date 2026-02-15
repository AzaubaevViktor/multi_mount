import hashlib
import logging
import os
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import pytest

from logging_setup import LOG_DIR_FORMAT


SESSION_LOG_FILENAME = "pytest.log"
TEST_LOG_DIRNAME = "tests"
NODEID_HASH_LENGTH = 8
DEFAULT_LOG_LEVEL = logging.INFO
PLUGIN_NAME = "multi_mount_pytest_session_logging"
HANDLER_NAME_PREFIX = "multi_mount_pytest_logging"
SAFE_NODEID_CHARS = frozenset("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-")


class PytestSessionLoggingError(Exception):
    pass


def _resolve_log_level(log_level: str) -> int:
    normalized_log_level = log_level.strip()
    if not normalized_log_level:
        return DEFAULT_LOG_LEVEL

    mapped_log_level = logging.getLevelName(normalized_log_level.upper())
    if isinstance(mapped_log_level, int):
        return mapped_log_level

    try:
        return int(normalized_log_level)
    except ValueError as error:
        raise PytestSessionLoggingError(
            f"Unsupported pytest log level '{log_level}'."
        ) from error


def _build_unique_session_dir(logs_root: Path) -> Path:
    timestamp = datetime.now().strftime(LOG_DIR_FORMAT)
    session_dir = logs_root / timestamp
    if not session_dir.exists():
        return session_dir

    suffix_index = 1
    while True:
        session_dir = logs_root / f"{timestamp}_{suffix_index:02d}"
        if not session_dir.exists():
            return session_dir
        suffix_index += 1


def _sanitize_nodeid(nodeid: str) -> str:
    normalized_chars = [
        symbol if symbol in SAFE_NODEID_CHARS else "_"
        for symbol in nodeid
    ]
    normalized_nodeid = "".join(normalized_chars).strip("._")
    nodeid_hash = hashlib.sha1(nodeid.encode("utf-8")).hexdigest()[:NODEID_HASH_LENGTH]
    if not normalized_nodeid:
        return f"test_{nodeid_hash}"
    return f"{normalized_nodeid}_{nodeid_hash}"


class SessionLogsManager:
    def __init__(self, config: pytest.Config) -> None:
        self._config = config
        self._base_path = Path(str(config.rootpath)).resolve()
        self._logs_root = self._base_path / "logs"
        self._session_dir = _build_unique_session_dir(self._logs_root)
        self._test_logs_dir = self._session_dir / TEST_LOG_DIRNAME
        self._session_handler: logging.FileHandler | None = None
        self._test_handler: logging.FileHandler | None = None
        self._original_record_factory: Callable[..., logging.LogRecord] = logging.getLogRecordFactory()

        log_format = self._config.getini("log_format")
        if not log_format or not str(log_format).strip():
            raise PytestSessionLoggingError("pytest log_format must not be empty.")

        self._formatter = logging.Formatter(
            fmt=log_format,
            datefmt=self._config.getini("log_date_format"),
        )
        self._log_level = _resolve_log_level(self._config.getini("log_level"))

    @property
    def session_dir(self) -> Path:
        return self._session_dir

    def start(self) -> None:
        self._logs_root.mkdir(parents=True, exist_ok=True)
        self._session_dir.mkdir(parents=False, exist_ok=False)
        self._test_logs_dir.mkdir(parents=False, exist_ok=False)

        logging.setLogRecordFactory(self._record_factory_with_relpath())

        self._session_handler = logging.FileHandler(
            self._session_dir / SESSION_LOG_FILENAME,
            encoding="utf-8",
        )
        self._session_handler.name = f"{HANDLER_NAME_PREFIX}_session"
        self._session_handler.setLevel(self._log_level)
        self._session_handler.setFormatter(self._formatter)
        logging.getLogger().addHandler(self._session_handler)

    def finish(self) -> None:
        self.finish_test()

        if self._session_handler is not None:
            root_logger = logging.getLogger()
            root_logger.removeHandler(self._session_handler)
            self._session_handler.close()
            self._session_handler = None

        logging.setLogRecordFactory(self._original_record_factory)

    def start_test(self, nodeid: str) -> None:
        self.finish_test()

        test_log_file = self._test_logs_dir / f"{_sanitize_nodeid(nodeid)}.log"
        self._test_handler = logging.FileHandler(test_log_file, encoding="utf-8")
        self._test_handler.name = f"{HANDLER_NAME_PREFIX}_test"
        self._test_handler.setLevel(self._log_level)
        self._test_handler.setFormatter(self._formatter)
        logging.getLogger().addHandler(self._test_handler)

    def finish_test(self) -> None:
        if self._test_handler is None:
            return

        root_logger = logging.getLogger()
        root_logger.removeHandler(self._test_handler)
        self._test_handler.close()
        self._test_handler = None

    def _record_factory_with_relpath(self) -> Callable[..., logging.LogRecord]:
        base_path = str(self._base_path)
        original_factory = self._original_record_factory

        def create_record(*args: Any, **kwargs: Any) -> logging.LogRecord:
            record = original_factory(*args, **kwargs)
            try:
                record.relpath = os.path.relpath(record.pathname, base_path)
            except ValueError:
                record.relpath = record.pathname
            return record

        return create_record


class SessionLogsPlugin:
    def __init__(self, config: pytest.Config) -> None:
        self._logs_manager = SessionLogsManager(config)
        self._logs_manager.start()

    @pytest.hookimpl(hookwrapper=True)
    def pytest_runtest_protocol(
        self,
        item: pytest.Item,
        nextitem: pytest.Item | None,
    ):
        del nextitem
        self._logs_manager.start_test(item.nodeid)
        try:
            yield
        finally:
            self._logs_manager.finish_test()

    def pytest_report_header(self) -> str:
        return f"pytest logs dir: {self._logs_manager.session_dir}"

    def pytest_sessionfinish(
        self,
        session: pytest.Session,
        exitstatus: int,
    ) -> None:
        del session, exitstatus
        self._logs_manager.finish()


def pytest_configure(config: pytest.Config) -> None:
    if config.pluginmanager.has_plugin(PLUGIN_NAME):
        return

    config.pluginmanager.register(SessionLogsPlugin(config), PLUGIN_NAME)
