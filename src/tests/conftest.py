import logging
import os
from pathlib import Path
from typing import Any

import pytest


@pytest.hookimpl(tryfirst=True)
def pytest_configure(config: pytest.Config) -> None:
    base_path = str(Path(str(config.rootpath)).resolve())
    original_factory = logging.getLogRecordFactory()

    def create_record(*args: Any, **kwargs: Any) -> logging.LogRecord:
        record = original_factory(*args, **kwargs)
        try:
            record.relpath = os.path.relpath(record.pathname, base_path)
        except ValueError:
            record.relpath = record.pathname
        return record

    logging.setLogRecordFactory(create_record)
    config._multi_mount_original_record_factory = original_factory


@pytest.hookimpl(trylast=True)
def pytest_unconfigure(config: pytest.Config) -> None:
    original_factory = getattr(config, "_multi_mount_original_record_factory", None)
    if original_factory is not None:
        logging.setLogRecordFactory(original_factory)
