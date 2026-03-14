from __future__ import annotations

import logging
import os


class RelativePathFilter(logging.Filter):
    def __init__(self, root: str) -> None:
        super().__init__()
        self.root = root

    def filter(self, record: logging.LogRecord) -> bool:
        try:
            record.relpath = os.path.relpath(record.pathname, self.root)
        except ValueError:  # pragma: no cover - path edge case
            record.relpath = record.pathname
        return True


def configure_logging(level: str = "INFO") -> None:
    root = os.path.dirname(__file__)
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("[%(levelname)6s] %(name)s %(relpath)s:%(lineno)d %(message)s"))
    handler.addFilter(RelativePathFilter(root))
    logger = logging.getLogger()
    logger.handlers.clear()
    logger.addHandler(handler)
    logger.setLevel(getattr(logging, level.upper(), logging.INFO))
