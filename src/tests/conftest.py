import logging
import os
from pathlib import Path
from typing import Any, Callable, Iterator

import pytest

from sim.clock import Clock as VirtualClock
from sky.physics import set_clock

# The original factory travels between the two hooks through the config stash:
# ``pytest.Config`` has no room for ad-hoc attributes, the stash is its typed slot.
_ORIGINAL_RECORD_FACTORY = pytest.StashKey[Callable[..., logging.LogRecord]]()


@pytest.fixture
def virtual_clock() -> Iterator[VirtualClock]:
    """Put ``Second.monotonic()`` -- i.e. Axis and PolarCompensator -- on virtual time.

    The same instance also satisfies the ``clock.Clock`` protocol taken by
    ``SerialLine`` and the motor drivers, so a test can hand it to them and run
    the whole stack on one clock. The previous clock is restored on teardown,
    so the next test sees real time again even if this one failed.
    """
    clock = VirtualClock()
    previous = set_clock(clock)
    try:
        yield clock
    finally:
        set_clock(previous)


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
    config.stash[_ORIGINAL_RECORD_FACTORY] = original_factory


@pytest.hookimpl(trylast=True)
def pytest_unconfigure(config: pytest.Config) -> None:
    original_factory = config.stash.get(_ORIGINAL_RECORD_FACTORY, None)
    if original_factory is not None:
        logging.setLogRecordFactory(original_factory)
