"""Red test for П7 (routine polling floods INFO), PLAN.md §1 П7.

Clients poll ``:GR#`` / ``:GD#`` (and ``:D#``) ~1 Hz, and each is logged twice
(Get + Answer) at INFO. Over a session that is thousands of INFO lines that bury
the meaningful events.

Desired contract: routine polling commands (GR, GD, D) are logged at DEBUG, not
INFO. A caplog capturing at INFO level must not catch any records mentioning
GR/GD handling. Today ``handle()`` logs them via ``_logger.info(...)`` — red.
"""

import logging

import pytest

from lx200.base import LX200Handler
from sky.physics import Dec, Ha


class _PollingHandler(LX200Handler):
    def get_telescope_ra(self) -> Ha:
        return Ha(3600)

    def get_telescope_dec(self) -> Dec:
        return Dec(0)

    def get_distance(self) -> str:
        return "#"


def test_routine_polling_commands_are_not_logged_at_info(caplog: pytest.LogCaptureFixture) -> None:
    """П7: GR/GD/D polling must be DEBUG, invisible to an INFO-level capture.

    See PLAN.md §1 П7 / §3 item 11.
    """
    handler = _PollingHandler()
    handler.connect()

    with caplog.at_level(logging.INFO, logger="lx200"):
        handler.handle("GR")
        handler.handle("GD")
        handler.handle("D")

    polling_info_records = [
        record
        for record in caplog.records
        if record.levelno >= logging.INFO
        and any(token in record.getMessage() for token in ("GR", "GD", "GET_TELECOPE_RA", "GET_TELESCOPE_DEC", "GET_DISTANCE"))
    ]

    assert not polling_info_records, (
        f"routine polling produced {len(polling_info_records)} INFO+ records "
        f"(expected DEBUG only): {[r.getMessage() for r in polling_info_records]}"
    )
