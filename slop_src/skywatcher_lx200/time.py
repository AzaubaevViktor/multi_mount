from __future__ import annotations

import datetime as dt

from lx200.models import LX200Date, LX200Time, LX200UtcOffset
from lx200.plugins.time import LX200TimeBackend
from lx200.protocol import LX200Constants

from .common import SkyWatcherBackendConstants


class SkyWatcherTimeBackend(LX200TimeBackend):
    def __init__(
        self,
        local_time: LX200Time | None = None,
        date: LX200Date | None = None,
        utc_offset: LX200UtcOffset | None = None,
    ) -> None:
        self._local_time = local_time or SkyWatcherBackendConstants.DEFAULT_LOCAL_TIME
        self._date = date or SkyWatcherBackendConstants.DEFAULT_DATE
        self._utc_offset = utc_offset or SkyWatcherBackendConstants.DEFAULT_UTC_OFFSET

    def initialize(self) -> None:
        now = dt.datetime.now().astimezone()
        offset = now.utcoffset()
        offset_hours = 0.0
        if offset is not None:
            offset_hours = offset.total_seconds() / LX200Constants.SECONDS_PER_HOUR
        self._local_time = LX200Time(hour=now.hour, minute=now.minute, second=now.second)
        self._date = LX200Date(month=now.month, day=now.day, year=now.year)
        self._utc_offset = LX200UtcOffset(hours=offset_hours)

    def set_local_time(self, value: LX200Time) -> bool:
        self._local_time = value
        return True

    def set_date(self, value: LX200Date) -> bool:
        self._date = value
        return True

    def set_utc_offset(self, value: LX200UtcOffset) -> bool:
        self._utc_offset = value
        return True

    def get_local_time(self) -> LX200Time:
        return self._local_time

    def get_date(self) -> LX200Date:
        return self._date

    def get_utc_offset(self) -> LX200UtcOffset:
        return self._utc_offset
