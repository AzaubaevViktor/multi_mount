from __future__ import annotations

import dataclasses
import datetime as dt
import logging
import socket
import threading
import time
from enum import StrEnum
from typing import Optional

from coords import clamp, wrap_hours
from lx200_prims import (
    LX200Constants,
    LX200Dec,
    LX200Date,
    LX200GotoResult,
    LX200MoveDirection,
    LX200ParseError,
    LX200Ra,
    LX200ServerBase,
    LX200SlewRate,
    LX200Time,
    LX200UtcOffset,
    LX200ValueError,
)

LOGGER = logging.getLogger("lx200.dummy")


class LX200DummyConstants:
    HOST = "127.0.0.1"
    PORT = 7624
    BACKLOG = 1
    BUFFER_SIZE = 1024
    ENCODING = "ascii"
    DECODE_ERRORS = "ignore"
    TERMINATOR_BYTE = LX200Constants.TERMINATOR.encode(ENCODING)
    ALIGNMENT_QUERY_BYTE = b"\x06"
    EMPTY_RESPONSE = ""
    RESPONSE_ERROR = LX200Constants.RESPONSE_ERR
    MIN_RA = LX200Constants.MIN_HOUR
    MAX_RA = LX200Constants.HOURS_PER_DAY
    MIN_DEC = LX200Constants.MIN_LAT_DEG
    MAX_DEC = LX200Constants.MAX_LAT_DEG
    MIN_LON = LX200Constants.MIN_LON_DEG
    MAX_LON = LX200Constants.MAX_LON_DEG
    DEFAULT_RA = 0.0
    DEFAULT_DEC = 0.0
    DEFAULT_LAT = 0.0
    DEFAULT_LON = 0.0
    DEFAULT_UTC_OFFSET = 0.0
    DEFAULT_LOCAL_TIME = dt.time(0, 0, 0)
    DEFAULT_LOCAL_DATE = dt.date(2020, 1, 1)
    UPDATE_INTERVAL_S = 1.0
    SECONDS_PER_MINUTE = 60
    MINUTES_PER_HOUR = 60
    HOURS_PER_DAY = 24
    SECONDS_PER_HOUR = SECONDS_PER_MINUTE * MINUTES_PER_HOUR
    SECONDS_PER_DAY = SECONDS_PER_HOUR * HOURS_PER_DAY
    SOCKET_TRUE = 1


class LX200DummyServerError(Exception):
    pass


class LX200AlignmentMode(StrEnum):
    ALT_AZ = "A"
    LAND = "L"
    POLAR = "P"


@dataclasses.dataclass
class LX200DummyState:
    current_ra: float = LX200DummyConstants.DEFAULT_RA
    current_dec: float = LX200DummyConstants.DEFAULT_DEC
    target_ra: float = LX200DummyConstants.DEFAULT_RA
    target_dec: float = LX200DummyConstants.DEFAULT_DEC
    slew_rate: LX200SlewRate = LX200SlewRate.SLEW
    latitude_deg: float = LX200DummyConstants.DEFAULT_LAT
    longitude_west_deg: float = LX200DummyConstants.DEFAULT_LON
    utc_offset: float = LX200DummyConstants.DEFAULT_UTC_OFFSET
    local_time: dt.time = LX200DummyConstants.DEFAULT_LOCAL_TIME
    local_date: dt.date = LX200DummyConstants.DEFAULT_LOCAL_DATE
    alignment_mode: LX200AlignmentMode = LX200AlignmentMode.POLAR
    moving: dict[LX200MoveDirection, bool] = dataclasses.field(default_factory=dict)
    last_update_monotonic: float = dataclasses.field(default_factory=time.monotonic)

    def __post_init__(self) -> None:
        self.current_ra = wrap_hours(self.current_ra)
        self.target_ra = wrap_hours(self.target_ra)
        self.current_dec = clamp(self.current_dec, LX200DummyConstants.MIN_DEC, LX200DummyConstants.MAX_DEC)
        self.target_dec = clamp(self.target_dec, LX200DummyConstants.MIN_DEC, LX200DummyConstants.MAX_DEC)
        self.latitude_deg = clamp(self.latitude_deg, LX200DummyConstants.MIN_DEC, LX200DummyConstants.MAX_DEC)
        self.longitude_west_deg = clamp(self.longitude_west_deg, LX200DummyConstants.MIN_LON, LX200DummyConstants.MAX_LON)
        for direction in LX200MoveDirection:
            self.moving.setdefault(direction, False)

    def update_time(self) -> None:
        now = time.monotonic()
        elapsed = now - self.last_update_monotonic
        if elapsed < LX200DummyConstants.UPDATE_INTERVAL_S:
            return
        self.last_update_monotonic = now
        total_seconds = (
            self.local_time.hour * LX200DummyConstants.SECONDS_PER_HOUR
            + self.local_time.minute * LX200DummyConstants.SECONDS_PER_MINUTE
            + self.local_time.second
        )
        total_seconds += int(elapsed)
        days, seconds = divmod(total_seconds, LX200DummyConstants.SECONDS_PER_DAY)
        hour, rem = divmod(seconds, LX200DummyConstants.SECONDS_PER_HOUR)
        minute, second = divmod(rem, LX200DummyConstants.SECONDS_PER_MINUTE)
        self.local_time = dt.time(hour, minute, second)
        if days:
            self.local_date = self.local_date + dt.timedelta(days=days)


class LX200DummyServer(LX200ServerBase):
    def __init__(self, state: Optional[LX200DummyState] = None, logger: Optional[logging.Logger] = None) -> None:
        super().__init__(logger=logger)
        self.state = state or LX200DummyState()
        self.lock = threading.Lock()

    def _update_time(self) -> None:
        with self.lock:
            self.state.update_time()

    def get_current_ra(self) -> LX200Ra:
        self._update_time()
        with self.lock:
            return LX200Ra(self.state.current_ra)

    def get_current_dec(self) -> LX200Dec:
        self._update_time()
        with self.lock:
            return LX200Dec(self.state.current_dec)

    def set_target_ra(self, ra: LX200Ra) -> bool:
        with self.lock:
            self.state.target_ra = wrap_hours(ra.hours)
        return True

    def set_target_dec(self, dec: LX200Dec) -> bool:
        with self.lock:
            self.state.target_dec = clamp(dec.degrees, LX200DummyConstants.MIN_DEC, LX200DummyConstants.MAX_DEC)
        return True

    def slew_to_target(self) -> LX200GotoResult:
        with self.lock:
            self.state.current_ra = wrap_hours(self.state.target_ra)
            self.state.current_dec = clamp(
                self.state.target_dec,
                LX200DummyConstants.MIN_DEC,
                LX200DummyConstants.MAX_DEC,
            )
        return LX200GotoResult.OK

    def sync_to_target(self) -> str:
        with self.lock:
            self.state.current_ra = wrap_hours(self.state.target_ra)
            self.state.current_dec = clamp(
                self.state.target_dec,
                LX200DummyConstants.MIN_DEC,
                LX200DummyConstants.MAX_DEC,
            )
        return f"{LX200Constants.SYNC_OK}{LX200Constants.TERMINATOR}"

    def stop_all(self) -> None:
        with self.lock:
            for direction in self.state.moving:
                self.state.moving[direction] = False

    def start_move(self, direction: LX200MoveDirection) -> None:
        with self.lock:
            self.state.moving[direction] = True

    def stop_move(self, direction: LX200MoveDirection) -> None:
        with self.lock:
            self.state.moving[direction] = False

    def set_slew_rate(self, rate: LX200SlewRate) -> None:
        with self.lock:
            self.state.slew_rate = rate

    def set_local_time(self, value) -> bool:
        with self.lock:
            self.state.local_time = dt.time(value.hour, value.minute, value.second)
        return True

    def set_date(self, value) -> bool:
        with self.lock:
            self.state.local_date = dt.date(value.year, value.month, value.day)
        return True

    def set_utc_offset(self, value: LX200UtcOffset) -> bool:
        with self.lock:
            self.state.utc_offset = value.hours
        return True

    def set_latitude(self, latitude_deg: float) -> bool:
        with self.lock:
            self.state.latitude_deg = clamp(latitude_deg, LX200DummyConstants.MIN_DEC, LX200DummyConstants.MAX_DEC)
        return True

    def set_longitude(self, longitude_west_deg: float) -> bool:
        with self.lock:
            self.state.longitude_west_deg = clamp(
                longitude_west_deg,
                LX200DummyConstants.MIN_LON,
                LX200DummyConstants.MAX_LON,
            )
        return True

    def get_local_time(self):
        self._update_time()
        with self.lock:
            return LX200Time(hour=self.state.local_time.hour, minute=self.state.local_time.minute, second=self.state.local_time.second)

    def get_date(self):
        self._update_time()
        with self.lock:
            return LX200Date(month=self.state.local_date.month, day=self.state.local_date.day, year=self.state.local_date.year)

    def get_utc_offset(self):
        with self.lock:
            return LX200UtcOffset(self.state.utc_offset)

    def get_latitude(self) -> float:
        with self.lock:
            return self.state.latitude_deg

    def get_longitude(self) -> float:
        with self.lock:
            return self.state.longitude_west_deg


class LX200DummyTcpServer:
    def __init__(
        self,
        handler: LX200ServerBase,
        *,
        host: str = LX200DummyConstants.HOST,
        port: int = LX200DummyConstants.PORT,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self.handler = handler
        self.host = host
        self.port = port
        self.log = logger or logging.getLogger("lx200.tcp")
        self._socket: Optional[socket.socket] = None

    def serve_forever(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, LX200DummyConstants.SOCKET_TRUE)
            srv.bind((self.host, self.port))
            srv.listen(LX200DummyConstants.BACKLOG)
            self._socket = srv
            self.log.info("LX200 dummy server listening on %s:%s", self.host, self.port)
            while True:
                conn, addr = srv.accept()
                self.log.info("LX200 client connected: %s", addr)
                thread = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                thread.start()

    def _handle_client(self, conn: socket.socket) -> None:
        with conn:
            buf = bytearray()
            while True:
                data = conn.recv(LX200DummyConstants.BUFFER_SIZE)
                if not data:
                    return
                idx = data.find(LX200DummyConstants.ALIGNMENT_QUERY_BYTE)
                if idx >= 0:
                    while idx >= 0:
                        if idx:
                            buf.extend(data[:idx])
                        response = self.handler_alignment_query()
                        self.log.debug("lx200 rx raw=%r cmd=<ACK>", LX200DummyConstants.ALIGNMENT_QUERY_BYTE)
                        self.log.debug("lx200 tx response=%r", response)
                        conn.sendall(response.encode(LX200DummyConstants.ENCODING))
                        data = data[idx + 1 :]
                        idx = data.find(LX200DummyConstants.ALIGNMENT_QUERY_BYTE)
                    if data:
                        buf.extend(data)
                else:
                    buf.extend(data)
                while True:
                    idx = buf.find(LX200DummyConstants.TERMINATOR_BYTE)
                    if idx < 0:
                        break
                    raw = bytes(buf[: idx + 1])
                    del buf[: idx + 1]
                    self._handle_raw(conn, raw)

    def _handle_raw(self, conn: socket.socket, raw: bytes) -> None:
        try:
            if LX200DummyConstants.ALIGNMENT_QUERY_BYTE in raw:
                response = self.handler_alignment_query()
                self.log.debug("lx200 rx raw=%r cmd=<ACK>", raw)
                self.log.debug("lx200 tx response=%r", response)
                conn.sendall(response.encode(LX200DummyConstants.ENCODING))
                return
            text = raw.decode(LX200DummyConstants.ENCODING, errors=LX200DummyConstants.DECODE_ERRORS)
            if LX200Constants.PREFIX not in text:
                return
            start = text.index(LX200Constants.PREFIX)
            command = text[start:]
            self.log.debug("lx200 rx raw=%r cmd=%r", raw, command)
            response = self.handler.handle_command(command)
        except (LX200ParseError, LX200ValueError) as exc:
            self.log.debug("LX200 parse error: %s", exc)
            response = LX200DummyConstants.RESPONSE_ERROR
        except Exception:
            self.log.exception("LX200 handler error")
            response = LX200DummyConstants.RESPONSE_ERROR
        if response == LX200DummyConstants.EMPTY_RESPONSE:
            self.log.debug("lx200 tx empty")
            return
        self.log.debug("lx200 tx response=%r", response)
        conn.sendall(response.encode(LX200DummyConstants.ENCODING))

    def handler_alignment_query(self) -> str:
        if isinstance(self.handler, LX200DummyServer):
            with self.handler.lock:
                return self.handler.state.alignment_mode.value
        return LX200AlignmentMode.POLAR.value


def run_dummy_server(
    host: str = LX200DummyConstants.HOST,
    port: int = LX200DummyConstants.PORT,
) -> None:
    server = LX200DummyTcpServer(LX200DummyServer(), host=host, port=port)
    server.serve_forever()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    run_dummy_server()
