import dataclasses
from enum import IntEnum, StrEnum
import logging
import time
from serial_wrapper.wrapper import SerialLine


class SkyWatcherWrongResponce(Exception):
    pass


class SkyWatcherCommandError(Exception):
    pass


class SkyWatcherRevu24Error(Exception):
    pass


class SkyWatcherCommand(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    SET_STEP_PERIOD = "I"
    SET_GOTO_TARGET = "S"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INSTANT_STOP = "L"
    INITIALIZE = "F"



class SkyWatcherDirection(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class SkyWatcherSlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class SkyWatcherSpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass(frozen=True)
class SkyWatcherStatus:
    raw: int
    running: bool
    initialized: bool
    slew_mode: SkyWatcherSlewMode
    direction: SkyWatcherDirection
    speed_mode: SkyWatcherSpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "SkyWatcherStatus":
        # LOGGER.info("status_data data=%r", data)
        b1 = data[0] if len(data) > 0 else 0
        b2 = data[1] if len(data) > 1 else 0
        b3 = data[2] if len(data) > 2 else 0
        raw = b2 | (b1 << 8) | (b3 << 16)
        running = bool(b2 & 0x01)
        initialized = bool(b3 & 0x01)
        slew_mode = SkyWatcherSlewMode.SLEW if (b1 & 0x01) else SkyWatcherSlewMode.GOTO
        direction = SkyWatcherDirection.BACKWARD if (b1 & 0x02) else SkyWatcherDirection.FORWARD
        speed_mode = SkyWatcherSpeedMode.HIGHSPEED if (b1 & 0x04) else SkyWatcherSpeedMode.LOWSPEED
        return cls(
            raw=raw,
            running=running,
            initialized=initialized,
            slew_mode=slew_mode,
            direction=direction,
            speed_mode=speed_mode,
        )
    

class Axis(StrEnum):
    RA = "1"
    DEC = "2"


class Revu24:
    @staticmethod
    def from_mount(data: str) -> int:
        if len(data) < 6:
            raise SkyWatcherRevu24Error(f"Expected at least 6 hex chars, got {len(data)}")

        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherRevu24Error(f"Invalid hex data: {data!r}") from exc

    @staticmethod
    def from_int(value: int) -> str:
        try:
            if value < 0 or value > 0xFFFFFF:
                raise SkyWatcherRevu24Error(
                    f"Expected value in range 0..{0xFFFFFF}, got {value}"
                )
        except TypeError as exc:
            raise SkyWatcherRevu24Error(f"Invalid value: {value!r}") from exc

        return value.to_bytes(3, "little").hex().upper()


class SkyWatcherMount:
    _LEADING = ":"
    _TRAILING = "\r"
    _COMMAND_ERROR_PREFIX = "!"
    _RESPONCE_PREFIX = "="

    def __init__(self, serial: SerialLine) -> None:
        self.logger = logging.getLogger("skywatcher")
        self._serial = serial

        self.timeout_s = 5
        self.poll_interval_s = .5

    def _get_axis(self):
        return Axis.RA  # RA axis

    def _transact(self, cmd: SkyWatcherCommand, arg: str | None = None) -> str:
        """ All transactions works only with RA """
        self.logger.info("Send %s(%s) ...", cmd, arg if arg is not None else "")

        payload = [
            self._LEADING,
            cmd.value,
            str(self._get_axis())
        ]

        if arg is not None:
            payload.append(arg)
        
        payload.append(self._TRAILING)

        payload_raw = ''.join(payload)

        self.logger.debug("TX %s", payload)
        response = self._serial.query(payload_raw)
        self.logger.debug("RX %s", response)

        # TODO: Create Exceptions below
        if not response.endswith(self._TRAILING):
            raise SkyWatcherWrongResponce(response)
        
        responce_clean = response.removesuffix(self._TRAILING)

        if response[0] == self._COMMAND_ERROR_PREFIX:
            raise SkyWatcherCommandError(response)
        
        if response[0] != self._RESPONCE_PREFIX:
            raise SkyWatcherWrongResponce(response)
        
        responce_clean = responce_clean.removeprefix(self._RESPONCE_PREFIX)
        
        self.logger.info("Receive: %s(%s) -> %s", cmd, arg if arg is not None else "", responce_clean)
        
        return responce_clean
    
    def _do_initialise(self):
        self.logger.info("Wait until mount is initialized...")

        if not self.get_status().initialized:
            self._transact(SkyWatcherCommand.INITIALIZE)
            start = time.monotonic()
            while not self.get_status().initialized:
                if (wait_time := time.monotonic() - start) > self.timeout_s:
                    raise TimeoutError(f"SkyWatcher mount can't initialize for {wait_time:.2f}s")
                time.sleep(self.poll_interval_s)
        else:
            self.logger.info("Mount initialized")

    def get_status(self) -> SkyWatcherStatus:
        status = SkyWatcherStatus.from_bytes(
            self._transact(SkyWatcherCommand.INQUIRE_STATUS).encode('ascii')
        )
        self.logger.debug("Mount status: %s", status)
        return status

    def connect(self):
        self._serial.connect()
        self._do_initialise()

    def get_telescope_ra(self):
        data = self._transact(SkyWatcherCommand.INQUIRE_POSITION)
        return Revu24.from_mount(data)
