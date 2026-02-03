import dataclasses
from enum import IntEnum, StrEnum
import logging
import time
from lx200.protocols import LX200Hours
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
    SET_BREAK_POINT = "M"
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


@dataclasses.dataclass
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


@dataclasses.dataclass
class SkyWatcherAxisState:
    cpr: int


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

    _STELLAR_DAY = 86164.098903691
    _STELLAR_SPEED = 15.041067179
    _SIDEREAL_DAY = 86164.09053083288
    _SIDEREAL_SPEED = 15.04106864

    _LOWSPEED_RATE = 128
    _LOWSPEED_MARGIN = 20000

    _LOWSPEED_PERIOD = 18  # ??
    _HIGH_PERIOD = 10

    _POSITION_OFFSET = 0x800000

    def __init__(self, serial: SerialLine) -> None:
        self.logger = logging.getLogger("skywatcher")
        self._serial = serial

        self.timeout_s = 5
        self.poll_interval_s = .5

        self.ra_steps_360: int
        self.ra_steps_worm: int
        self.ra_highspeed_ratio: int

    def _transact(self, cmd: SkyWatcherCommand, arg: str | None = None, axis: Axis = Axis.RA) -> str:
        """ All transactions works only with RA """
        self.logger.info("Send %s(%s) ...", cmd.name, arg if arg is not None else "")

        payload = [
            self._LEADING,
            cmd.value,
            str(axis)
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
        
        self.logger.info("Receive: %s(%s) -> %s", cmd.name, arg if arg is not None else "", responce_clean)
        
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

        self.logger.info("Get rate values")
        self.ra_steps_360 = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_CPR))
        self.ra_steps_worm = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_TIMER_FREQ))
        # TODO: Make 2-hex-digits revu24
        # self.ra_highspeed_ratio = Revu24.from_mount(self._transact(SkyWatcherCommand.INQUIRE_HIGHSPEED_RATIO))

    def get_status(self) -> SkyWatcherStatus:
        status = SkyWatcherStatus.from_bytes(
            self._transact(SkyWatcherCommand.INQUIRE_STATUS).encode('ascii')
        )
        self.logger.debug("Mount status: %s", status)
        return status

    def connect(self):
        self._serial.connect()
        self._do_initialise()

    def _ticks_to_hours(self, ticks: int) -> float:
        return ticks / self.ra_steps_360 * 24
    
    def _hours_to_ticks(self, hours: float) -> int:
        return int(hours / 24 * self.ra_steps_360)

    def get_telescope_ra(self):
        data = self._transact(SkyWatcherCommand.INQUIRE_POSITION)
        ticks = (Revu24.from_mount(data) - self._POSITION_OFFSET) % self.ra_steps_360
        # Ticks / Full circle / (24h) -> hours
        hours = self._ticks_to_hours(ticks)
        total_seconds = int(round(hours * 3600)) % (24 * 3600)
        return LX200Hours.from_seconds(total_seconds)
    
    def set_telescope_ra(self, position: LX200Hours) -> bool:
        hours = position.to_hours()
        ticks = (self._hours_to_ticks(hours) + self._POSITION_OFFSET) % self.ra_steps_360

        self._transact(SkyWatcherCommand.SET_AXIS_POSITION, Revu24.from_int(int(ticks)))

        return True
    
    def _wait_motor_stop(self):
        # self._transact(SkyWatcherCommand.STOP_MOTION)
        self.logger.debug("Wait while motor stops...")
        while True:
            status = self.get_status()
            if not status.running:
                break

            time.sleep(.5)
            
    
    def _set_motion(self, new_status: SkyWatcherStatus):
        self._wait_motor_stop()
        if new_status.slew_mode == SkyWatcherSlewMode.SLEW:
            motion_mode = "1" if new_status.speed_mode == SkyWatcherSpeedMode.LOWSPEED else "3"
        elif new_status.slew_mode == SkyWatcherSlewMode.GOTO:
            motion_mode = "2" if new_status.speed_mode == SkyWatcherSpeedMode.LOWSPEED else "0"
        else:
            motion_mode = "1"

        direction_mode = "0" if new_status.direction == SkyWatcherDirection.FORWARD else "1"

        self._transact(SkyWatcherCommand.SET_MOTION_MODE, f"{motion_mode}{direction_mode}")

    def _set_speed(self, period: int):
        self._transact(SkyWatcherCommand.SET_STEP_PERIOD, Revu24.from_int(period))
    
    def _set_target(self, ticks: int):
        self._transact(SkyWatcherCommand.SET_GOTO_TARGET, Revu24.from_int(ticks))

    def _set_target_breaks(self, ticks: int):
        self._transact(SkyWatcherCommand.SET_BREAK_POINT_INCREMENT, Revu24.from_int(ticks))

    def _start_motor(self):
        self._transact(SkyWatcherCommand.START_MOTION)
    
    def slew_to_ra(self, position: LX200Hours) -> bool:
        new_status = self.get_status()
        new_status.slew_mode = SkyWatcherSlewMode.GOTO

        current_position = self.get_telescope_ra()

        delta = position - current_position
        delta_hours = delta.to_hours()
        
        if delta_hours > 12:
            new_status.direction = SkyWatcherDirection.BACKWARD
            distance = -delta
        else:
            new_status.direction = SkyWatcherDirection.FORWARD
            distance = delta

        delta_ticks = self._hours_to_ticks(distance.to_hours())

        if delta_ticks > self._LOWSPEED_MARGIN:
            new_status.speed_mode = SkyWatcherSpeedMode.HIGHSPEED
            is_highspeed = True
        else:
            new_status.speed_mode = SkyWatcherSpeedMode.LOWSPEED
            is_highspeed = False
        
        new_status.speed_mode = SkyWatcherSpeedMode.LOWSPEED


        self._set_motion(new_status)
        self._set_speed(self._HIGH_PERIOD if is_highspeed else self._LOWSPEED_PERIOD)
        self._set_target(self._hours_to_ticks(position.to_hours()) + self._POSITION_OFFSET)
        self._set_target_breaks(min(200, delta_ticks))  # TODO: Check highspeed
        self._start_motor()
        return True

    def move_ra(self, rate: float) -> bool:
        status = self.get_status()
        if status.running and status.slew_mode == SkyWatcherSlewMode.GOTO:
            self.logger.warning("Can not slew while GOTO in progress")
            return False
        
