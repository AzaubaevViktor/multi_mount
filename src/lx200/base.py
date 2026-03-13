from dataclasses import dataclass
from enum import StrEnum
import logging
import queue
import re
import threading
import time
from typing import Any, Sequence

from sky.constants import STELLAR_SPEED
from sky.physics import AxisPos, Dec, DecPerSecond, DecPerSecond, HaPerSecond, Ha, AxisSpeed, Second
from sky.physics import SkyDirection

from .protocol import AlignmentMode


class LX200Commands(StrEnum):
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"

    GET_TELESCOPE_DEC = "GD"
    SET_TELESCOPE_DEC = "Sd"

    SYNC = "CM"
    SLEW = "MS"
    GET_DISTANCE = "D"

    MOVE_EAST = "Me"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    MOVE_WEST = "Mw"

    HALT_ALL = "Q"

    HALT_EAST = "Qe"
    HALT_NORTH = "Qn"
    HALT_SOUTH = "Qs"
    HALT_WEST = "Qw"

    SET_TARGET_DEC = "Sd"
    SET_TARGET_RA = "Sr"
    
    GET_CALENDAR_FORMAT = "Gc"
    GET_SITE1_NAME = "GM"

    GET_TRACKING_RATE = "GT"
    GET_CURRENT_SITE_LATITUDE= "Gt"
    GET_CURRENT_SITE_LONGITUDE = "Gg"
    GET_UTC_OFFSET_TIME = "GG"
    GET_LOCAL_TIME = "GL"
    GET_CURRENT_DATE = "GC"

    SET_CURRENT_SITE_LONGTITUDE = "Sg"
    SET_CURRENT_SITE_LATITUDE = "St"
    SET_UTC = "SG"

    SET_LOCAL_TIME = "SL"
    SET_LOCAL_DATE = "SC"

    SET_MINIMUM_ELEVATION = "Sh"
    SET_HIGHEST_ELEVATION = "So"

    SET_SLEW_TO_GUIDE = "RG"
    SET_SLEW_TO_CENTER = "RC"
    SET_SLEW_TO_FIND = "RM"
    SET_SLEW_TO_MAX = "RS"

    GUIDE = "Mg"


class _LX200NotImplementedCommand:
    def __init__(self, cmd: str) -> None:
        self.cmd = cmd


_logger = logging.getLogger("lx200")


@dataclass
class GuideTask:
    direction: SkyDirection
    ms: int


class AxisCommandType(StrEnum):
    SET_TRACKING_SPEED = "set_tracking_speed"
    HALT_MOTION = "halt_motion"
    HALT_DIRECTION = "halt_direction"


@dataclass
class AxisCommand:
    type: AxisCommandType
    speed: AxisSpeed | None = None
    update_sky_speed: bool = False
    direction: SkyDirection | None = None


class LX200Base:
    def connect(self) -> None:
        raise NotImplementedError()
    
    def stop(self) -> None:
        raise NotImplementedError()
    
    def __del__(self):
        self.stop()
    
    # Auxilary
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        raise NotImplementedError()
    
    def get_calendar_format(self) -> str:
        return "24"
    
    def get_site1_name(self) -> str:
        return "base_lx200"
    
    def get_tracking_rate(self) -> str:
        return "60.0"

    def set_minimum_elevation(self, position: Dec) -> bool:
        return True

    def set_highest_elevation(self, position: Dec) -> bool:
        return True
    
    # Telecope control
    def get_telescope_ra(self) -> Ha:
        raise NotImplementedError()
    
    def motor_position(self) -> tuple[Ha, Dec]:
        raise NotImplementedError()
    
    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        result = self.sync_telescope_ra(ra)
        result &= self.sync_telescope_dec(dec)
        return result

    def sync_telescope_ra(self, position: Ha) -> bool:
        raise NotImplementedError()
    
    def get_telescope_dec(self) -> Dec:
        raise NotImplementedError()
    
    def sync_telescope_dec(self, position: Dec) -> bool:
        raise NotImplementedError()
    
    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        result = self.slew_to_ra(ra)
        result &= self.slew_to_dec(dec)
        return result

    def slew_to_ra(self, position: Ha) -> bool:
        raise NotImplementedError()
    
    def slew_to_dec(self, position: Dec) -> bool:
        raise NotImplementedError()

    def set_slew_to_find(self) -> bool:
        raise NotImplementedError()

    def set_slew_to_guide(self) -> bool:
        return self.set_slew_to_find()

    def set_slew_to_center(self) -> bool:
        return self.set_slew_to_find()

    def set_slew_to_max(self) -> bool:
        return self.set_slew_to_find()

    def get_distance(self) -> str:
        raise NotImplementedError()
    
    # Manual moving
    def move_east(self) -> bool:
        raise NotImplementedError()
    
    def move_north(self) -> bool:
        raise NotImplementedError()
    
    def move_south(self) -> bool:
        raise NotImplementedError()
    
    def move_west(self) -> bool:
        raise NotImplementedError()

    # Halt movements
    def halt_all(self) -> bool:
        raise NotImplementedError()
    
    def halt_east(self) -> bool:
        raise NotImplementedError()
    
    def halt_north(self) -> bool:
        raise NotImplementedError()
    
    def halt_south(self) -> bool:
        raise NotImplementedError()
    
    def halt_west(self) -> bool:
        raise NotImplementedError()

    def guide_east(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_north(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_south(self, ms: int) -> None:
        raise NotImplementedError()
    
    def guide_west(self, ms: int) -> None:
        raise NotImplementedError()


class LX200AxisHandler[_POS_CLS: AxisPos, _SPEED_CLS: AxisSpeed](LX200Base):
    AXIS_NAME: str
    DIRECTIONS: Sequence[SkyDirection]
    POS_CLS: type[_POS_CLS]
    SPEED_CLS: type[_SPEED_CLS]

    _TELEMETRY_INTERVAL_S = 1.0
    _RATE_COMPENSATE_INTERVAL_S = Second(.5)
    COMPENSATE_MOTOR_SIGN: int  # Is this really the best way for RA/DEC?

    BACKWARD_TRACKING_SPEED: _SPEED_CLS
    DEFAULT_TRACKING_SPEED: _SPEED_CLS
    FORWARD_TRACKING_SPEED: _SPEED_CLS
    DEFAULT_GUIDE_INTERVAL_MS: Second = Second(4)

    def __init__(self) -> None:
        if not issubclass(self.POS_CLS, AxisPos):
            raise TypeError(f"POS_CLS should be subclass of AxisPos, got {self.POS_CLS}")
        if not issubclass(self.SPEED_CLS, AxisSpeed):
            raise TypeError(f"SPEED_CLS should be subclass of AxisSpeed, got {self.SPEED_CLS}")

        self._POSITION_DELTA_ACCEPTED_RATE_S: _SPEED_CLS = self.SPEED_CLS(.1)

        self.logger = logging.getLogger(type(self).__name__)
        self._working = True

        self._position_update_lock = threading.RLock()
        self._mount_position_raw: _POS_CLS = self.POS_CLS(0)
        self._motor_position_raw: _POS_CLS = self.POS_CLS(0)
        self._last_update_s: Second = Second.monotonic()

        self._sky_track_speed: _SPEED_CLS = self.DEFAULT_TRACKING_SPEED

        self._guide_interval: Second = self.DEFAULT_GUIDE_INTERVAL_MS
        self._axis_command_queue: queue.Queue[AxisCommand] = queue.Queue()

        self._telemetry_thread = threading.Thread(target=self._do_log_telemetry, name=f"{type(self).__name__}_telemetry")
        self._telemetry_thread.start()

        self._compensate_thread = threading.Thread(target=self._compensate_tracking_speed, name=f"{type(self).__name__}_compensate")
        self._compensate_thread.start()

        self._goto_to: Any

    # TODO: extract NotImplemented _X methods to Motor interface (in different file), unite statuses, motion type and other stuff to base classes, use it for skywatcher and tmc2209adapter

    def _is_motor_connected(self) -> bool:
        raise NotImplementedError()

    def _get_motor_status(self) -> Any:
        raise NotImplementedError()
    
    def _get_motor_raw_position(self) -> _POS_CLS:
        raise NotImplementedError()
    
    def _set_tracking_speed(self, speed: _SPEED_CLS) -> _SPEED_CLS | None:
        raise NotImplementedError()

    def set_tracking_speed(self, speed: _SPEED_CLS, update_sky_speed: bool = False) -> None:
        self._axis_command_queue.put(
            AxisCommand(
                type=AxisCommandType.SET_TRACKING_SPEED,
                speed=speed,
                update_sky_speed=update_sky_speed,
            )
        )
    
    def resume_tracking(self) -> None:
        self.set_tracking_speed(self._sky_track_speed)

    def _halt_motion(self) -> None:
        raise NotImplementedError()

    def calculate_guide_speed(self, direction: SkyDirection, ms: int) -> _SPEED_CLS | None:
        if direction not in self.DIRECTIONS:
            return None

        guide_fration = Second.from_milliseconds(ms) / self._guide_interval

        match direction:
            case SkyDirection.EAST | SkyDirection.SOUTH:
                guide_speed = (self.FORWARD_TRACKING_SPEED - self.DEFAULT_TRACKING_SPEED) * guide_fration + self.DEFAULT_TRACKING_SPEED
            case SkyDirection.WEST | SkyDirection.NORTH:
                guide_speed = self.DEFAULT_TRACKING_SPEED - (
                    (self.DEFAULT_TRACKING_SPEED - self.BACKWARD_TRACKING_SPEED) * guide_fration
                )
            case _:
                self.logger.warning("Wrong direction: %s", direction)
                return None
        
        return guide_speed
    
    def _apply_guide_speed(self, direction: SkyDirection, ms: int) -> None:
        guide_speed = self.calculate_guide_speed(direction, ms)
        if guide_speed is None:
            return

        self.logger.debug(
            "Change guide speed: %s(%d/%d) -> speed: %.3f .. [%.3f] .. %.3f", 
            direction, ms, self._guide_interval.to_milliseconds(), 
            self.BACKWARD_TRACKING_SPEED, guide_speed, self.FORWARD_TRACKING_SPEED,
        )
        
        self.set_tracking_speed(guide_speed, update_sky_speed=True)

    def guide_east(self, ms: int) -> None:
        self._apply_guide_speed(SkyDirection.EAST, ms)

    def guide_north(self, ms: int) -> None:
        self._apply_guide_speed(SkyDirection.NORTH, ms)

    def guide_south(self, ms: int) -> None:
        self._apply_guide_speed(SkyDirection.SOUTH, ms)

    def guide_west(self, ms: int) -> None:
        self._apply_guide_speed(SkyDirection.WEST, ms)

    def halt_all(self):
        self.halt_motion()
        return True

    def halt_motion(self) -> None:
        self._axis_command_queue.put(
            AxisCommand(type=AxisCommandType.HALT_MOTION)
        )

    def _halt_direction(self, direction: SkyDirection) -> bool:
        if direction not in self.DIRECTIONS:
            return False

        self._axis_command_queue.put(
            AxisCommand(type=AxisCommandType.HALT_DIRECTION, direction=direction)
        )
        return True

    def halt_east(self) -> bool:
        return self._halt_direction(SkyDirection.EAST)

    def halt_north(self) -> bool:
        return self._halt_direction(SkyDirection.NORTH)

    def halt_south(self) -> bool:
        return self._halt_direction(SkyDirection.SOUTH)

    def halt_west(self) -> bool:
        return self._halt_direction(SkyDirection.WEST)

    def _apply_axis_command(self, cmd: AxisCommand) -> None:
        if cmd.type == AxisCommandType.HALT_DIRECTION and cmd.direction not in self.DIRECTIONS:
            self.logger.warning("Ignore wrong halt direction for %s: %s", self.AXIS_NAME, cmd.direction)
            return

        self.logger.debug("Apply axis command: %s", cmd)

        match cmd.type:
            case AxisCommandType.SET_TRACKING_SPEED:
                halt_motion = False
                speed = cmd.speed
                update_sky_speed = cmd.update_sky_speed
            case AxisCommandType.HALT_MOTION:
                halt_motion = True
                speed = self._sky_track_speed
                update_sky_speed = True
            case AxisCommandType.HALT_DIRECTION:
                halt_motion = True
                speed = self._sky_track_speed
                update_sky_speed = False
            case _:
                self.logger.warning("Unknown axis command: %s", cmd)
                return 
        
        if speed is None:
            raise ValueError("Rate should be set for command type %s", cmd.type)

        if not isinstance(speed, self.SPEED_CLS):
            raise ValueError("Rate should be of type %s for %s axis, got %s", self.SPEED_CLS, self.AXIS_NAME, type(speed))

        previous_sky_speed = self._sky_track_speed
        previous_motor_position = self._motor_position_raw

        with self._position_update_lock:
            if halt_motion:
                self._halt_motion()

            rounded_speed = self._set_tracking_speed(speed)

            applied_speed = rounded_speed if rounded_speed is not None else speed

            if update_sky_speed:
                self._sky_track_speed = applied_speed
            sky_speed = self._sky_track_speed

            motor_position = self._get_motor_raw_position()
            if not isinstance(motor_position, self.POS_CLS):
                raise ValueError("Motor position should be of type %s for %s axis, got %s", self.POS_CLS, self.AXIS_NAME, type(motor_position))
            self._motor_position_raw = motor_position.wrap()  # TODO: store if wrapping happens, need to compensate it in other axis if it happens in DEC

            self._last_update_s = Second.monotonic()

        requested_speed = speed if cmd.speed is None else cmd.speed

        self.logger.info(
            "Applied command: %s; speed: %.3f->%.3f; sky_speed: %.3f->%.3f; motor_position: %.3f-> %.3f",
            cmd.type, 
            requested_speed, applied_speed,
            previous_sky_speed, sky_speed,
            previous_motor_position, motor_position
        )

    def _do_log_telemetry(self):
        _logger = self.logger.getChild("telemetry")
        while self._working:
            if not self._is_motor_connected():
                time.sleep(self._TELEMETRY_INTERVAL_S)
                continue

            try:
                status = self._get_motor_status()
            except Exception:
                _logger.exception("While polling telemetry")
                time.sleep(self._TELEMETRY_INTERVAL_S)
                continue

            consistent = self._position_update_lock.acquire(timeout=.1)

            current_mount_position = self._mount_position_raw
            current_motor_position = self._motor_position_raw

            if consistent:
                self._position_update_lock.release()

            _logger.info(
                "%s: MNT(%s raw=%d) MTR(%s raw=%s) status=%s goto_active=%s consistent=%s",
                self.AXIS_NAME,
                self.POS_CLS(float(current_mount_position)),
                current_mount_position,
                self.POS_CLS(float(current_motor_position)),
                current_motor_position,
                str(status),
                getattr(self, "_goto_to", None) is not None,
                consistent,
            )

            time.sleep(self._TELEMETRY_INTERVAL_S)

    def _compensate_tracking_speed(self):
        _logger = self.logger.getChild("compensate")
        self._last_update_s = Second.monotonic()

        while self._working:
            if not self._is_motor_connected():
                time.sleep(float(self._RATE_COMPENSATE_INTERVAL_S))
                continue

            now = Second.monotonic()
            wait_timeout_s = max(Second(0.), self._RATE_COMPENSATE_INTERVAL_S - (now - self._last_update_s))

            try:
                command = self._axis_command_queue.get(timeout=float(wait_timeout_s))
            except queue.Empty:
                command = None
            if command:
                commands = [command]
                while True:
                    try:
                        commands.append(self._axis_command_queue.get_nowait())
                    except queue.Empty:
                        break

                for queued_command in commands:
                    try:
                        self._apply_axis_command(queued_command)
                    except Exception:
                        _logger.exception("While applying axis command: %s", queued_command)
                continue

            with self._position_update_lock:
                try:
                    motor_position = self._get_motor_raw_position().wrap()
                    self.logger.debug("Motor raw position: %f", motor_position)
                    now = Second.monotonic()
                except Exception as e:
                    _logger.warning("While get raw position: %s", e)
                    continue

                elapsed_s = now - self._last_update_s

                expected_delta_seconds = self._sky_track_speed * elapsed_s

                actual_delta_seconds = motor_position - self._motor_position_raw

                actual_delta_seconds *= self.COMPENSATE_MOTOR_SIGN

                delta = expected_delta_seconds - actual_delta_seconds

                self.logger.debug(
                    "Calculated delta by %.3fs: %.3f = exp=%.3f = (%.3fs * (%.3fs/s)) - act=%.3f = (%.3f - %.3f); MNT: %.3f",
                    elapsed_s,
                    delta, 
                    expected_delta_seconds, elapsed_s, self._sky_track_speed,
                    actual_delta_seconds, motor_position,  self._motor_position_raw,
                    self._mount_position_raw,
                )
                
                if abs(delta) < self._POSITION_DELTA_ACCEPTED_RATE_S * elapsed_s:
                    delta = self.POS_CLS(0)

                new_mount_position = (self._mount_position_raw + delta).wrap()

                try:
                    self.logger.info(
                        "Update mount position: %s -> %s (%.2f -> %.2f)",
                        self._mount_position_raw,
                        new_mount_position,
                        self._mount_position_raw, 
                        new_mount_position,
                    )
                except:
                    _logger.exception("While log")

                self._mount_position_raw = new_mount_position

                self._motor_position_raw = motor_position
                self._last_update_s = now

    def stop(self):
        if not self._working:
            return 

        self._working = False
        if self._telemetry_thread and self._telemetry_thread.is_alive():
            try:
                self._telemetry_thread.join(timeout=self._TELEMETRY_INTERVAL_S * 5)
            except Exception:
                self.logger.exception("While finishing telemetry thread")
        if self._compensate_thread and self._compensate_thread.is_alive():
            self._compensate_thread.join(timeout=float(self._RATE_COMPENSATE_INTERVAL_S * 5))

    def __del__(self):
        self.stop()


class LX200RAHandler(LX200AxisHandler[Ha, HaPerSecond]):
    POS_CLS = Ha
    SPEED_CLS = HaPerSecond
    AXIS_NAME = "Ra"
    DIRECTIONS = SkyDirection.ha_directions()
    COMPENSATE_MOTOR_SIGN = 1

    MIN_TRACKING_RATE = HaPerSecond(0)
    DEFAULT_TRACKING_RATE = STELLAR_SPEED
    MAX_TRACKING_RATE = STELLAR_SPEED * 2


class LX200DECHandler(LX200AxisHandler[Dec, DecPerSecond]):
    POS_CLS = Dec
    SPEED_CLS = DecPerSecond
    AXIS_NAME = "Dec"
    DIRECTIONS = SkyDirection.dec_directions()
    COMPENSATE_MOTOR_SIGN = -1

    MIN_TRACKING_RATE = DecPerSecond(-100)
    DEFAULT_TRACKING_RATE = DecPerSecond(0)
    MAX_TRACKING_RATE = DecPerSecond(100)


class LX200Handler(LX200Base):
    """
    All methods should be fast and non-blocking.
    """
    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._target_ra: Ha = Ha(0)
        self._target_dec: Dec = Dec(0)
        self._minimum_elevation: Dec = Dec(0)
        self._highest_elevation: Dec = Dec(90 * 60 * 60)

        self._manual_move_directions: list[SkyDirection] = []

        self._is_connected = False

    def connect(self) -> None:
        self._is_connected = True
    
    def _do_handle(self, cmd: LX200Commands, argument: Any) -> Any:
        result = None

        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                self._target_ra = Ha.from_string(position)
                result = True

            case LX200Commands.GET_TELESCOPE_DEC, _:
                result = self.get_telescope_dec()
            case LX200Commands.SET_TELESCOPE_DEC, position:
                self._target_dec = Dec.from_string(position)
                result = True

            case LX200Commands.SYNC, _:
                result = self.sync_telescope(self._target_ra, self._target_dec)
                result = "OK"
            case LX200Commands.SLEW, _:
                result = self.slew_to(self._target_ra, self._target_dec)
                result = False
                # But 1<below horison>#
                # But 2<below higher>#

            case LX200Commands.MOVE_EAST, _:
                if self.move_east():
                    if SkyDirection.EAST not in self._manual_move_directions:
                        self._manual_move_directions.append(SkyDirection.EAST)
                result = None
            case LX200Commands.MOVE_NORTH, _:
                if self.move_north():
                    if SkyDirection.NORTH not in self._manual_move_directions:
                        self._manual_move_directions.append(SkyDirection.NORTH)
                result = None
            case LX200Commands.MOVE_SOUTH, _:
                if self.move_south():
                    if SkyDirection.SOUTH not in self._manual_move_directions:
                        self._manual_move_directions.append(SkyDirection.SOUTH)
                result = None
            case LX200Commands.MOVE_WEST, _:
                if self.move_west():
                    if SkyDirection.WEST not in self._manual_move_directions:
                        self._manual_move_directions.append(SkyDirection.WEST)
                result = None

            case LX200Commands.HALT_ALL, _:
                self.halt_all()
                self._manual_move_directions.clear()
                result = None
            case LX200Commands.HALT_EAST, _:
                if SkyDirection.EAST in self._manual_move_directions:
                    self.halt_east()
                    self._manual_move_directions.remove(SkyDirection.EAST)
                result = None
            case LX200Commands.HALT_NORTH, _:
                if SkyDirection.NORTH in self._manual_move_directions:
                    self.halt_north()
                    self._manual_move_directions.remove(SkyDirection.NORTH)
                result = None
            case LX200Commands.HALT_SOUTH, _:
                if SkyDirection.SOUTH in self._manual_move_directions:
                    self.halt_south()
                    self._manual_move_directions.remove(SkyDirection.SOUTH)
                result = None
            case LX200Commands.HALT_WEST, _:
                if SkyDirection.WEST in self._manual_move_directions:
                    self.halt_west()
                    self._manual_move_directions.remove(SkyDirection.WEST)
                result = None

            case LX200Commands.GET_CALENDAR_FORMAT, _:
                result = self.get_calendar_format()
            case LX200Commands.GET_SITE1_NAME, _:
                result = self.get_site1_name()
            case LX200Commands.GET_TRACKING_RATE, _:
                result = self.get_tracking_rate()
            case LX200Commands.GET_CURRENT_SITE_LATITUDE, _:
                result = "+00*00"
            case LX200Commands.GET_CURRENT_SITE_LONGITUDE, _:
                result = "+000*00"
            case LX200Commands.SET_CURRENT_SITE_LONGTITUDE, _:
                result = True
            case LX200Commands.SET_CURRENT_SITE_LATITUDE, _:
                result = False

            case LX200Commands.GET_UTC_OFFSET_TIME, _:
                result = "+0"
            case LX200Commands.SET_UTC, _:
                result = True

            case LX200Commands.GET_LOCAL_TIME, _:
                result = "00:00:00"
            case LX200Commands.SET_LOCAL_TIME, _:
                result = True
            case LX200Commands.GET_CURRENT_DATE, _:
                result = "01/01/26"
            case LX200Commands.SET_LOCAL_DATE, _:
                result = True

            case LX200Commands.SET_MINIMUM_ELEVATION, position:
                match = re.fullmatch(r"([+-]?)(\d{2})\*", position)
                if not match:
                    result = False
                else:
                    sign = -1 if match.group(1) == "-" else 1
                    result = self.set_minimum_elevation(Dec(sign * int(match.group(2)) * 60 * 60))
            case LX200Commands.SET_HIGHEST_ELEVATION, position:
                match = re.fullmatch(r"([+-]?)(\d{2})\*", position)
                if not match:
                    result = False
                else:
                    sign = -1 if match.group(1) == "-" else 1
                    result = self.set_highest_elevation(Dec(sign * int(match.group(2)) * 60 * 60))

            case LX200Commands.SET_SLEW_TO_GUIDE, _:
                self.set_slew_to_guide()
                result = None
            case LX200Commands.SET_SLEW_TO_CENTER, _:
                self.set_slew_to_center()
                result = None
            case LX200Commands.SET_SLEW_TO_FIND, _:
                self.set_slew_to_find()
                result = None
            case LX200Commands.SET_SLEW_TO_MAX, _:
                self.set_slew_to_max()
                result = None
            case LX200Commands.GUIDE, data:
                direction = data[0].lower()
                ms = int(data[1:])

                match direction:
                    case 'w': 
                        self.guide_west(ms)
                    case 'e':
                        self.guide_east(ms)
                    case 'n':
                        self.guide_north(ms)
                    case 's':
                        self.guide_south(ms)
                    case _:
                        raise RuntimeError(f"Wrong guide direction: {direction}")
                
                result = None
            case LX200Commands.GET_DISTANCE, _:
                result = self.get_distance()
            case _:
                _logger.warning("Not implemented command: %s(%s)", cmd, argument)
                result = _LX200NotImplementedCommand(cmd)
        
        return result

    def handle(self, full_command: str) -> Any:
        _cmd, argument = full_command[:2], full_command[2:]
        try:
            cmd = LX200Commands(_cmd)
        except ValueError as e:
            raise RuntimeError(f"Unknown LX200 command: {_cmd}({argument})") from e
        _logger.info("Get command %s %s(%s)", cmd, cmd.name, argument)

        result = self._do_handle(cmd, argument)
        
        if isinstance(result, _LX200NotImplementedCommand):
            raise RuntimeError(f"Not implemented LX200 command: {cmd} {cmd.name}({argument})")
        elif result is not None:
            _logger.info("Answer command %s %s(%s) -> %s", cmd, cmd.name, argument, result)
        else:
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

        return result

    def set_minimum_elevation(self, position: Dec) -> bool:
        self._minimum_elevation = position
        return True

    def set_highest_elevation(self, position: Dec) -> bool:
        self._highest_elevation = position
        return True

    def stop(self):
        pass
