from collections import deque
from dataclasses import dataclass
from enum import StrEnum
import logging
import re
import threading
from typing import TypedDict

from sky.physics import Dec, Ha, HaFormatError, Second, SkyDirection

from .protocol import AlignmentMode, Protocol


class LX200CommandMonitor(TypedDict):
    """Snapshot of LX200 command traffic, as read by the stdout dashboard.

    ``stats`` entries are ``(command name, times seen, last seen at, last argument)``,
    already sorted by frequency.
    """

    recent: list[tuple[Second, str]]
    guide: tuple[Second, str] | None
    stats: list[tuple[str, int, Second, str]]


class LX200Commands(StrEnum):
    # Telescope coordinates
    GET_TELECOPE_RA = "GR"
    SET_TELESCOPE_RA = "Sr"

    GET_TELESCOPE_DEC = "GD"
    SET_TELESCOPE_DEC = "Sd"

    # Telescope actions
    SYNC = "CM"
    SLEW = "MS"
    GET_DISTANCE = "D"

    # Manual motion
    MOVE_EAST = "Me"
    MOVE_NORTH = "Mn"
    MOVE_SOUTH = "Ms"
    MOVE_WEST = "Mw"

    # Halt motion
    HALT_ALL = "Q"
    HALT_EAST = "Qe"
    HALT_NORTH = "Qn"
    HALT_SOUTH = "Qs"
    HALT_WEST = "Qw"

    # LX200 info and site settings
    GET_CALENDAR_FORMAT = "Gc"
    GET_SITE1_NAME = "GM"

    GET_TRACKING_RATE = "GT"
    GET_CURRENT_SITE_LATITUDE = "Gt"
    GET_CURRENT_SITE_LONGITUDE = "Gg"
    GET_UTC_OFFSET_TIME = "GG"
    GET_LOCAL_TIME = "GL"
    GET_CURRENT_DATE = "GC"

    SET_CURRENT_SITE_LONGTITUDE = "Sg"
    SET_CURRENT_SITE_LATITUDE = "St"
    SET_UTC = "SG"

    SET_LOCAL_TIME = "SL"
    SET_LOCAL_DATE = "SC"

    # Elevation limits
    SET_MINIMUM_ELEVATION = "Sh"
    SET_HIGHEST_ELEVATION = "So"

    # Slew rates
    SET_SLEW_TO_GUIDE = "RG"
    SET_SLEW_TO_CENTER = "RC"
    SET_SLEW_TO_FIND = "RM"
    SET_SLEW_TO_MAX = "RS"

    # Guide pulse
    GUIDE = "Mg"


@dataclass(frozen=True)
class LX200SlewResult:
    """The answer to ``:MS#``, which is *not* a 0/1 acknowledgement.

    LX200 defines it as: ``0`` alone when the slew was accepted, and
    ``<code><human readable reason>#`` when it was not -- ``1`` for a target
    below the horizon, ``2`` for a target above the highest elevation limit.
    Note that ``0`` therefore means *success*, which is the opposite of the
    ``Sr``/``Sd`` acknowledgements, and is why ``MS`` needs its own type: the
    handler used to answer a bare ``False``, which serialises to ``"0"`` and so
    reported every slew as accepted no matter what the mount said.
    """

    code: int
    reason: str = ""

    @property
    def accepted(self) -> bool:
        return self.code == 0

    @classmethod
    def accept(cls) -> "LX200SlewResult":
        return cls(0)

    @classmethod
    def reject(cls, reason: str) -> "LX200SlewResult":
        return cls(1, reason)

    @classmethod
    def above_limit(cls, reason: str) -> "LX200SlewResult":
        return cls(2, reason)

    def to_wire(self) -> str:
        if self.accepted:
            return "0"
        return f"{self.code}{self.reason}{Protocol.TERMINATOR}"

    def __str__(self) -> str:
        # For the log and the manual console; the wire format is `to_wire()`.
        return "accepted" if self.accepted else f"rejected ({self.code}): {self.reason}"


# What a single LX200 command answers with: a coordinate (GR/GD), a ready-made ASCII payload
# (GT/GM/Gc/...), a 0/1 acknowledgement (Sr/Sd/...), an `MS` slew status, or nothing at all
# (motion commands, which the protocol leaves unanswered).
LX200Answer = Ha | Dec | bool | str | LX200SlewResult | None


class LX200Error(Exception):
    pass


class LX200BadCommandError(LX200Error):
    """The client sent something this handler will not execute.

    Carries the answer the connection should send instead, because LX200 has no
    generic error frame: a rejected ``Sr``/``Sd``/``Sh`` value is a plain ``0``,
    while a command the protocol leaves unanswered (``Mg``, the motion commands)
    stays unanswered. Raising something the socket loop understands is the whole
    point -- a bare ``IndexError`` out of ``Mg`` used to kill the client thread.
    """

    def __init__(self, message: str, answer: LX200Answer = None) -> None:
        super().__init__(message)
        self.answer = answer


class LX200UnknownCommandError(LX200BadCommandError):
    """Two leading characters that are not a command this handler implements."""


_logger = logging.getLogger("lx200")


class LX200Base:
    def connect(self) -> None:
        raise NotImplementedError()

    def is_connected(self) -> bool:
        raise NotImplementedError()

    def stop(self) -> None:
        raise NotImplementedError()

    def __del__(self) -> None:
        self.stop()

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

    def get_telescope_ra(self) -> Ha:
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

    def move_east(self) -> bool:
        raise NotImplementedError()

    def move_north(self) -> bool:
        raise NotImplementedError()

    def move_south(self) -> bool:
        raise NotImplementedError()

    def move_west(self) -> bool:
        raise NotImplementedError()

    def halt_all(self) -> bool:
        raise NotImplementedError()

    def stop_all(self) -> bool:
        return self.halt_all()

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


class LX200Handler(LX200Base):
    """
    All methods should be fast and non-blocking.
    """
    #: `Sh`/`So` payload: an optionally signed two-digit degree count, `+45*`.
    _ELEVATION_PATTERN = re.compile(r"([+-]?)(\d{2})\*")
    #: `Mg` payload: one direction letter plus a pulse width in milliseconds.
    _GUIDE_PATTERN = re.compile(r"([nsewNSEW])(\d{1,5})")
    #: `St` payload: `sDD*MM`. `Sg` payload: `sDDD*MM` (the sign is optional in both).
    _LATITUDE_PATTERN = re.compile(r"([+-]?)(\d{2})\*(\d{2})")
    _LONGITUDE_PATTERN = re.compile(r"([+-]?)(\d{3})\*(\d{2})")

    DEFAULT_SITE_LATITUDE = "+00*00"
    DEFAULT_SITE_LONGITUDE = "+000*00"

    def __init__(self) -> None:
        self.logger = logging.getLogger(type(self).__name__)
        self._target_ra = Ha(0)
        self._target_dec = Dec(0)
        self._minimum_elevation = Dec(0)
        self._highest_elevation = Dec(90 * 60 * 60)
        self._site_latitude = self.DEFAULT_SITE_LATITUDE
        self._site_longitude = self.DEFAULT_SITE_LONGITUDE
        self._manual_move_directions: list[SkyDirection] = []
        self._monitor_lock = threading.RLock()
        self._recent_commands: deque[tuple[Second, str]] = deque(maxlen=8)
        self._last_guide_command: tuple[Second, str] | None = None
        self._command_stats: dict[str, tuple[int, Second, str]] = {}

        self._is_connected = False

    def connect(self) -> None:
        self._is_connected = True

    def is_connected(self) -> bool:
        return self._is_connected

    def begin_client_session(self, client: str = "") -> None:
        """Drop everything the *previous* client left behind.

        Target RA/DEC and the active manual-move list are per-conversation state:
        one client's `Sr`/`Sd` is what the next `CM`/`MS` acts on, and the manual
        list decides which axis a bare `Qw` halts. Carrying them across
        connections means a client that died mid-slew makes the next one sync to
        a target it never set.
        """
        with self._monitor_lock:
            self._target_ra = Ha(0)
            self._target_dec = Dec(0)
            self._manual_move_directions.clear()
        _logger.info("New LX200 client session%s, target and manual-move state cleared", f" ({client})" if client else "")

    # ------------------------------------------------------------------
    # Argument parsing. Separate from the dispatch below on purpose: every one of
    # these is reachable from the network, so a malformed payload has to come back
    # as an LX200BadCommandError the socket loop can answer, never as a raw
    # IndexError/ValueError escaping into the connection thread.
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_target_ra(argument: str) -> Ha:
        try:
            return Ha.from_string(argument)
        except (HaFormatError, ValueError) as error:
            raise LX200BadCommandError(f"Sr{argument!r} is not an LX200 HH:MM:SS", answer=False) from error

    @staticmethod
    def _parse_target_dec(argument: str) -> Dec:
        try:
            return Dec.from_string(argument)
        except ValueError as error:
            raise LX200BadCommandError(f"Sd{argument!r} is not an LX200 sDD*MM:SS", answer=False) from error

    @classmethod
    def _parse_elevation(cls, command: LX200Commands, argument: str) -> Dec:
        match = cls._ELEVATION_PATTERN.fullmatch(argument)
        if match is None:
            raise LX200BadCommandError(f"{command.value}{argument!r} is not an LX200 sDD*", answer=False)
        sign = -1 if match.group(1) == "-" else 1
        return Dec(sign * int(match.group(2)) * 60 * 60)

    @classmethod
    def _parse_guide(cls, argument: str) -> tuple[SkyDirection, int]:
        match = cls._GUIDE_PATTERN.fullmatch(argument)
        if match is None:
            # This is the crash from PLAN.md #30: `Mg` used to index argument[0] and
            # int() the rest, so `:Mg#` raised IndexError and `:Mgx#` raised ValueError
            # straight out of the connection thread.
            raise LX200BadCommandError(f"Mg{argument!r} is not <direction><milliseconds>")
        direction = {
            "n": SkyDirection.NORTH,
            "s": SkyDirection.SOUTH,
            "e": SkyDirection.EAST,
            "w": SkyDirection.WEST,
        }[match.group(1).lower()]
        return direction, int(match.group(2))

    @classmethod
    def _parse_site_latitude(cls, argument: str) -> str:
        match = cls._LATITUDE_PATTERN.fullmatch(argument)
        if match is None:
            raise LX200BadCommandError(f"St{argument!r} is not an LX200 sDD*MM", answer=False)
        sign = "-" if match.group(1) == "-" else "+"
        return f"{sign}{match.group(2)}*{match.group(3)}"

    @classmethod
    def _parse_site_longitude(cls, argument: str) -> str:
        match = cls._LONGITUDE_PATTERN.fullmatch(argument)
        if match is None:
            raise LX200BadCommandError(f"Sg{argument!r} is not an LX200 sDDD*MM", answer=False)
        sign = "-" if match.group(1) == "-" else "+"
        return f"{sign}{match.group(2)}*{match.group(3)}"

    @staticmethod
    def _get_opposite_manual_direction(direction: SkyDirection) -> SkyDirection:
        match direction:
            case SkyDirection.EAST:
                return SkyDirection.WEST
            case SkyDirection.WEST:
                return SkyDirection.EAST
            case SkyDirection.NORTH:
                return SkyDirection.SOUTH
            case SkyDirection.SOUTH:
                return SkyDirection.NORTH

    def _remember_manual_move_direction(self, direction: SkyDirection) -> None:
        opposite_direction = self._get_opposite_manual_direction(direction)
        if opposite_direction in self._manual_move_directions:
            self._manual_move_directions.remove(opposite_direction)

        if direction not in self._manual_move_directions:
            self._manual_move_directions.append(direction)

    def _halt_manual_direction_if_active(self, *directions: SkyDirection) -> None:
        for direction in directions:
            if direction not in self._manual_move_directions:
                continue

            match direction:
                case SkyDirection.EAST:
                    self.halt_east()
                case SkyDirection.WEST:
                    self.halt_west()
                case SkyDirection.NORTH:
                    self.halt_north()
                case SkyDirection.SOUTH:
                    self.halt_south()

            self._manual_move_directions.remove(direction)
            break
        else:
            self.logger.debug("No active manual direction for %s, fallback to halt_all", directions)
            self.halt_all()
            self._manual_move_directions.clear()
    
    def _slew_to_target(self) -> LX200SlewResult:
        """Run `:MS#` and turn the mount's answer into the status LX200 asks for."""
        self._manual_move_directions.clear()
        # Codes 1 and 2 (below horizon / above the highest elevation) need an
        # altitude model this handler does not have: `Sh`/`So` are stored, but
        # turning a target RA/DEC into an altitude needs the site latitude and the
        # local sidereal time, and `GL`/`GC` are still constants here. So the only
        # rejection it can honestly report is the one the mount itself reports.
        if not self.slew_to(self._target_ra, self._target_dec):
            return LX200SlewResult.reject("mount refused the slew")
        return LX200SlewResult.accept()

    def _do_handle(self, cmd: LX200Commands, argument: str, now: Second | None = None) -> LX200Answer:
        result: LX200Answer = None
        if now is None:
            now = Second.monotonic()

        match (cmd, argument):
            case LX200Commands.GET_TELECOPE_RA, _:
                result = self.get_telescope_ra()
            case LX200Commands.SET_TELESCOPE_RA, position:
                self._target_ra = self._parse_target_ra(position)
                result = True

            case LX200Commands.GET_TELESCOPE_DEC, _:
                result = self.get_telescope_dec()
            case LX200Commands.SET_TELESCOPE_DEC, position:
                self._target_dec = self._parse_target_dec(position)
                result = True
            case LX200Commands.SYNC, _:
                self.sync_telescope(self._target_ra, self._target_dec)
                result = "OK"
            case LX200Commands.SLEW, _:
                result = self._slew_to_target()

            case LX200Commands.MOVE_EAST, _:
                if self.move_east():
                    self._remember_manual_move_direction(SkyDirection.EAST)
            case LX200Commands.MOVE_NORTH, _:
                if self.move_north():
                    self._remember_manual_move_direction(SkyDirection.NORTH)
            case LX200Commands.MOVE_SOUTH, _:
                if self.move_south():
                    self._remember_manual_move_direction(SkyDirection.SOUTH)
            case LX200Commands.MOVE_WEST, _:
                if self.move_west():
                    self._remember_manual_move_direction(SkyDirection.WEST)
            case LX200Commands.HALT_ALL, _:
                self.halt_all()
                self._manual_move_directions.clear()
            case LX200Commands.HALT_EAST, _:
                self._halt_manual_direction_if_active(SkyDirection.WEST, SkyDirection.EAST)
            case LX200Commands.HALT_NORTH, _:
                self._halt_manual_direction_if_active(SkyDirection.NORTH)
            case LX200Commands.HALT_SOUTH, _:
                self._halt_manual_direction_if_active(SkyDirection.SOUTH)
            case LX200Commands.HALT_WEST, _:
                self._halt_manual_direction_if_active(SkyDirection.EAST, SkyDirection.WEST)
            case LX200Commands.GET_CALENDAR_FORMAT, _:
                result = self.get_calendar_format()
            case LX200Commands.GET_SITE1_NAME, _:
                result = self.get_site1_name()
            case LX200Commands.GET_TRACKING_RATE, _:
                result = self.get_tracking_rate()
            case LX200Commands.GET_CURRENT_SITE_LATITUDE, _:
                result = self._site_latitude
            case LX200Commands.GET_CURRENT_SITE_LONGITUDE, _:
                result = self._site_longitude
            # `Sg` used to answer 1 and `St` 0 for the same kind of payload, and
            # neither stored anything, so `Gt`/`Gg` contradicted whatever the client
            # had just set. Both now parse, store and echo back.
            case LX200Commands.SET_CURRENT_SITE_LONGTITUDE, site:
                self._site_longitude = self._parse_site_longitude(site)
                result = True
            case LX200Commands.SET_CURRENT_SITE_LATITUDE, site:
                self._site_latitude = self._parse_site_latitude(site)
                result = True

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
                result = self.set_minimum_elevation(self._parse_elevation(cmd, position))
            case LX200Commands.SET_HIGHEST_ELEVATION, position:
                result = self.set_highest_elevation(self._parse_elevation(cmd, position))

            case LX200Commands.SET_SLEW_TO_GUIDE, _:
                self.set_slew_to_guide()
            case LX200Commands.SET_SLEW_TO_CENTER, _:
                self.set_slew_to_center()
            case LX200Commands.SET_SLEW_TO_FIND, _:
                self.set_slew_to_find()
            case LX200Commands.SET_SLEW_TO_MAX, _:
                self.set_slew_to_max()
            case LX200Commands.GUIDE, data:
                direction, ms = self._parse_guide(data)
                match direction:
                    case SkyDirection.WEST:
                        self.guide_west(ms)
                    case SkyDirection.EAST:
                        self.guide_east(ms)
                    case SkyDirection.NORTH:
                        self.guide_north(ms)
                    case SkyDirection.SOUTH:
                        self.guide_south(ms)

            case LX200Commands.GET_DISTANCE, _:
                result = self.get_distance()
            case _:
                raise LX200UnknownCommandError(f"Not implemented LX200 command: {cmd} {cmd.name}({argument})")

        return result

    @staticmethod
    def parse(full_command: str) -> tuple[LX200Commands, str]:
        """Split `<cmd><argument>` and name the command. Parsing only, no side effects."""
        name, argument = full_command[:2], full_command[2:]
        try:
            return LX200Commands(name), argument
        except ValueError as error:
            raise LX200UnknownCommandError(f"Unknown LX200 command: {name}({argument})") from error

    def handle(self, full_command: str) -> LX200Answer:
        cmd, argument = self.parse(full_command)

        now = Second.monotonic()
        with self._monitor_lock:
            count, _last_at, _last_argument = self._command_stats.get(cmd.value, (0, now, argument))
            self._command_stats[cmd.value] = (count + 1, now, argument)

            if cmd == LX200Commands.GUIDE:
                self._last_guide_command = (now, full_command)
            elif cmd not in {LX200Commands.GET_TELECOPE_RA, LX200Commands.GET_TELESCOPE_DEC}:
                self._recent_commands.append((now, full_command))

            result = self._do_handle(cmd, argument, now)

        # Clients poll GR/GD/D at ~1 Hz, that is thousands of records per session which bury the
        # meaningful commands (PLAN.md §1 П7). Everything else stays visible on INFO.
        level = logging.DEBUG if cmd in {LX200Commands.GET_TELECOPE_RA, LX200Commands.GET_TELESCOPE_DEC, LX200Commands.GET_DISTANCE} else logging.INFO

        _logger.log(level, "Get command %s %s(%s)", cmd, cmd.name, argument)

        if result is not None:
            _logger.log(level, "Answer command %s %s(%s) -> %s", cmd, cmd.name, argument, result)
        else:
            _logger.warning("Empty responce: %s %s(%s) -> ∅", cmd, cmd.name, argument)

        return result

    def set_minimum_elevation(self, position: Dec) -> bool:
        self._minimum_elevation = position
        return True

    def set_highest_elevation(self, position: Dec) -> bool:
        self._highest_elevation = position
        return True

    def stop(self) -> None:
        pass

    def command_monitor(self) -> LX200CommandMonitor:
        with self._monitor_lock:
            return {
                "recent": list(self._recent_commands),
                "guide": self._last_guide_command,
                "stats": [
                    (LX200Commands(command).name, count, last_at, argument)
                    for command, (count, last_at, argument) in sorted(
                        self._command_stats.items(),
                        key=lambda item: (-item[1][0], -float(item[1][1]), item[0]),
                    )
                ],
            }
