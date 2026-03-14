from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from sky.combiner import GuideDirection
from sky.physics import Dec, Ha, PointCoordinates

from .protocol import LX200Command, LX200Reply, bool_reply, empty_reply, string_reply


@dataclass(slots=True)
class ClockSiteState:
    longitude: str = "+000*00"
    latitude: str = "+00*00"
    utc_offset: str = "+00.0"
    local_time: str = "00:00:00"
    local_date: str = "01/01/26"
    site_name: str = "Rebuilt Multi-Mount"
    telescope_name: str = "Hybrid Mount"
    clock_format: str = "24"


class LX200Base(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def handle_alignment(self) -> str:
        raise NotImplementedError

    @abstractmethod
    def current_coordinates(self) -> PointCoordinates:
        raise NotImplementedError

    @abstractmethod
    def sync_telescope(self, coordinates: PointCoordinates) -> None:
        raise NotImplementedError

    @abstractmethod
    def slew_to(self, coordinates: PointCoordinates) -> bool:
        raise NotImplementedError

    @abstractmethod
    def set_rate_preset(self, preset: str) -> None:
        raise NotImplementedError

    @abstractmethod
    def move(self, direction: GuideDirection) -> None:
        raise NotImplementedError

    @abstractmethod
    def halt(self, direction: GuideDirection | None = None) -> None:
        raise NotImplementedError

    @abstractmethod
    def guide(self, direction: GuideDirection, milliseconds: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def clock_site_state(self) -> ClockSiteState:
        raise NotImplementedError

    @abstractmethod
    def update_clock_site_state(self, **values: str) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_highest_elevation(self, value: Dec) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_minimum_elevation(self, value: Dec) -> None:
        raise NotImplementedError


class LX200Handler:
    def __init__(self, lx200: LX200Base) -> None:
        self.lx200 = lx200
        self.target_ra = Ha(0.0)
        self.target_dec = Dec(0.0)
        self.manual_directions: set[GuideDirection] = set()

    def handle(self, command: str) -> bytes:
        reply = self._do_handle(command)
        return reply.to_bytes()

    def _do_handle(self, command: str) -> LX200Reply:
        if command == LX200Command.GET_RA.value:
            return string_reply(self.lx200.current_coordinates().ra.to_lx200())
        if command == LX200Command.GET_DEC.value:
            return string_reply(self.lx200.current_coordinates().dec.to_lx200())
        if command.startswith(LX200Command.SET_RA.value):
            self.target_ra = Ha.from_lx200(command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_DEC.value):
            self.target_dec = Dec.from_lx200(command[2:])
            return bool_reply(True)
        if command == LX200Command.SYNC.value:
            self.lx200.sync_telescope(PointCoordinates(ra=self.target_ra, dec=self.target_dec))
            return string_reply("Sync complete")
        if command == LX200Command.SLEW.value:
            self.lx200.slew_to(PointCoordinates(ra=self.target_ra, dec=self.target_dec))
            return bool_reply(False)
        if command == LX200Command.STATUS.value:
            return empty_reply()
        if command == LX200Command.RATE_GUIDE.value:
            self.lx200.set_rate_preset("guide")
            return bool_reply(True)
        if command == LX200Command.RATE_CENTER.value:
            self.lx200.set_rate_preset("center")
            return bool_reply(True)
        if command == LX200Command.RATE_FIND.value:
            self.lx200.set_rate_preset("find")
            return bool_reply(True)
        if command == LX200Command.RATE_MAX.value:
            self.lx200.set_rate_preset("max")
            return bool_reply(True)
        if command == LX200Command.MOVE_EAST.value:
            self.manual_directions.add(GuideDirection.EAST)
            self.lx200.move(GuideDirection.EAST)
            return empty_reply()
        if command == LX200Command.MOVE_WEST.value:
            self.manual_directions.add(GuideDirection.WEST)
            self.lx200.move(GuideDirection.WEST)
            return empty_reply()
        if command == LX200Command.MOVE_NORTH.value:
            self.manual_directions.add(GuideDirection.NORTH)
            self.lx200.move(GuideDirection.NORTH)
            return empty_reply()
        if command == LX200Command.MOVE_SOUTH.value:
            self.manual_directions.add(GuideDirection.SOUTH)
            self.lx200.move(GuideDirection.SOUTH)
            return empty_reply()
        if command == LX200Command.HALT_ALL.value:
            self.manual_directions.clear()
            self.lx200.halt()
            return empty_reply()
        if command == LX200Command.HALT_EAST.value:
            self.manual_directions.discard(GuideDirection.EAST)
            self.lx200.halt(GuideDirection.EAST)
            return empty_reply()
        if command == LX200Command.HALT_WEST.value:
            self.manual_directions.discard(GuideDirection.WEST)
            self.lx200.halt(GuideDirection.WEST)
            return empty_reply()
        if command == LX200Command.HALT_NORTH.value:
            self.manual_directions.discard(GuideDirection.NORTH)
            self.lx200.halt(GuideDirection.NORTH)
            return empty_reply()
        if command == LX200Command.HALT_SOUTH.value:
            self.manual_directions.discard(GuideDirection.SOUTH)
            self.lx200.halt(GuideDirection.SOUTH)
            return empty_reply()
        if command.startswith(LX200Command.GUIDE.value):
            direction = GuideDirection(command[2].lower())
            self.lx200.guide(direction, int(command[3:]))
            return bool_reply(True)
        if command == LX200Command.GET_CLOCK_FORMAT.value:
            return string_reply(self.lx200.clock_site_state().clock_format)
        if command == LX200Command.GET_SITE_NAME.value:
            return string_reply(self.lx200.clock_site_state().site_name)
        if command == LX200Command.GET_TELESCOPE_NAME.value:
            return string_reply(self.lx200.clock_site_state().telescope_name)
        if command == LX200Command.GET_LATITUDE.value:
            return string_reply(self.lx200.clock_site_state().latitude)
        if command == LX200Command.GET_LONGITUDE.value:
            return string_reply(self.lx200.clock_site_state().longitude)
        if command == LX200Command.GET_UTC_OFFSET.value:
            return string_reply(self.lx200.clock_site_state().utc_offset)
        if command == LX200Command.GET_TIME.value:
            return string_reply(self.lx200.clock_site_state().local_time)
        if command == LX200Command.GET_DATE.value:
            return string_reply(self.lx200.clock_site_state().local_date)
        if command.startswith(LX200Command.SET_LATITUDE.value):
            self.lx200.update_clock_site_state(latitude=command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_LONGITUDE.value):
            self.lx200.update_clock_site_state(longitude=command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_UTC_OFFSET.value):
            self.lx200.update_clock_site_state(utc_offset=command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_TIME.value):
            self.lx200.update_clock_site_state(local_time=command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_DATE.value):
            self.lx200.update_clock_site_state(local_date=command[2:])
            return bool_reply(True)
        if command.startswith(LX200Command.SET_HIGHEST_ELEVATION.value):
            self.lx200.set_highest_elevation(Dec.from_lx200(command[2:]))
            return bool_reply(True)
        if command.startswith(LX200Command.SET_MINIMUM_ELEVATION.value):
            self.lx200.set_minimum_elevation(Dec.from_lx200(command[2:]))
            return bool_reply(True)
        return empty_reply()
