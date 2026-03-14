from __future__ import annotations

from dataclasses import dataclass

from lx200.base import ClockSiteState, LX200Base
from sky.combiner import Combiner, GuideDirection
from sky.physics import Dec, PointCoordinates


@dataclass(slots=True)
class MountLimits:
    highest_elevation: Dec = Dec(90.0)
    minimum_elevation: Dec = Dec(-90.0)


class SkyLX200(LX200Base):
    def __init__(self, combiner: Combiner) -> None:
        self.combiner = combiner
        self._clock_site = ClockSiteState()
        self._limits = MountLimits()

    def connect(self) -> None:
        self.combiner.connect()

    def stop(self) -> None:
        self.combiner.disconnect()

    def handle_alignment(self) -> str:
        return "P"

    def current_coordinates(self) -> PointCoordinates:
        return self.combiner.position()

    def sync_telescope(self, coordinates: PointCoordinates) -> None:
        self.combiner.sync_to(coordinates)

    def slew_to(self, coordinates: PointCoordinates) -> bool:
        self.combiner.goto_to(coordinates)
        return False

    def set_rate_preset(self, preset: str) -> None:
        self.combiner.set_rate_preset(preset)

    def move(self, direction: GuideDirection) -> None:
        self.combiner.move(direction)

    def halt(self, direction: GuideDirection | None = None) -> None:
        if direction is None:
            self.combiner.halt_all()
            return
        if direction in {GuideDirection.EAST, GuideDirection.WEST}:
            self.combiner.halt_ra()
            return
        self.combiner.halt_dec()

    def guide(self, direction: GuideDirection, milliseconds: int) -> None:
        self.combiner.guide(direction, milliseconds)

    def clock_site_state(self) -> ClockSiteState:
        return self._clock_site

    def update_clock_site_state(self, **values: str) -> None:
        for key, value in values.items():
            if not hasattr(self._clock_site, key):
                raise AttributeError(f"Unknown clock/site field: {key}")
            setattr(self._clock_site, key, value)

    def set_highest_elevation(self, value: Dec) -> None:
        self._limits.highest_elevation = value

    def set_minimum_elevation(self, value: Dec) -> None:
        self._limits.minimum_elevation = value

    @property
    def limits(self) -> MountLimits:
        return self._limits
