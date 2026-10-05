import logging
from datetime import UTC, datetime
from typing import Any

from lx200.base import LX200Handler
from lx200.protocol import AlignmentMode
from pointing.service import PointingService, validate_solve
from sky.axis import PointCoordinates
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second, SkyDirection


class SkyLX200(LX200Handler):
    GUIDE_RA_SPEED = STELLAR_SPEED
    GUIDE_DEC_SPEED = DecPerSecond(5)

    CENTER_RA_SPEED = HaPerSecond(20 * float(STELLAR_SPEED))
    CENTER_DEC_SPEED = DecPerSecond(100)

    FIND_RA_SPEED = HaPerSecond(40 * float(STELLAR_SPEED))
    FIND_DEC_SPEED = DecPerSecond(1000)

    MAX_RA_SPEED = HaPerSecond(80 * float(STELLAR_SPEED))
    MAX_DEC_SPEED = DecPerSecond(2000)

    def __init__(self, combiner: Combiner, pointing: PointingService | None = None) -> None:
        super().__init__()
        self.logger = logging.getLogger(type(self).__name__)
        self._combiner = combiner
        self.pointing = pointing
        self._manual_ra_speed = self.GUIDE_RA_SPEED
        self._manual_dec_speed = self.GUIDE_DEC_SPEED

    def connect(self) -> None:
        self._combiner.connect()
        super().connect()

    def stop(self) -> None:
        if not self._is_connected:
            return
        self._combiner.disconnect()
        self._is_connected = False

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR

    def get_telescope_ra(self) -> Ha:
        return self._combiner.get_position().ra

    def monitor(self) -> dict[str, Any]:
        result = self._combiner.monitor()
        now = float(Second.monotonic())
        monitor = self.command_monitor()
        result["lx200"] = {
            "stats": [{"command": command, "count": count, "age_s": max(0, now - float(at)), "argument": argument} for command, count, at, argument in monitor["stats"]],
            "recent": [{"age_s": max(0, now - float(at)), "command": command} for at, command in monitor["recent"]],
        }
        return result

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        self.sync_from_solve(float(ra) / 3600, float(dec) / 3600, strict=False)
        return True

    def sync_from_solve(
        self, ra_hours: float, dec_deg: float, *, timestamp: datetime | None = None,
        measurement_id: str | None = None, strict: bool = True,
    ) -> dict[str, Any]:
        validate_solve(ra_hours, dec_deg, timestamp or datetime.now(UTC))
        result: dict[str, Any] = {"status": "unsupported", "sample_added": False}
        if self.pointing is not None:
            try:
                result = self.pointing.record_sync(ra_hours, dec_deg, timestamp=timestamp, measurement_id=measurement_id)
            except (ValueError, OSError):
                if strict:
                    raise
                self.logger.exception("Sensor calibration could not record SYNC")
                result = {"status": "calibration_error", "sample_added": False}
        self._combiner.set_position(PointCoordinates(ra=Ha(ra_hours * 3600), dec=Dec(dec_deg * 3600)))
        result["mount_synced"] = True
        return result

    def get_telescope_dec(self) -> Dec:
        return self._combiner.get_position().dec

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        self._combiner.goto_to(PointCoordinates(ra=ra, dec=dec))
        return True

    def set_slew_to_guide(self) -> bool:
        self._manual_ra_speed = self.GUIDE_RA_SPEED
        self._manual_dec_speed = self.GUIDE_DEC_SPEED
        self._combiner.set_moving_speed(self._manual_ra_speed, self._manual_dec_speed)
        return True

    def set_slew_to_center(self) -> bool:
        self._manual_ra_speed = self.CENTER_RA_SPEED
        self._manual_dec_speed = self.CENTER_DEC_SPEED
        self._combiner.set_moving_speed(self._manual_ra_speed, self._manual_dec_speed)
        return True

    def set_slew_to_find(self) -> bool:
        self._manual_ra_speed = self.FIND_RA_SPEED
        self._manual_dec_speed = self.FIND_DEC_SPEED
        self._combiner.set_moving_speed(self._manual_ra_speed, self._manual_dec_speed)
        return True

    def set_slew_to_max(self) -> bool:
        self._manual_ra_speed = self.MAX_RA_SPEED
        self._manual_dec_speed = self.MAX_DEC_SPEED
        self._combiner.set_moving_speed(self._manual_ra_speed, self._manual_dec_speed)
        return True

    def get_site1_name(self) -> str:
        return "sky_combiner"

    def get_distance(self) -> str:
        if self._combiner.is_moving_to():
            return "|"
        return ""

    def move_east(self) -> bool:
        self._combiner.move(SkyDirection.EAST, self._manual_ra_speed)
        return True

    def move_north(self) -> bool:
        self._combiner.move(SkyDirection.NORTH, self._manual_dec_speed)
        return True

    def move_south(self) -> bool:
        self._combiner.move(SkyDirection.SOUTH, self._manual_dec_speed)
        return True

    def move_west(self) -> bool:
        self._combiner.move(SkyDirection.WEST, self._manual_ra_speed)
        return True

    def halt_all(self) -> bool:
        self._combiner.halt_all()
        return True

    def stop_all(self) -> bool:
        self._combiner.stop_all()
        return True

    def halt_east(self) -> bool:
        self._combiner.halt_direction(SkyDirection.EAST)
        return True

    def halt_north(self) -> bool:
        self._combiner.halt_direction(SkyDirection.NORTH)
        return True

    def halt_south(self) -> bool:
        self._combiner.halt_direction(SkyDirection.SOUTH)
        return True

    def halt_west(self) -> bool:
        self._combiner.halt_direction(SkyDirection.WEST)
        return True

    def guide_east(self, ms: int) -> None:
        self._combiner.guide(SkyDirection.EAST, ms)

    def guide_north(self, ms: int) -> None:
        self._combiner.guide(SkyDirection.NORTH, ms)

    def guide_south(self, ms: int) -> None:
        self._combiner.guide(SkyDirection.SOUTH, ms)

    def guide_west(self, ms: int) -> None:
        self._combiner.guide(SkyDirection.WEST, ms)
