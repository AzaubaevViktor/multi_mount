from dataclasses import dataclass
import threading
from sky.axis import AxisMotionMode, AxisRA, AxisDEC, PointCoordinates
from sky.constants import STELLAR_SPEED
from sky.physics import AxisSpeed, DecPerSecond, HaPerSecond, Second, SkyDirection
from sky.polar_compensator import PolarCompensator

@dataclass
class GuideSpeed[_SPEED_CLS: AxisSpeed]:
    backward: _SPEED_CLS
    default: _SPEED_CLS
    forward: _SPEED_CLS

    def calculate_speed(self, direction: SkyDirection, pulse_width_s: Second, guide_interval_s: Second) -> _SPEED_CLS:
        guide_fration = pulse_width_s / guide_interval_s
        match direction:
            case SkyDirection.EAST | SkyDirection.NORTH:
                return (self.forward - self.default) * guide_fration + self.default
            case SkyDirection.WEST | SkyDirection.SOUTH:
                return (self.backward - self.default) * guide_fration + self.default
            case _:
                raise ValueError(f"Invalid direction: {direction}")


class Combiner:
    GUIDE_INTERVAL_S = Second(4)

    RA_GUIDE_SPEED = GuideSpeed[HaPerSecond](
        backward=STELLAR_SPEED - HaPerSecond(2),
        default=STELLAR_SPEED,
        forward=STELLAR_SPEED + HaPerSecond(2),
    )

    DEC_GUIDE_SPEED = GuideSpeed[DecPerSecond](
        backward=DecPerSecond(-100),
        default=DecPerSecond(0),
        forward=DecPerSecond(100),
    )

    def __init__(self, ra: AxisRA, dec: AxisDEC):
        self.ra = ra
        self.dec = dec

        if union := (set(self.ra.DIRECTIONS) & set(self.dec.DIRECTIONS)):
            raise ValueError(f"RA and DEC axes have common directions: {union}")

        self._polar_compensator = PolarCompensator()

        self._polar_compensator_thread: threading.Thread | None = None
        self._guide_updated = threading.Event()

    _POLAR_SKIP_MODES = frozenset({AxisMotionMode.SLEW, AxisMotionMode.GOTO})

    def _polar_compensation(self) -> None:
        while self.is_connected():
            if not self._guide_updated.wait(float(self.GUIDE_INTERVAL_S)):
                pass

            if self.ra.mode() in self._POLAR_SKIP_MODES or self.dec.mode() in self._POLAR_SKIP_MODES:
                self._guide_updated.clear()
                continue
            
            pos = self.get_position()
            self._polar_compensator.update_position(pos.ra, pos.dec)

            speeds = self._polar_compensator.get_guide_speeds()
            if speeds is not None:
                self.set_sky_speed(*speeds, update_polar_compensator=False)

            self._guide_updated.clear()

    def connect(self) -> None:
        self.ra.connect()
        self.dec.connect()
        if self._polar_compensator_thread is None or not self._polar_compensator_thread.is_alive():
            self._polar_compensator_thread = threading.Thread(target=self._polar_compensation, name="PolarCompensator")
        self._polar_compensator_thread.start()

    def is_connected(self) -> bool:
        return self.ra.is_connected() and self.dec.is_connected()

    def disconnect(self) -> None:
        self.ra.disconnect()
        self.dec.disconnect()
        self._guide_updated.set()
        if self._polar_compensator_thread is not None:
            self._polar_compensator_thread.join(float(self.GUIDE_INTERVAL_S))

    def get_position(self) -> PointCoordinates:
        pos_from_ra = self.ra.get_position()
        pos_from_dec = self.dec.get_position()

        # TODO: When DEC reflection crosses the pole, mirror RA by +12h as well.
        ra = (pos_from_ra.ra + pos_from_dec.ra)
        dec = pos_from_dec.dec + pos_from_ra.dec

        return PointCoordinates(ra=ra, dec=dec)
    
    def set_position(self, position: PointCoordinates) -> None:
        self.ra.set_position(PointCoordinates(ra=position.ra, dec=type(position.dec)(0)))
        self.dec.set_position(PointCoordinates(ra=type(position.ra)(0), dec=position.dec))

    def _dispatch_axis(self, direction: SkyDirection) -> AxisRA | AxisDEC:
        if direction in self.ra.DIRECTIONS:
            return self.ra
        if direction in self.dec.DIRECTIONS:
            return self.dec
        raise ValueError(f"Direction {direction} is not supported for both axes")

    def set_sky_speed(self, ra_speed: HaPerSecond | None, dec_speed: DecPerSecond | None, update_polar_compensator: bool = True) -> None:
        if ra_speed is not None:
            self.ra.change_speed(self.ra.FORWARD_DIRECTION, ra_speed, update_sky_speed=True)
        if dec_speed is not None:
            self.dec.change_speed(self.dec.FORWARD_DIRECTION, dec_speed, update_sky_speed=True)
        if update_polar_compensator:
            if ra_speed is not None:
                self._polar_compensator.guide_ra(ra_speed)
            if dec_speed is not None:
                self._polar_compensator.guide_dec(dec_speed)

    def move(self, direction: SkyDirection, speed: HaPerSecond | DecPerSecond) -> None:
        axis = self._dispatch_axis(direction)

        if isinstance(axis, AxisRA):
            if not isinstance(speed, HaPerSecond):
                raise ValueError(f"Speed should be of type {HaPerSecond} for {direction} direction, got {type(speed)}")
            axis.move(direction, speed)
        else:
            if not isinstance(speed, DecPerSecond):
                raise ValueError(f"Speed should be of type {DecPerSecond} for {direction} direction, got {type(speed)}")
            axis.move(direction, speed)

    def set_moving_speed(self, ra_speed: HaPerSecond, dec_speed: DecPerSecond) -> None:
        self.ra.change_speed(self.ra.FORWARD_DIRECTION, ra_speed, update_sky_speed=False)
        self.dec.change_speed(self.dec.FORWARD_DIRECTION, dec_speed, update_sky_speed=False)
    
    def goto_to(self, position: PointCoordinates) -> None:
        self.ra.goto_to(position)
        self.dec.goto_to(position)

    def guide(self, direction: SkyDirection, ms: int) -> None:
        axis = self._dispatch_axis(direction)

        if isinstance(axis, AxisRA):
            speed = self.RA_GUIDE_SPEED.calculate_speed(direction, Second.from_milliseconds(ms), self.GUIDE_INTERVAL_S)
            axis.change_speed(
                axis.FORWARD_DIRECTION, 
                speed, 
                update_sky_speed=True,
            )
            self._polar_compensator.guide_ra(speed)
        else:
            speed = self.DEC_GUIDE_SPEED.calculate_speed(direction, Second.from_milliseconds(ms), self.GUIDE_INTERVAL_S)
            axis.change_speed(
                axis.FORWARD_DIRECTION, 
                speed, 
                update_sky_speed=True,
            )
            self._polar_compensator.guide_dec(speed)
        
        self._guide_updated.set()
    
    def is_moving_to(self) -> bool:
        return self.ra.is_moving_to() or self.dec.is_moving_to()
    
    def halt_direction(self, direction: SkyDirection) -> None:
        axis = self._dispatch_axis(direction)
        axis.halt_direction(direction)
    
    def halt_all(self) -> None:
        """ Halt all drop current guide speeds """
        self.ra.halt_all()
        self.ra.change_speed(self.ra.FORWARD_DIRECTION, STELLAR_SPEED, update_sky_speed=True)
        self.dec.halt_all()
        self.dec.change_speed(self.dec.FORWARD_DIRECTION, DecPerSecond(0), update_sky_speed=True)

    def stop_all(self) -> None:
        """ Stop all motion including tracking and clear guide state """
        self.ra.change_speed(self.ra.FORWARD_DIRECTION, HaPerSecond(0), update_sky_speed=True)
        self.ra.halt_all()

        self.dec.change_speed(self.dec.FORWARD_DIRECTION, DecPerSecond(0), update_sky_speed=True)
        self.dec.halt_all()

        self._polar_compensator.eps_E = None
        self._polar_compensator.eps_N = None
        self._polar_compensator.ra_speed = None
        self._polar_compensator.dec_speed = None
        self._polar_compensator.stable_guide_ra_pulses_count = 0
        self._polar_compensator.stable_guide_dec_pulses_count = 0
        self._polar_compensator.last_guide_pulse = Second(0)
        self._polar_compensator.last_ra_guide_pulse = Second(0)
        self._polar_compensator.last_dec_guide_pulse = Second(0)

        self._guide_updated.set()
