from abc import ABC, abstractmethod
from enum import IntEnum, StrEnum
import re
from typing import Self, Sequence, overload, Any
import time


""" 
DO NOT CHANGE VALUES IN INSTANCES! f.e. in __iX__
Often we send classes throught multiple abstract layers, and changing values may cause bugs.
For example, if we have a class Ha that represents hours in seconds,
    and we have a method that takes Ha and modifies it, it may cause bugs if we change the value of Ha in place. 
Instead, we should always return a new instance of Ha with the modified value. 
This way, we can ensure that the original value of Ha is not changed and we can avoid bugs that may arise from changing values in place.
"""


class _BasicAriphmetic(ABC):
    @abstractmethod
    def __init__(self, value: float) -> None:
        ...

    @abstractmethod
    def __float__(self) -> float:
        ...

    def __int__(self) -> int:
        return int(float(self))

    def __neg__(self) -> Self:
        return self.__class__(-float(self))
    
    def __abs__(self) -> Self:
        return self.__class__(abs(float(self)))
    
    @overload
    def __sub__(self, other: Self) -> Self: ...

    def __sub__(self, other: Any) -> Any:
        if isinstance(other, self.__class__):
            return self.__class__(float(self) - float(other))
        raise TypeError(f"Unsupported subtraction: {type(self)} - {type(other)}")

    @overload
    def __mul__(self, other: float) -> Self: ...

    def __mul__(self, other: Any) -> Any:
        if isinstance(other, float):
            return self.__class__(float(self) * other)
        raise TypeError(f"Unsupported multiplication: {type(self)} * {type(other)}")

    @overload
    def __add__(self, other: Self) -> Self: ...

    def __add__(self, other: Any) -> Any:
        if isinstance(other, self.__class__):
            return self.__class__(float(self) + float(other))
        raise TypeError(f"Unsupported addition: {type(self)} + {type(other)}")
    
    def __lt__(self, other: Self):
        if isinstance(other, self.__class__):
            return float(self) < float(other)
        raise TypeError(f"Unsupported comparison: {type(self)} < {type(other)}")
    
    def __eq__(self, value: Self) -> bool:
        if isinstance(value, self.__class__):
            return float(self) == float(value)
        raise TypeError(f"Unsupported comparison: {type(self)} == {type(value)}")
    
    @overload
    def __truediv__(self, other: float) -> Self: ...

    @overload
    def __truediv__(self, other: Self) -> float: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, float):
            return self.__class__(float(self) / other)
        if isinstance(other, self.__class__):
            return float(self) / float(other)
        raise TypeError(f"Unsupported division: {type(self)} / {type(other)}")


class Second(_BasicAriphmetic):
    MILLISECONDS_PER_SECOND = 1000

    def __init__(self, seconds: float):
        self.seconds = seconds
    
    def to_milliseconds(self) -> float:
        return self.seconds * self.MILLISECONDS_PER_SECOND
    
    @classmethod
    def from_milliseconds(cls, milliseconds: float) -> Self:
        return cls(milliseconds / cls.MILLISECONDS_PER_SECOND)
    
    @classmethod
    def monotonic(cls) -> Self:
        return cls(time.monotonic())
    
    def __float__(self) -> float:
        return self.seconds

    def __str__(self) -> str:
        return f"{self.seconds:.3f}s"


class AxisPos(_BasicAriphmetic):
    @abstractmethod
    def __init__(self, *args: float) -> None:
        ...
    
    @classmethod
    @abstractmethod
    def from_string(cls, s: str) -> Any:
        ...

    @abstractmethod
    def wrap(self) -> Self:
        ...

    @overload
    def __truediv__(self, other: Second) -> AxisSpeed: ...

    @overload
    def __truediv__(self, other: AxisSpeed) -> Second: ...

    @overload
    def __truediv__(self, other: float) -> Self: ...

    @overload
    def __truediv__(self, other: Self) -> float: ...


class AxisSpeed(_BasicAriphmetic):
    @overload
    def __mul__(self, other: Second) -> AxisPos: ...

    @overload
    def __mul__(self, other: float) -> Self: ...


class HaFormatError(ValueError):
    pass


_SECONDS_PER_HOUR = 3600
_HOURS_PER_DAY = 24
_SECONDS_PER_DAY = _HOURS_PER_DAY * _SECONDS_PER_HOUR
DEG_PER_HOUR = 360 / _HOURS_PER_DAY

SECONDS_PER_DAY = Second(_SECONDS_PER_DAY)


class Ha(AxisPos):
    __slots__ = ("_total_seconds",)

    HOURS_PATTERN = re.compile(r"^(\d{2}):(\d{2}):(\d{2})$")

    def __init__(self, seconds: float):
        if seconds > _SECONDS_PER_DAY:
            seconds %= _SECONDS_PER_DAY
        if seconds < -_SECONDS_PER_DAY:
            seconds %= -_SECONDS_PER_DAY

        self._total_seconds = seconds

    def __float__(self) -> float:
        return self._total_seconds
    
    def wrap(self) -> Self:
        return self.__class__(self._total_seconds % _SECONDS_PER_DAY)

    @overload
    def __truediv__(self, other: Second) -> AxisSpeed: ...

    @overload
    def __truediv__(self, other: AxisSpeed) -> Second: ...

    @overload
    def __truediv__(self, other: float) -> Self: ...

    @overload
    def __truediv__(self, other: Self) -> float: ...

    def __truediv__(self, other: Any) -> Any:
        try:
            super().__truediv__(other)
        except TypeError:
            pass
            
        if isinstance(other, Second): 
            return HaPerSecond(float(self) / float(other))
        if isinstance(other, HaPerSecond):
            return Second(float(self) / float(other))
        raise TypeError(f"Unsupported division: {type(self)} / {type(other)}")
    
    def _rounded_total_seconds(self) -> int:
        rounded_total_seconds = abs(int(round(self._total_seconds)))
        return rounded_total_seconds % _SECONDS_PER_DAY

    @property
    def hours(self) -> int:
        rounded_total_seconds = self._rounded_total_seconds()
        return rounded_total_seconds // _SECONDS_PER_HOUR
    
    def to_hours_deg(self) -> float:
        return self._total_seconds / _SECONDS_PER_HOUR * DEG_PER_HOUR

    @property
    def minutes(self) -> int:
        rounded_total_seconds = self._rounded_total_seconds()
        remainder = rounded_total_seconds % _SECONDS_PER_HOUR
        return remainder // 60

    @property
    def seconds(self) -> int:
        rounded_total_seconds = self._rounded_total_seconds()
        return rounded_total_seconds % 60

    @classmethod
    def from_string(cls, s: str) -> Self:
        match = cls.HOURS_PATTERN.match(s)
        if not match:
            raise HaFormatError(f"Invalid HH:MM:SS format: {s!r}")

        hours = int(match.group(1))
        minutes = int(match.group(2))
        seconds = int(match.group(3))
        return cls(hours * _SECONDS_PER_HOUR + minutes * 60 + seconds)

    def __str__(self) -> str:
        sign = "-" if self._total_seconds < 0 else ""
        return f"{sign}{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}"


class HaPerSecond(AxisSpeed):
    def __init__(self, seconds_per_second: float):
        self._total_ha_seconds_per_second = seconds_per_second

    def __float__(self) -> float:
        return self._total_ha_seconds_per_second
    
    def to_ha_deg_per_hour(self) -> HaDegPerHour:
        return HaDegPerHour(self._total_ha_seconds_per_second * DEG_PER_HOUR)

    @overload
    def __mul__(self, other: Second) -> Ha: ...

    @overload
    def __mul__(self, other: float) -> Self: ...

    def __mul__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return Ha(self._total_ha_seconds_per_second * other.seconds)
        if isinstance(other, float):
            return Ha(float(self) * other)
        raise TypeError(f"Unsupported multiplication: {type(self)} * {type(other)}")
    
    @overload
    def __truediv__(self, other: HaPerSecond) -> float: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, HaPerSecond):
            return float(self) / float(other)
        raise TypeError(f"Unsupported division: {type(self)} / {type(other)}")


class HaDegPerHour(AxisSpeed):
    def __init__(self, degrees_per_hour: float):
        self._degrees_per_hour = degrees_per_hour

    def __float__(self) -> float:
        return self._degrees_per_hour
    
    def to_ha_per_second(self) -> HaPerSecond:
        return HaPerSecond(self._degrees_per_hour * 15 / _SECONDS_PER_HOUR)


class Dec(AxisPos):
    DEC_PATTERN = re.compile(
        rf"^([+-])(\d{{2}})\*(\d{{2}}):(\d{{2}})$"
    )
    ARCSECONDS_PER_QUATER_CIRCLE = 90 * _SECONDS_PER_HOUR

    def __init__(self, arcseconds: float):
        if arcseconds > self.ARCSECONDS_PER_QUATER_CIRCLE:
            arcseconds %= self.ARCSECONDS_PER_QUATER_CIRCLE
        if arcseconds < -self.ARCSECONDS_PER_QUATER_CIRCLE:
            arcseconds %= -self.ARCSECONDS_PER_QUATER_CIRCLE
        self._total_arcseconds = arcseconds

    def __float__(self) -> float:
        return self._total_arcseconds
    
    def wrap(self) -> Self:
        """ Wrap around -90° to +90°, so that 91° becomes 89°, and -91° becomes -89°. """
        if self._total_arcseconds > self.ARCSECONDS_PER_QUATER_CIRCLE:
            return self.__class__(self.ARCSECONDS_PER_QUATER_CIRCLE * 2 - self._total_arcseconds)
        if self._total_arcseconds < -self.ARCSECONDS_PER_QUATER_CIRCLE:
            return self.__class__(-self.ARCSECONDS_PER_QUATER_CIRCLE * 2 - self._total_arcseconds)
        return self

    def _rounded_total_arcseconds(self) -> int:
        rounded_total_arcseconds = abs(int(round(self._total_arcseconds)))
        if rounded_total_arcseconds > self.ARCSECONDS_PER_QUATER_CIRCLE:
            rounded_total_arcseconds %= self.ARCSECONDS_PER_QUATER_CIRCLE
        return rounded_total_arcseconds

    @property
    def degrees(self) -> int:
        rounded_total_arcseconds = self._rounded_total_arcseconds()
        return rounded_total_arcseconds // _SECONDS_PER_HOUR
    
    def to_degrees(self) -> float:
        return self._total_arcseconds / _SECONDS_PER_HOUR
    
    @property
    def arcminutes(self) -> int:
        rounded_total_arcseconds = abs(self._rounded_total_arcseconds())
        remainder = rounded_total_arcseconds % _SECONDS_PER_HOUR
        return remainder // 60
    
    @property
    def arcseconds(self) -> int:
        rounded_total_arcseconds = abs(self._rounded_total_arcseconds())
        return rounded_total_arcseconds % 60
    
    @classmethod
    def from_string(cls, s: str) -> Self:
        match = cls.DEC_PATTERN.match(s)
        if not match:
            raise ValueError(f"Invalid DEC format: {s!r}")

        sign = -1 if match.group(1) == "-" else 1
        degrees = int(match.group(2))
        arcminutes = int(match.group(3))
        arcseconds = int(match.group(4))
        total_arcseconds = sign * ((degrees * _SECONDS_PER_HOUR) + (arcminutes * 60) + arcseconds)
        return cls(total_arcseconds)

    @overload
    def __truediv__(self, other: Second) -> DecPerSecond: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return DecPerSecond(self._total_arcseconds / other.seconds)
        raise TypeError(f"Unsupported division: {type(self)} / {type(other)}")
    
    def __str__(self) -> str:
        sign = "-" if self._total_arcseconds < 0 else "+"
        return f"{sign}{abs(self.degrees):02d}*{self.arcminutes:02d}:{self.arcseconds:02d}"


class DecPerSecond(AxisSpeed):
    def __init__(self, arcseconds_per_second: float):
        self._total_arcseconds_per_second = arcseconds_per_second

    def __float__(self) -> float:
        return self._total_arcseconds_per_second

    @overload
    def __mul__(self, other: Second) -> Dec: ...

    def __mul__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return Dec(self._total_arcseconds_per_second * other.seconds)
        raise TypeError(f"Unsupported multiplication: {type(self)} * {type(other)}")

    @overload
    def __truediv__(self, other: DecPerSecond) -> float: ...

    @overload
    def __truediv__(self, other: float) -> DecPerSecond: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, DecPerSecond):
            return float(self) / float(other)
        if isinstance(other, float):
            return DecPerSecond(float(self) / other)
        raise TypeError(f"Unsupported division: {type(self)} / {type(other)}")

class SkyDirection(StrEnum):
    EAST = "east"
    NORTH = "north"
    SOUTH = "south"
    WEST = "west"

    @classmethod
    def ha_directions(cls) -> Sequence[Self]:
        return (SkyDirection.EAST, SkyDirection.WEST)
    
    @classmethod
    def dec_directions(cls) -> Sequence[Self]:
        return (SkyDirection.NORTH, SkyDirection.SOUTH)


class Direction(IntEnum):
    FORWARD = 1
    BACKWARD = -1
    STOP = 0

# TODO: Add StepsPerSecond
# TODO: Add StepsPerHa / StepsPerDec
