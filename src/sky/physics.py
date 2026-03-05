from abc import ABC, abstractmethod
import re
from typing import Self, overload, Any


class Second:
    def __init__(self, seconds: float):
        self.seconds = seconds


class AxisPos(ABC):
    @abstractmethod
    def __init__(self, *args: float) -> None:
        ...
    
    @classmethod
    @abstractmethod
    def from_string(cls, s: str) -> Any:
        ...

    @abstractmethod
    def to_raw(self) -> float:
        ...
    
    def __neg__(self):
        return self.__class__(-self.to_raw())


class AxisSpeed(ABC):
    @abstractmethod
    def __init__(self, *args: float) -> None:
        ...


class HaFormatError(ValueError):
    pass


class HA(AxisPos):
    __slots__ = ("_total_seconds",)

    SECONDS_PER_CIRCLE = 24 * 3600
    HOURS_PATTERN = re.compile(r"^(\d{2}):(\d{2}):(\d{2})$")

    def __init__(self, seconds: float):
        if seconds > self.SECONDS_PER_CIRCLE:
            seconds %= self.SECONDS_PER_CIRCLE
        if seconds < -self.SECONDS_PER_CIRCLE:
            seconds %= -self.SECONDS_PER_CIRCLE

        self._total_seconds = seconds

    def to_raw(self) -> float:
        return self._total_seconds

    @overload
    def __truediv__(self, other: Second) -> HAPerSecond: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return HAPerSecond(self._total_seconds / other.seconds)
        raise NotImplementedError(f"Unsupported division: {type(self)} / {type(other)}")
    
    def _rounded_total_seconds(self) -> int:
        rounded_total_seconds = abs(int(round(self._total_seconds)))
        return rounded_total_seconds % self.SECONDS_PER_CIRCLE

    @property
    def hours(self) -> int:
        rounded_total_seconds = self._rounded_total_seconds()
        return rounded_total_seconds // 3600

    @property
    def minutes(self) -> int:
        rounded_total_seconds = self._rounded_total_seconds()
        remainder = rounded_total_seconds % 3600
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
        return cls(hours * 3600 + minutes * 60 + seconds)

    def __str__(self) -> str:
        sign = "-" if self._total_seconds < 0 else ""
        return f"{sign}{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}"


class HAPerSecond(AxisSpeed):
    def __init__(self, seconds_per_second: float):
        self._total_seconds_per_second = seconds_per_second

    @overload
    def __mul__(self, other: Second) -> HA: ...

    def __mul__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return HA(self._total_seconds_per_second * other.seconds)
        raise NotImplementedError(f"Unsupported multiplication: {type(self)} * {type(other)}")


class Dec(AxisPos):
    DEC_PATTERN = re.compile(
        rf"^([+-])(\d{{2}})\*(\d{{2}}):(\d{{2}})$"
    )
    ARCSECONDS_PER_QUATER_CIRCLE = 90 * 3600

    def __init__(self, arcseconds: float):
        if arcseconds > self.ARCSECONDS_PER_QUATER_CIRCLE:
            arcseconds %= self.ARCSECONDS_PER_QUATER_CIRCLE
        if arcseconds < -self.ARCSECONDS_PER_QUATER_CIRCLE:
            arcseconds %= -self.ARCSECONDS_PER_QUATER_CIRCLE
        self._total_arcseconds = arcseconds

    def to_raw(self) -> float:
        return self._total_arcseconds
    
    def _rounded_total_arcseconds(self) -> int:
        rounded_total_arcseconds = abs(int(round(self._total_arcseconds)))
        if rounded_total_arcseconds > self.ARCSECONDS_PER_QUATER_CIRCLE:
            rounded_total_arcseconds %= self.ARCSECONDS_PER_QUATER_CIRCLE
        return rounded_total_arcseconds

    @property
    def degrees(self) -> int:
        rounded_total_arcseconds = self._rounded_total_arcseconds()
        return rounded_total_arcseconds // 3600
    
    @property
    def arcminutes(self) -> int:
        rounded_total_arcseconds = abs(self._rounded_total_arcseconds())
        remainder = rounded_total_arcseconds % 3600
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
        total_arcseconds = sign * ((degrees * 3600) + (arcminutes * 60) + arcseconds)
        return cls(total_arcseconds)

    @overload
    def __truediv__(self, other: Second) -> DecPerSecond: ...

    def __truediv__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return DecPerSecond(self._total_arcseconds / other.seconds)
        raise NotImplementedError(f"Unsupported division: {type(self)} / {type(other)}")
    
    def __str__(self) -> str:
        sign = "-" if self._total_arcseconds < 0 else "+"
        return f"{sign}{abs(self.degrees):02d}*{self.arcminutes:02d}:{self.arcseconds:02d}"


class DecPerSecond(AxisSpeed):
    def __init__(self, arcseconds_per_second: float):
        self._total_arcseconds_per_second = arcseconds_per_second

    @overload
    def __mul__(self, other: Second) -> Dec: ...

    def __mul__(self, other: Any) -> Any:
        if isinstance(other, Second): 
            return Dec(self._total_arcseconds_per_second * other.seconds)
        raise NotImplementedError(f"Unsupported multiplication: {type(self)} * {type(other)}")
