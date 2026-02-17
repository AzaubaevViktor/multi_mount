import re
from typing import Self

SIGN_VALUES = {"+", "-"}
SIGN_CHARS = "+-"

MINUTES_PER_DEGREE = 60
SECONDS_PER_DEGREE = 60 * MINUTES_PER_DEGREE
MAX_DEC_DEGREES = 90

APOSTROPHE = ":"
UNICODE_APOSTROPHE = ":"
WHOLE_UNIT_TOLERANCE = 1e-9

HOURS_PATTERN = re.compile(r"^(\d{2}):(\d{2}):(\d{2})$")
DEC_PATTERN = re.compile(rf"^([{SIGN_CHARS}])(\d{{2}})\*(\d{{2}})[{APOSTROPHE}{UNICODE_APOSTROPHE}](\d{{2}})$")



class LX200HoursError(ValueError):
    pass


class LX200HoursFormatError(LX200HoursError):
    pass


class LX200HoursRangeError(LX200HoursError):
    pass


class LX200DecError(ValueError):
    pass


class LX200DecFormatError(LX200DecError):
    pass


class LX200DecRangeError(LX200DecError):
    pass


class LX200PositionBase:
    @classmethod
    def from_raw(cls, raw_position: float) -> Self:
        raise NotImplementedError()
    
    def to_raw(self) -> float:
        raise NotImplementedError()
    
    @classmethod
    def from_string(cls, s: str) -> Self:
        raise NotImplementedError()
    
    def __str__(self) -> str:
        raise NotImplementedError()


class LX200Ha(LX200PositionBase):
    __slots__ = ("_total_seconds",)

    SECONDS_PER_CIRCLE = 24 * 3600

    def __init__(self, hours: int, minutes: int, seconds: float) -> None:
        self._validate_parts(hours, minutes, seconds)
        self._total_seconds = (hours * 3600) + (minutes * 60) + seconds

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
        match = HOURS_PATTERN.match(s)
        if not match:
            raise LX200HoursFormatError(f"Invalid HH:MM:SS format: {s!r}")

        hours = int(match.group(1))
        minutes = int(match.group(2))
        seconds = int(match.group(3))
        return cls(hours, minutes, seconds)

    @classmethod
    def from_seconds(cls, total_seconds: float) -> Self:
        circle_seconds = cls.SECONDS_PER_CIRCLE
        remainder = total_seconds % circle_seconds
        if total_seconds > 0 and remainder == 0:
            total_seconds = circle_seconds
        else:
            total_seconds = remainder

        hours = int(total_seconds // 3600)
        remainder = total_seconds % 3600
        minutes = int(remainder // 60)
        seconds = remainder % 60
        return cls(hours, minutes, seconds)
    
    @classmethod
    def from_raw(cls, raw_position: float) -> Self:
        return cls.from_seconds(raw_position)
    
    def __sub__(self, b: LX200Ha):
        return LX200Ha.from_hours(self.to_hours() - b.to_hours())
    
    def __neg__(self):
        return LX200Ha.from_hours(-self.to_hours())

    @classmethod
    def from_hours(cls, total_hours: float) -> Self:
        total_seconds = total_hours * 3600
        return cls.from_seconds(total_seconds)

    def to_seconds(self) -> float:
        return self._total_seconds

    def to_hours(self) -> float:
        return self.to_seconds() / 3600

    def __str__(self) -> str:
        return f"{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}"

    def __repr__(self) -> str:
        return f"LX200Hours('{self}')"

    @staticmethod
    def _validate_parts(hours: int, minutes: int, seconds: float) -> None:
        if hours < 0 or hours > 23:
            raise LX200HoursRangeError(f"Hours out of range: {hours!r}")
        if minutes < 0 or minutes > 59:
            raise LX200HoursRangeError(f"Minutes out of range: {minutes!r}")
        if seconds < 0 or seconds >= 60:
            raise LX200HoursRangeError(f"Seconds out of range: {seconds!r}")

    def _rounded_total_seconds(self) -> int:
        rounded_total_seconds = int(round(self._total_seconds))
        return rounded_total_seconds % self.SECONDS_PER_CIRCLE


class LX200Dec(LX200PositionBase):
    __slots__ = ("_arcseconds",)

    def __init__(self, sign: str, degrees: int, minutes: int, arcseconds: float) -> None:
        self._validate_parts(sign, degrees, minutes, arcseconds)
        total_arcseconds = (
            (degrees * SECONDS_PER_DEGREE)
            + (minutes * MINUTES_PER_DEGREE)
            + arcseconds
        )
        if sign == "-":
            total_arcseconds = -total_arcseconds
        self._arcseconds = total_arcseconds

    @property
    def sign(self) -> str:
        return "-" if self._arcseconds < 0 else "+"

    @property
    def degrees(self) -> int:
        arcseconds = self._arcseconds
        if arcseconds < 0:
            arcseconds = -arcseconds
        return int(arcseconds // SECONDS_PER_DEGREE)

    @property
    def minutes(self) -> int:
        arcseconds = self._arcseconds
        if arcseconds < 0:
            arcseconds = -arcseconds
        remainder = arcseconds % SECONDS_PER_DEGREE
        return int(remainder // MINUTES_PER_DEGREE)

    @property
    def seconds(self) -> float:
        arcseconds = self._arcseconds
        if arcseconds < 0:
            arcseconds = -arcseconds
        remainder = arcseconds % SECONDS_PER_DEGREE
        minutes = int(remainder // MINUTES_PER_DEGREE)
        return remainder - (minutes * MINUTES_PER_DEGREE)

    @classmethod
    def from_string(cls, s: str) -> Self:
        match = DEC_PATTERN.match(s)
        if not match:
            raise LX200DecFormatError(f"Invalid sDD*MM:SS format: {s!r}")

        sign = match.group(1)
        degrees = int(match.group(2))
        minutes = int(match.group(3))
        seconds = int(match.group(4))
        return cls(sign, degrees, minutes, seconds)

    @classmethod
    def from_degrees(cls, total_degrees: float) -> Self:
        total_arcseconds = total_degrees * SECONDS_PER_DEGREE
        return cls.from_arcseconds(total_arcseconds)

    @classmethod
    def from_arcseconds(cls, total_arcseconds: float) -> Self:
        max_arcseconds = MAX_DEC_DEGREES * SECONDS_PER_DEGREE
        if abs(total_arcseconds) > max_arcseconds:
            raise LX200DecRangeError(f"Arcseconds out of range: {total_arcseconds!r}")

        sign = "-" if total_arcseconds < 0 else "+"
        abs_arcseconds = abs(total_arcseconds)
        degrees = int(abs_arcseconds // SECONDS_PER_DEGREE)
        remainder = abs_arcseconds - (degrees * SECONDS_PER_DEGREE)
        minutes = int(remainder // MINUTES_PER_DEGREE)
        arcseconds = remainder - (minutes * MINUTES_PER_DEGREE)

        if degrees == 0 and minutes == 0 and arcseconds == 0:
            sign = "+"

        return cls(sign, degrees, minutes, arcseconds)
    
    @classmethod
    def from_raw(cls, raw_position: float) -> Self:
        return cls.from_arcseconds(raw_position)

    def to_degrees(self) -> float:
        return self._arcseconds / SECONDS_PER_DEGREE

    def to_arcseconds(self) -> float:
        return self._arcseconds

    def __str__(self) -> str:
        rounded_arcseconds = int(round(self._arcseconds))
        if rounded_arcseconds < 0:
            sign = "-"
            rounded_arcseconds = -rounded_arcseconds
        else:
            sign = "+"

        degrees = rounded_arcseconds // SECONDS_PER_DEGREE
        remainder = rounded_arcseconds - (degrees * SECONDS_PER_DEGREE)
        minutes = remainder // MINUTES_PER_DEGREE
        seconds = remainder - (minutes * MINUTES_PER_DEGREE)
        if rounded_arcseconds == 0:
            sign = "+"

        return (
            f"{sign}{degrees:02d}*{minutes:02d}{APOSTROPHE}{seconds:02d}"
        )

    def __repr__(self) -> str:
        return f"LX200Dec('{self}')"

    @staticmethod
    def _validate_parts(sign: str, degrees: int, minutes: int, arcseconds: float) -> None:
        if sign not in SIGN_VALUES:
            raise LX200DecRangeError(f"Sign out of range: {sign!r}")
        if degrees < 0 or degrees > 90:
            raise LX200DecRangeError(f"Degrees out of range: {degrees!r}")
        if minutes < 0 or minutes > 59:
            raise LX200DecRangeError(f"Minutes out of range: {minutes!r}")
        if arcseconds < 0 or arcseconds >= 60:
            raise LX200DecRangeError(f"Arcseconds out of range: {arcseconds!r}")
        if degrees == 90 and (minutes != 0 or arcseconds != 0):
            raise LX200DecRangeError(f"Degrees out of range: {degrees!r}")
