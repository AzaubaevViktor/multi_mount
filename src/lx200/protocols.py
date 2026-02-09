import re

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


class LX200Ha:
    __slots__ = ("_hours", "_minutes", "_seconds")

    SECONDS_PER_CIRCLE = 24 * 3600

    def __init__(self, hours: int, minutes: int, seconds: float) -> None:
        self._validate_parts(hours, minutes, seconds)
        self._hours = hours
        self._minutes = minutes
        self._seconds = seconds

    @property
    def hours(self) -> int:
        return self._hours

    @property
    def minutes(self) -> int:
        return self._minutes

    @property
    def seconds(self) -> float:
        return self._seconds

    @classmethod
    def from_string(cls, value: str) -> "LX200Ha":
        match = HOURS_PATTERN.match(value)
        if not match:
            raise LX200HoursFormatError(f"Invalid HH:MM:SS format: {value!r}")

        hours = int(match.group(1))
        minutes = int(match.group(2))
        seconds = int(match.group(3))
        return cls(hours, minutes, seconds)

    @classmethod
    def from_seconds(cls, total_seconds: float) -> "LX200Ha":
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
    
    def __sub__(self, b: LX200Ha):
        return LX200Ha.from_hours(self.to_hours() - b.to_hours())
    
    def __neg__(self):
        return LX200Ha.from_hours(-self.to_hours())

    @classmethod
    def from_hours(cls, total_hours: float) -> "LX200Ha":
        total_seconds = total_hours * 3600
        return cls.from_seconds(total_seconds)

    def to_seconds(self) -> float:
        return (self._hours * 3600) + (self._minutes * 60) + self._seconds

    def to_hours(self) -> float:
        return self.to_seconds() / 3600

    def __str__(self) -> str:
        return f"{self._hours:02d}:{self._minutes:02d}:{self._seconds:02.0f}"

    def __repr__(self) -> str:
        return f"LX200Hours('{self}')"

    @staticmethod
    def _validate_parts(hours: int, minutes: int, seconds: float) -> None:
        if hours < 0 or hours > 23:
            raise LX200HoursRangeError(f"Hours out of range: {hours!r}")
        if minutes < 0 or minutes > 59:
            raise LX200HoursRangeError(f"Minutes out of range: {minutes!r}")
        if seconds < 0 or seconds > 59:
            raise LX200HoursRangeError(f"Seconds out of range: {seconds!r}")


class LX200Dec:
    __slots__ = ("_sign", "_degrees", "_minutes", "_seconds")

    def __init__(self, sign: str, degrees: int, minutes: int, seconds: float) -> None:
        self._validate_parts(sign, degrees, minutes, seconds)
        self._sign = sign
        self._degrees = degrees
        self._minutes = minutes
        self._seconds = seconds

    @property
    def sign(self) -> str:
        return self._sign

    @property
    def degrees(self) -> int:
        return self._degrees

    @property
    def minutes(self) -> int:
        return self._minutes

    @property
    def seconds(self) -> float:
        return self._seconds

    @classmethod
    def from_string(cls, value: str) -> "LX200Dec":
        match = DEC_PATTERN.match(value)
        if not match:
            raise LX200DecFormatError(f"Invalid sDD*MM:SS format: {value!r}")

        sign = match.group(1)
        degrees = int(match.group(2))
        minutes = int(match.group(3))
        seconds = int(match.group(4))
        return cls(sign, degrees, minutes, seconds)

    @classmethod
    def from_degrees(cls, total_degrees: float) -> "LX200Dec":
        sign = "-" if total_degrees < 0 else "+"
        abs_degrees = abs(total_degrees)
        total_arcseconds = abs_degrees * 3600

        degrees = int(total_arcseconds // 3600)
        remainder = total_arcseconds % 3600
        minutes = int(remainder // 60)
        seconds = remainder % 60

        if degrees > 90:
            raise LX200DecRangeError(f"Degrees out of range: {total_degrees!r}")
        if degrees == 90 and (minutes != 0 or seconds != 0):
            raise LX200DecRangeError(f"Degrees out of range: {total_degrees!r}")

        if degrees == 0 and minutes == 0 and seconds == 0:
            sign = "+"

        return cls(sign, degrees, minutes, seconds)

    def to_degrees(self) -> float:
        total = self._degrees + (self._minutes / 60) + (self._seconds / 3600)
        if self._sign == "-":
            return -total
        return total

    def __str__(self) -> str:
        return f"{self._sign}{self._degrees:02d}*{self._minutes:02d}{APOSTROPHE}{self._seconds:02d}"

    def __repr__(self) -> str:
        return f"LX200Dec('{self}')"

    @staticmethod
    def _validate_parts(sign: str, degrees: int, minutes: int, seconds: float) -> None:
        if sign not in SIGN_VALUES:
            raise LX200DecRangeError(f"Sign out of range: {sign!r}")
        if degrees < 0 or degrees > 90:
            raise LX200DecRangeError(f"Degrees out of range: {degrees!r}")
        if minutes < 0 or minutes > 59:
            raise LX200DecRangeError(f"Minutes out of range: {minutes!r}")
        if seconds < 0 or seconds > 59:
            raise LX200DecRangeError(f"Seconds out of range: {seconds!r}")
        if degrees == 90 and (minutes != 0 or seconds != 0):
            raise LX200DecRangeError(f"Degrees out of range: {degrees!r}")
