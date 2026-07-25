"""Pure protocol layer of the SkyWatcher RA board: bytes in, values out.

Nothing here does I/O, keeps state or knows what time it is, so every rule of
``docs/protocol/RA_PROTOCOL.md`` §7, §8 and §10.1 can be checked by calling a
function with a string. The three layers above are
:class:`skywatcher.board.SkyWatcherBoard` (what this particular board is),
:class:`skywatcher.session.SkyWatcherSession` (state and firmware quirks) and
:class:`skywatcher.motor.SkyWatcherMotor` (what the axis sees).

The frame is the same for every command the board understands:
``:<cmd><channel><data>\\r`` answered with ``=<data>\\r`` or ``!<code>\\r``.
There is no second framing — §8: only ``\\r`` terminates anything — so there is
no second branch anywhere in this module either.
"""

import dataclasses
from enum import IntEnum, StrEnum

from skywatcher.protocol import Protocol


class SkyWatcherMotorError(Exception):
    def __init__(self, message: str) -> None:
        self.message = message
        super().__init__(message)


class SkyWatcherMotorProtocolError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorCommandError(SkyWatcherMotorError):
    pass


class SkyWatcherMotorTimeoutError(SkyWatcherMotorError, TimeoutError):
    pass


class SkyWatcherMotorRebootError(SkyWatcherMotorError):
    """The board rebooted under the driver's feet (§12.2) and nothing was started.

    Raised only where the alternative would be to act on a board whose state has
    just been rebuilt: a motion command whose acknowledgement would otherwise be
    reported as "the axis is moving".
    """


class Command(StrEnum):
    INQUIRE_TIMER_FREQ = "b"
    INQUIRE_CPR = "a"
    INQUIRE_MOTOR_BOARD_VERSION = "e"
    INQUIRE_POSITION = "j"
    INQUIRE_STATUS = "f"
    INQUIRE_HIGHSPEED_RATIO = "g"
    INQUIRE_STEP_PERIOD = "i"
    INQUIRE_TRACKING_PERIOD = "D"
    INQUIRE_EXTENDED = "q"
    SET_STEP_PERIOD = "I"
    SET_MEMORY_ADDRESS = "C"
    INQUIRE_MEMORY_BYTE = "n"
    SET_GOTO_TARGET_INCREMENT = "H"
    SET_BREAK_POINT_INCREMENT = "M"
    SET_AXIS_POSITION = "E"
    SET_MOTION_MODE = "G"
    START_MOTION = "J"
    STOP_MOTION = "K"
    INITIALIZE = "F"


class Axis(StrEnum):
    RA = "1"


class Direction(IntEnum):
    BACKWARD = 0
    FORWARD = 1


class SlewMode(IntEnum):
    SLEW = 0
    GOTO = 1


class SpeedMode(IntEnum):
    LOWSPEED = 0
    HIGHSPEED = 1


@dataclasses.dataclass(frozen=True)
class Status:
    """Decoded ``:f1`` answer (§10.1, confirmed on the live board)."""

    raw: int
    running: bool
    initialized: bool
    slew_mode: SlewMode
    direction: Direction
    speed_mode: SpeedMode

    @classmethod
    def from_bytes(cls, data: bytes) -> "Status":
        # A short body is a damaged frame, not a status with the missing flags cleared:
        # zero-padding `=41` reports "stopped, not initialized" for a slewing axis, and
        # `wait_till_stop` would then return while the mount keeps moving.
        if len(data) != 3:
            raise SkyWatcherMotorProtocolError(f"expected 3 status chars, got {data!r}")
        b1, b2, b3 = data
        raw = b2 | (b1 << 8) | (b3 << 16)
        return cls(
            raw=raw,
            running=bool(b2 & 0x01),
            initialized=bool(b3 & 0x01),
            slew_mode=SlewMode.SLEW if (b1 & 0x01) else SlewMode.GOTO,
            direction=Direction.BACKWARD if (b1 & 0x02) else Direction.FORWARD,
            speed_mode=SpeedMode.HIGHSPEED if (b1 & 0x04) else SpeedMode.LOWSPEED,
        )


@dataclasses.dataclass(frozen=True)
class MotionStatus:
    """The three things ``:G1<mode><dir>`` carries, before they become two chars."""

    slew_mode: SlewMode
    direction: Direction
    speed_mode: SpeedMode

    def to_command(self) -> str:
        if self.slew_mode == SlewMode.SLEW:
            motion_mode = "1" if self.speed_mode == SpeedMode.LOWSPEED else "3"
        else:
            motion_mode = "2" if self.speed_mode == SpeedMode.LOWSPEED else "0"
        direction_mode = "0" if self.direction == Direction.FORWARD else "1"
        return f"{motion_mode}{direction_mode}"


class SkyWatcherCodec:
    """Every conversion between the wire and a Python value, and nothing else."""

    RESPONSE_PREFIXES = (Protocol.RESPONSE_PREFIX_BYTE, Protocol.COMMAND_ERROR_PREFIX_BYTE)

    @staticmethod
    def frame(command: Command, arg: str | None = None, channel: str = Axis.RA) -> str:
        return f"{Protocol.COMMAND_PREFIX}{command.value}{channel}{arg or ''}{Protocol.COMMAND_TERMINATOR}"

    @staticmethod
    def parse_reply(response: str | None) -> str:
        """Strip ``=<data>\\r`` down to ``data``; raise on anything else.

        ``!<code>`` is a :class:`SkyWatcherMotorCommandError` — the board
        understood the frame and refused it — while a truncated, empty or
        unprefixed answer is a :class:`SkyWatcherMotorProtocolError`, which is
        what the retry loop above reacts to.
        """
        if not response:
            raise SkyWatcherMotorProtocolError(f"empty response: {response!r}")
        if not response.endswith(Protocol.ANSWER_END):
            raise SkyWatcherMotorProtocolError(f"unterminated response: {response!r}")
        if response[0] == Protocol.COMMAND_ERROR_PREFIX:
            raise SkyWatcherMotorCommandError(f"command error: {response!r}")
        if response[0] != Protocol.RESPONSE_PREFIX:
            raise SkyWatcherMotorProtocolError(f"invalid response: {response!r}")
        return response[1:-len(Protocol.ANSWER_END)]

    @staticmethod
    def decode_revu24(data: str, hex_chars: int = 6) -> int:
        # `hex_chars` is the width the queried value really has (reference §4.5: 24-bit ->
        # 6 hex chars, 8-bit -> 2), and only that narrower answer may be zero-extended.
        # Doing it for every command instead — as this did — turns a damaged frame into a
        # plausible number: a position answer that lost four of its six digits would
        # decode as a valid position on the other side of the axis.
        if len(data) == hex_chars < 6:
            data = f"{data}{'0' * (6 - hex_chars)}"
        if len(data) != 6:
            raise SkyWatcherMotorProtocolError(f"expected {hex_chars} hex chars, got {data!r}")
        reordered = data[4:6] + data[2:4] + data[0:2]
        try:
            return int(reordered, 16)
        except ValueError as exc:
            raise SkyWatcherMotorProtocolError(f"invalid hex data: {data!r}") from exc

    @staticmethod
    def encode_revu24(value: int) -> str:
        if value < 0 or value > 0xFFFFFF:
            raise SkyWatcherMotorProtocolError(f"expected value in range 0..{0xFFFFFF}, got {value}")
        return value.to_bytes(3, "little").hex().upper()

    @staticmethod
    def parse_status(body: str) -> Status:
        return Status.from_bytes(body.encode("ascii"))

    @staticmethod
    def decode_board_version(body: str) -> tuple[int, int]:
        """``:e1`` -> (firmware version, mount code).

        The answer is Revu24 like everything else, and then the three bytes have
        to be turned back around: `=03110A` is firmware 0x0311 with mount code
        0x0A (§2), not the 0x0A1103 a straight decode gives.
        """
        value = SkyWatcherCodec.decode_revu24(body)
        value = ((value & 0xFF) << 16) | (value & 0xFF00) | ((value & 0xFF0000) >> 16)
        return value >> 8, value & 0xFF

    @staticmethod
    def encode_memory_address(address: int) -> str:
        """Argument of ``:C1`` — 16-bit, low byte first (§6.8).

        `:C11C00` is address 0x001C, not 0x1C00. Getting this backwards reads a
        different cell and still answers a plausible byte.
        """
        if address < 0 or address > 0xFFFF:
            raise SkyWatcherMotorProtocolError(f"expected address in range 0..{0xFFFF}, got {address}")
        return f"{address & 0xFF:02X}{(address >> 8) & 0xFF:02X}"

    @staticmethod
    def decode_memory_byte(body: str) -> int:
        """``:n1`` -> one byte, two hex chars, nothing else accepted."""
        if len(body) != 2 or any(char not in "0123456789abcdefABCDEF" for char in body):
            raise SkyWatcherMotorProtocolError(f"expected 2 hex chars from the memory window, got {body!r}")
        return int(body, 16)
