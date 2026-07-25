"""What the board on the other end of the wire actually is, asked once.

:class:`SkyWatcherBoard` is a frozen snapshot taken during the handshake plus
the arithmetic that follows from it. There is no table of magic constants keyed
by mount code anywhere in this module — every number is either an answer of the
board or a value derived from those answers, which is the whole point: the
driver used to guess the minimum step period from the mount code (`0x0A ->
0x0600`, a number with no source in the specification, in the EQMod reference
or in the vendor's own app) and was wrong in both directions (§10.5, §13 #5/#6).
"""

import dataclasses
import logging
from typing import Callable

from sky.constants import STELLAR_DAY, STELLAR_SPEED
from sky.motor import MotorStateError
from sky.physics import HaPerSecond, HaStepsPerSecond, StepsPerSecond
from skywatcher.codec import (
    Command,
    SkyWatcherCodec,
    SkyWatcherMotorError,
    SkyWatcherMotorProtocolError,
    SpeedMode,
)


@dataclasses.dataclass(frozen=True, order=True, slots=True)
class TimerPeriod:
    """How many ticks of the board's own timer one motor step lasts (`:I1`/`:i1`).

    The conjugate of :class:`sky.physics.StepsPerSecond`, and the reason that
    class exists: the two are reciprocal, both integral and both, until now,
    plain ``int``. Confusing them is not a small error — it is the difference
    between 800x sidereal and 71.8x, which is what the driver actually asked for
    once (`RA_PROTOCOL_STEP_2.md` §11).

    Deliberately *not* a number:

    * no arithmetic. A period is only ever compared (``clamp_period``), written
      to the board or divided into the timer frequency, and every one of those
      needs ``int(period)`` spelled out, which is where a reader can see what is
      happening;
    * no ``__index__``. It would let a period slide into ``encode_revu24`` and
      every other ``int`` slot unnoticed, which is exactly the class of mistake
      this type is here to stop;
    * no conversion methods. Turning a period into a step rate needs the board's
      timer frequency and counts per revolution, so it belongs to
      :class:`SkyWatcherBoard` and nowhere else.

    Not parameterised by axis, unlike ``StepsPerSecond``: only the SkyWatcher RA
    controller has a step-period register at all. The DEC controller is told a
    rate directly, and if it ever grows a timer of its own, its period will be
    counted in a different timer's ticks and will want its own type anyway.
    """

    ticks: int

    def __int__(self) -> int:
        return self.ticks

    def __str__(self) -> str:
        return str(self.ticks)


# Fallback for a board whose own clamp could not be measured: the value the INDI
# reference uses for every mount it does not recognize. It clamps nothing the
# board does not clamp itself, so the axis still runs — only the speed reported
# upwards may be optimistic, which is what the log says when this is used.
DEFAULT_MIN_PERIOD = TimerPeriod(6)

# Written during the probe: below every plausible clamp, and harmless — the board
# answers `=` and stores its own minimum instead, even for 0 (§10.5).
MIN_PERIOD_PROBE = TimerPeriod(1)

# §5/§2: Status EX. One read, kept in the snapshot because it is a board
# capability word and belongs with the rest of them.
STATUS_EX_ID = "010000"

# The rate this controller actually manages, whatever period is loaded
# (`RA_PROTOCOL_STEP_2.md` §2.6). The period clamp above says 1103, which the
# formula reads as 14 506 steps/s; the axis, sampled every 50 ms through a
# five-second run, settles at 9 015 — the same rate it does at period 1724
# (8 847). So the clamp is not the limit, and reporting `timer_freq / period`
# above 9 000 tells `Axis` a GOTO will take 60% less time than it will.
#
# Applied to what is *reported*, never to what is written: the period stays the
# one the board was asked for, because the axis does go as fast as it can and
# nothing is gained by asking for less.
BOARD_SPEED_CEILING_SPS = HaStepsPerSecond(9_000)

Transactor = Callable[[Command, str | None], str]


@dataclasses.dataclass(frozen=True)
class SkyWatcherBoard:
    """Capabilities of one controller: constants and the maths they imply.

    ``cpr`` (`:a1`), ``timer_freq`` (`:b1`) and ``highspeed_ratio`` (`:g1`) are
    the three numbers every speed conversion needs; the object refuses to exist
    without all three, so no caller can divide by a half-read handshake.
    """

    cpr: int
    timer_freq: int
    highspeed_ratio: int
    min_period: TimerPeriod = DEFAULT_MIN_PERIOD
    firmware_version: int = 0
    mount_code: int = 0
    # `:D1`, the 1x tracking period the board powers up with (§12.1). None means
    # the board would not say, and :attr:`power_up_period` computes it instead.
    tracking_period_1x: TimerPeriod | None = None
    status_ex: int | None = None
    # `:c1`, the braking distance the board hard-wires into every GOTO. Not a
    # setting: `:M1` is accepted at any value and changes nothing, and `:m1` is
    # always "target - this" (`RA_PROTOCOL_STEP_2.md` §2.2, §11). The driver
    # needs it because it is the length below which a GOTO's own brake point
    # falls behind its start — the degenerate geometry the safe GOTO refuses to
    # hand the board. 16 980 on this controller; the default is the same number
    # only so that a board which will not answer `:c1` still has a sane bound.
    brake_steps: int = 16_980

    def __post_init__(self) -> None:
        if self.cpr <= 0 or self.timer_freq <= 0 or self.highspeed_ratio <= 0:
            raise SkyWatcherMotorProtocolError(
                f"invalid mount config: cpr={self.cpr} timer_freq={self.timer_freq} "
                f"highspeed_ratio={self.highspeed_ratio}"
            )

    @property
    def power_up_period(self) -> TimerPeriod:
        """Step period a freshly booted board reports (§12.1: `:i1` = `:D1`).

        One of the three signs of a reboot, so it may not be a guess when the
        board answers `:D1` — and when it does not, the computed 1x tracking
        period reproduces the board's own answer to within a count.
        """
        if self.tracking_period_1x is not None:
            return self.tracking_period_1x
        return TimerPeriod(max(1, round(float(STELLAR_DAY) * self.timer_freq / self.cpr)))

    def times_sidereal(self, period: TimerPeriod) -> float:
        return float(STELLAR_DAY) * self.timer_freq / self.cpr / int(period)

    def clamp_period(self, period: TimerPeriod) -> TimerPeriod:
        """What the board will store for a requested period.

        The board takes `:I1` with any value and keeps `max(value, its own
        minimum)`. Doing the same here is not a restriction the driver invents:
        it is the only way the speed reported upwards can be the speed the axis
        will really run at (§2.2).
        """
        return max(period, self.min_period)

    def period_from_speed_sps(self, speed_sps: StepsPerSecond[HaPerSecond], speed_mode: SpeedMode) -> TimerPeriod:
        """Timer period to load for a step rate. The inverse of :meth:`speed_sps_from_period`.

        The two are reciprocal, both integral and both "just a number", which is
        exactly why they are different types: passing a period here, or the
        answer of this method where a rate is expected, is the mix-up that
        turned an 800x request into 71.8x once already (§11).
        """
        if speed_sps <= 0:
            raise MotorStateError("speed must be positive")
        rate = float(speed_sps) * (24 * 60 * 60) / self.cpr / float(STELLAR_SPEED)
        if speed_mode == SpeedMode.HIGHSPEED:
            rate /= self.highspeed_ratio
        return TimerPeriod(int(STELLAR_DAY * self.timer_freq / self.cpr / rate))

    def speed_sps_from_period(self, period: TimerPeriod, speed_mode: SpeedMode) -> HaStepsPerSecond:
        """Steps per second the axis will really run at with this period."""
        if int(period) <= 0:
            raise MotorStateError("period must be positive")
        rate = self.times_sidereal(period)
        if speed_mode == SpeedMode.HIGHSPEED:
            rate *= self.highspeed_ratio
        asked = HaStepsPerSecond(round(rate * float(STELLAR_SPEED) * self.cpr / (24 * 60 * 60)))
        return min(asked, BOARD_SPEED_CEILING_SPS)


def probe_board(transact: Transactor, logger: logging.Logger) -> tuple[SkyWatcherBoard, TimerPeriod | None]:
    """Read the snapshot off a board that has already answered the handshake.

    Returns the snapshot and the step period the board is left holding — the
    probe is the only thing that knows it, and the session needs it as the first
    entry of its expected state (§12.4.4).
    """
    firmware_version, mount_code = SkyWatcherCodec.decode_board_version(
        transact(Command.INQUIRE_MOTOR_BOARD_VERSION, None)
    )
    cpr = SkyWatcherCodec.decode_revu24(transact(Command.INQUIRE_CPR, None))
    timer_freq = SkyWatcherCodec.decode_revu24(transact(Command.INQUIRE_TIMER_FREQ, None))
    # §2.1: `:g1` answers 2 hex chars, not 6. Zero-extending anything wider
    # would turn a damaged frame into a plausible ratio.
    highspeed_ratio = SkyWatcherCodec.decode_revu24(transact(Command.INQUIRE_HIGHSPEED_RATIO, None), hex_chars=2)
    # Built as soon as the three numbers every conversion needs are in: a board
    # that answered nonsense to any of them is rejected here, before the driver
    # spends further commands on a controller it will not be able to drive.
    board = SkyWatcherBoard(
        cpr=cpr,
        timer_freq=timer_freq,
        highspeed_ratio=highspeed_ratio,
        firmware_version=firmware_version,
        mount_code=mount_code,
    )
    board = dataclasses.replace(
        board,
        tracking_period_1x=_optional_period(transact, Command.INQUIRE_TRACKING_PERIOD, logger),
        brake_steps=_optional_value(transact, Command.INQUIRE_BRAKE_STEPS, None, logger) or board.brake_steps,
        status_ex=_optional_value(transact, Command.INQUIRE_EXTENDED, STATUS_EX_ID, logger),
    )
    min_period, loaded_period = _measure_min_period(transact, board, logger)
    return dataclasses.replace(board, min_period=min_period), loaded_period


def _optional_value(transact: Transactor, command: Command, arg: str | None, logger: logging.Logger) -> int | None:
    """A board constant the driver can live without.

    `:D1` and `:q1010000` were both read off the live board (§2, §5.1), but
    neither is worth failing a connect over: the tracking period has a computed
    fallback and Status EX is reported, not acted upon.
    """
    try:
        return SkyWatcherCodec.decode_revu24(transact(command, arg))
    except SkyWatcherMotorError as error:
        logger.info("Board did not answer %s, leaving it out of the snapshot: %s", command.name, error)
        return None


def _optional_period(transact: Transactor, command: Command, logger: logging.Logger) -> TimerPeriod | None:
    ticks = _optional_value(transact, command, None, logger)
    return None if ticks is None else TimerPeriod(ticks)


def _measure_min_period(
    transact: Transactor, board: SkyWatcherBoard, logger: logging.Logger
) -> tuple[TimerPeriod, TimerPeriod | None]:
    """Ask the board itself how fast it is willing to go (§10.5).

    The board takes `:I1` with any value, answers `=`, and stores
    `max(value, its own minimum)` — 1103 on this controller, i.e. exactly 100x
    sidereal, a limit neither the specification nor the INDI reference mentions.
    Four commands once per connect replace a guess with a fact; the axis must be
    stopped, which is what the probe checks first.

    Returns the measured clamp and the period the board is left holding.
    """
    if SkyWatcherCodec.parse_status(transact(Command.INQUIRE_STATUS, None)).running:
        logger.warning(
            "Axis is running at connect, cannot probe the board's minimum step period; "
            "falling back to %s, so reported speeds may be optimistic",
            DEFAULT_MIN_PERIOD,
        )
        return DEFAULT_MIN_PERIOD, None
    try:
        saved_period = TimerPeriod(SkyWatcherCodec.decode_revu24(transact(Command.INQUIRE_STEP_PERIOD, None)))
    except SkyWatcherMotorError as error:
        logger.warning("Could not read the step period back, falling back to %s: %s", DEFAULT_MIN_PERIOD, error)
        return DEFAULT_MIN_PERIOD, None

    loaded_period: TimerPeriod | None = None
    try:
        transact(Command.SET_STEP_PERIOD, SkyWatcherCodec.encode_revu24(int(MIN_PERIOD_PROBE)))
        measured = TimerPeriod(SkyWatcherCodec.decode_revu24(transact(Command.INQUIRE_STEP_PERIOD, None)))
    except SkyWatcherMotorError as error:
        logger.warning("Could not probe the board's minimum step period, falling back to %s: %s", DEFAULT_MIN_PERIOD, error)
        measured = DEFAULT_MIN_PERIOD
    finally:
        # The probe leaves the fastest period the board has loaded. Whatever happened
        # above, put back what was there: a `:J1` from anywhere else (a leftover GOTO,
        # a manual session) would otherwise start the axis at the board's top speed
        # instead of at the tracking period the board powers up with (§12.1).
        try:
            transact(Command.SET_STEP_PERIOD, SkyWatcherCodec.encode_revu24(int(saved_period)))
            loaded_period = saved_period
        except SkyWatcherMotorError as error:
            logger.warning("Could not restore the step period %s after the probe: %s", saved_period, error)

    min_period = max(measured, DEFAULT_MIN_PERIOD)
    logger.info("Board clamps the step period at %s (%.1fx sidereal at most)", min_period, board.times_sidereal(min_period))
    return min_period, loaded_period
