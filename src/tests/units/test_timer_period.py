"""`TimerPeriod`: the board's step period, and why it is not a number.

The period and the step rate are reciprocal, both integral and both used to be
plain ``int``. Swapping them is the mistake that once made an 800x request come
out as 71.8x (``docs/protocol/RA_PROTOCOL_STEP_2.md`` §11), and it is invisible
in a diff. mypy now rejects the swap outright; what is checked here is the
runtime half — that a period refuses to behave like the number it wraps, so an
untyped boundary (a test fake, a tool, a JSON row) cannot let one through by
accident.
"""

import pytest

from sky.constants import STELLAR_SPEED
from sky.physics import HaStepsPerSecond
from skywatcher.board import SkyWatcherBoard, TimerPeriod
from skywatcher.codec import SpeedMode

# The live RA controller: `:a1`, `:b1`, `:g1` as read off the board.
_BOARD = SkyWatcherBoard(cpr=12_489_074, timer_freq=15_400_960, highspeed_ratio=1, min_period=TimerPeriod(1103))


def test_a_period_is_not_equal_to_its_own_count():
    # The point of the whole exercise: `period == 1103` is a question about two
    # different kinds of thing, and the answer has to be "no", not "yes by luck".
    # The `type: ignore` is the static half of the same statement: mypy already
    # calls this comparison non-overlapping, which is the guarantee being asserted.
    assert TimerPeriod(1103) != 1103  # type: ignore[comparison-overlap]
    assert TimerPeriod(1103) == TimerPeriod(1103)


def test_a_period_does_not_order_against_a_bare_number():
    with pytest.raises(TypeError):
        assert TimerPeriod(1103) < 2000  # type: ignore[operator]


def test_a_period_does_not_do_arithmetic():
    with pytest.raises(TypeError):
        TimerPeriod(1103) + 1  # type: ignore[operator]


def test_a_period_is_not_a_step_rate():
    assert TimerPeriod(1103) != HaStepsPerSecond(1103)
    with pytest.raises(TypeError):
        assert TimerPeriod(1103) < HaStepsPerSecond(1103)  # type: ignore[operator]


def test_the_count_and_the_text_are_the_raw_number():
    # Both are load-bearing: `int()` is what reaches `encode_revu24` and the JSON
    # manifests, `str()` is what reaches the log lines.
    assert int(TimerPeriod(1103)) == 1103
    assert str(TimerPeriod(1103)) == "1103"


def test_periods_order_among_themselves_so_the_clamp_works():
    assert TimerPeriod(1103) < TimerPeriod(2000)
    assert _BOARD.clamp_period(TimerPeriod(6)) == TimerPeriod(1103)
    assert _BOARD.clamp_period(TimerPeriod(5000)) == TimerPeriod(5000)


def test_the_board_is_the_only_bridge_between_a_period_and_a_rate():
    # 32x sidereal: above the board's period clamp and below its rate ceiling, so
    # nothing is cut on the way there or back.
    rate = HaStepsPerSecond(round(float(STELLAR_SPEED) * 32 * _BOARD.cpr / (24 * 60 * 60)))

    period = _BOARD.period_from_speed_sps(rate, SpeedMode.LOWSPEED)
    assert isinstance(period, TimerPeriod)
    assert _BOARD.speed_sps_from_period(period, SpeedMode.LOWSPEED) == pytest.approx(float(rate), rel=1e-3)
    # And the bridge is not the identity: the period is nothing like the rate.
    assert int(period) != int(rate)
