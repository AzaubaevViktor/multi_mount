"""What `StepsPerSecond` refuses to do at runtime.

The type's main job is done by mypy: `StepsPerSecond[HaPerSecond]` and
`StepsPerSecond[DecPerSecond]` are unrelated types, and neither is an
`AxisSpeed` or a `TimerPeriod`. These tests hold the *second* line of defence —
the isinstance guards inherited from `_BasicAriphmetic`, which matter because
these values cross untyped boundaries (test fakes, LX200 payloads, the tools in
`src/tools/`).
"""

import pytest

from sky.physics import (
    DecPerSecond,
    DecStepsPerSecond,
    Ha,
    HaPerSecond,
    HaStepsPerSecond,
    Second,
    StepsPerSecond,
)


def test_steps_per_second_is_not_an_angular_speed():
    # An AxisSpeed times Second is a position. A step rate times seconds would be a
    # step count, which is not a quantity this project carries -- so the operator has
    # to refuse rather than invent one.
    with pytest.raises(TypeError):
        HaStepsPerSecond(100) * Second(2)  # type: ignore[operator]


def test_angular_speed_does_not_add_to_a_step_rate():
    with pytest.raises(TypeError):
        HaStepsPerSecond(100) + HaPerSecond(1)  # type: ignore[operator]


def test_step_rate_does_not_add_to_a_position():
    with pytest.raises(TypeError):
        HaStepsPerSecond(100) + Ha(1)  # type: ignore[operator]


@pytest.mark.parametrize(
    "operation",
    [
        lambda a, b: a + b,
        lambda a, b: a - b,
        lambda a, b: a < b,
        lambda a, b: a >= b,
    ],
)
def test_the_two_axes_do_not_mix(operation):
    # 4 000 steps/s on RA and 4 000 steps/s on DEC are different physical rates:
    # the two axes have different counts per revolution and different gear trains.
    with pytest.raises(TypeError):
        operation(HaStepsPerSecond(4000), DecStepsPerSecond(4000))


def test_scaling_keeps_the_axis():
    doubled = HaStepsPerSecond(1000) * 2
    assert isinstance(doubled, HaStepsPerSecond)
    assert float(doubled) == pytest.approx(2000)

    halved = HaStepsPerSecond(1000) / 2
    assert isinstance(halved, HaStepsPerSecond)
    assert float(halved) == pytest.approx(500)

    assert isinstance(-HaStepsPerSecond(1000), HaStepsPerSecond)
    assert isinstance(abs(DecStepsPerSecond(-1000)), DecStepsPerSecond)


def test_ratio_of_two_rates_on_the_same_axis_is_a_plain_number():
    ratio = HaStepsPerSecond(4650) / HaStepsPerSecond(2325)
    assert not isinstance(ratio, StepsPerSecond)
    assert ratio == pytest.approx(2.0)


def test_comparison_against_a_bare_number_stays_allowed():
    # Thresholds in the drivers are read off the board as plain integers, so
    # comparing against one is legal; comparing against another *unit* is not.
    assert HaStepsPerSecond(4650) > 4000
    assert DecStepsPerSecond(10) <= 10


def test_str_names_the_unit():
    assert str(HaStepsPerSecond(4650.4)) == "4650 steps/s"
    assert "steps/s" in repr(DecStepsPerSecond(12))


def test_concrete_axes_are_step_rates():
    assert isinstance(HaStepsPerSecond(1), StepsPerSecond)
    assert isinstance(DecStepsPerSecond(1), StepsPerSecond)
    assert not isinstance(HaPerSecond(1), StepsPerSecond)
    assert not isinstance(DecPerSecond(1), StepsPerSecond)
