from __future__ import annotations

from lx200.base import LX200Handler
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from sky.polar_compensator import PolarCompensator
from tests.base.fakes import FakeDECMotor, FakeRAMotor


def test_limit_commands_update_mount_limits() -> None:
    sky = SkyLX200(
        Combiner(
            ra_axis=AxisRA(FakeRAMotor()),
            dec_axis=AxisDEC(FakeDECMotor()),
            polar_compensator=PolarCompensator(),
        )
    )
    handler = LX200Handler(sky)

    assert handler.handle("Sh+75*00") == b"1"
    assert handler.handle("So+10*00") == b"1"
    assert sky.limits.highest_elevation.degrees == 75.0
    assert sky.limits.minimum_elevation.degrees == 10.0
