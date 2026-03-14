import stdout_dashboard as stdout_dashboard_module
from sky.axis import AxisMotionMode, PointCoordinates
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second
from lx200.base import LX200Handler
from stdout_dashboard import StdoutDashboard


class _StubLX200(LX200Handler):
    def get_telescope_ra(self) -> Ha:
        return Ha(0)

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        return True

    def get_telescope_dec(self) -> Dec:
        return Dec(0)

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        return True

    def move_east(self) -> bool:
        return True

    def move_north(self) -> bool:
        return True

    def move_south(self) -> bool:
        return True

    def move_west(self) -> bool:
        return True

    def halt_all(self) -> bool:
        return True

    def halt_east(self) -> bool:
        return True

    def halt_north(self) -> bool:
        return True

    def halt_south(self) -> bool:
        return True

    def halt_west(self) -> bool:
        return True

    def guide_east(self, ms: int) -> None:
        return None

    def guide_north(self, ms: int) -> None:
        return None

    def guide_south(self, ms: int) -> None:
        return None

    def guide_west(self, ms: int) -> None:
        return None


class _StubMotor:
    def __init__(self, status: MotorStatus, position: Ha | Dec) -> None:
        self._status = status
        self._position = position

    def status(self) -> MotorStatus:
        return self._status

    def convert_steps_to_position(self, steps: int) -> Ha | Dec:
        return self._position


class _StubAxis:
    def __init__(
        self,
        mode: AxisMotionMode,
        motor: _StubMotor,
        sky_speed: HaPerSecond | DecPerSecond,
        queue_size: int,
        processed: list[tuple[Second, str]],
    ) -> None:
        self._mode = mode
        self._motor = motor
        self._sky_speed = sky_speed
        self._queue_size = queue_size
        self._processed = processed

    def mode(self) -> AxisMotionMode:
        return self._mode

    def is_moving_to(self) -> bool:
        return False

    def command_monitor(self) -> dict[str, object]:
        return {
            "queue_size": self._queue_size,
            "processed": list(self._processed),
        }


class _StubPolarCompensator:
    STABLE_GUIDE_PULSES_COUNT = 5
    DROP_GUIDE_PULSES_COUNT_AFTER = Second(20)

    def __init__(self) -> None:
        self.eps_E = None
        self.eps_N = None
        self.ra_speed = STELLAR_SPEED * 1.25
        self.dec_speed = DecPerSecond(0.5)
        self.last_guide_pulse = Second(10)
        self.stable_guide_ra_pulses_count = 2
        self.stable_guide_dec_pulses_count = 3


class _StubCombiner:
    def __init__(self) -> None:
        now = Second(90)
        self.ra = _StubAxis(
            AxisMotionMode.TRACK,
            _StubMotor(
                MotorStatus(
                    is_connected=True,
                    steps=123456,
                    motion_mode=MotionMode.RUN,
                    speed_sps=321,
                    accel_sps=None,
                    direction=MotorDirection.FORWARD,
                    target=None,
                    microsteps=None,
                ),
                Ha(3600),
            ),
            HaPerSecond(1.0),
            0,
            [(now, "change_speed east 1.0")],
        )
        self.dec = _StubAxis(
            AxisMotionMode.TRACK,
            _StubMotor(
                MotorStatus(
                    is_connected=True,
                    steps=654321,
                    motion_mode=MotionMode.IDLE,
                    speed_sps=0,
                    accel_sps=None,
                    direction=MotorDirection.STOP,
                    target=None,
                    microsteps=None,
                ),
                Dec(1800),
            ),
            DecPerSecond(0.0),
            1,
            [(now, "halt_all")],
        )
        self._polar_compensator = _StubPolarCompensator()
        self._mount_position = PointCoordinates(ra=Ha(3600), dec=Dec(1800))

    def get_position(self) -> PointCoordinates:
        return self._mount_position


def test_lx200_monitor_filters_gr_gd_and_tracks_guide() -> None:
    lx200 = _StubLX200()

    lx200.handle("GR")
    lx200.handle("GD")
    lx200.handle("MS")
    lx200.handle("Mgw1000")

    snapshot = lx200.command_monitor()

    assert [command for _at, command in snapshot["recent"]] == ["MS"]
    assert snapshot["guide"] is not None
    assert snapshot["guide"][1] == "Mgw1000"


def test_stdout_dashboard_render_fits_20x130(monkeypatch) -> None:
    lx200 = _StubLX200()
    lx200.handle("MS")
    lx200.handle("Mge0500")

    combiner = _StubCombiner()
    dashboard = StdoutDashboard(combiner, lx200, refresh_s=0.01)

    monkeypatch.setattr(stdout_dashboard_module.time, "strftime", lambda _fmt: "12:34:56")
    monkeypatch.setattr(stdout_dashboard_module.time, "monotonic", lambda: 100.0)
    dashboard.render(clear_screen=False)

    combiner._mount_position = PointCoordinates(ra=Ha(3610), dec=Dec(1820))
    combiner.ra._motor._position = Ha(3605)
    combiner.dec._motor._position = Dec(1815)

    monkeypatch.setattr(stdout_dashboard_module.time, "monotonic", lambda: 101.0)
    frame = dashboard.render(clear_screen=False)
    lines = frame.rstrip("\n").splitlines()

    assert len(lines) <= 20
    assert all(len(line) <= 130 for line in lines)
    assert lines[0].startswith("RA")
    assert any("-- AXIS " in line for line in lines)
    assert any("-- MOTOR " in line for line in lines)
    assert any("-- POLAR " in line for line in lines)
    assert any("-- LX200 " in line for line in lines)
    assert any("polar disabled" in line for line in lines)
    assert any("mount +10.000hs" in line for line in lines)
    assert any("mount +20.00as" in line for line in lines)
    assert any("rate  +5.000hs" in line for line in lines)
    assert any("rate  +15.00as" in line for line in lines)
    assert any("guide +1.250x sid" in line for line in lines)
    assert any("guide   +0.50as" in line for line in lines)
    assert any("guide" in line for line in lines)
    assert all("GR" not in line and "GD" not in line for line in lines)
