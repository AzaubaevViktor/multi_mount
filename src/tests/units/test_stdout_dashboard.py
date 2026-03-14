import os

import stdout_dashboard as stdout_dashboard_module
from sky.axis import AxisMotionMode, PointCoordinates
from sky.constants import STELLAR_SPEED
from sky.motor import MotionMode, MotorDirection, MotorStatus
from sky.physics import Dec, DecPerSecond, Ha, HaPerSecond, Second, SkyDirection
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
    def __init__(self, status: MotorStatus, position: Ha | Dec, power_v: float | None = None) -> None:
        self._status = status
        self._position = position
        self._power_v = power_v

    def status(self) -> MotorStatus:
        return self._status

    def get_power_v(self) -> float | None:
        return self._power_v

    def convert_steps_to_position(self, steps: int) -> Ha | Dec:
        return self._position

    def get_speed_by_speed_sps(self, speed_sps: int) -> HaPerSecond | DecPerSecond:
        if isinstance(self._position, Ha):
            return HaPerSecond(float(speed_sps))
        return DecPerSecond(float(speed_sps))


class _StubAxis:
    def __init__(
        self,
        mode: AxisMotionMode,
        motor: _StubMotor,
        position: Ha | Dec,
        sky_speed: HaPerSecond | DecPerSecond,
        queue_size: int,
        processed: list[tuple[Second, str]],
        move_direction: SkyDirection | None = None,
        goto_target: Ha | Dec | None = None,
        goto_direction: SkyDirection | None = None,
    ) -> None:
        self._mode = mode
        self._motor = motor
        self._position = position
        self._sky_speed = sky_speed
        self._queue_size = queue_size
        self._processed = processed
        self._move_direction = move_direction
        self._goto_target = goto_target
        self._goto_direction = goto_direction

    def mode(self) -> AxisMotionMode:
        return self._mode

    def is_moving_to(self) -> bool:
        return False

    def get_position(self) -> PointCoordinates:
        if isinstance(self._position, Ha):
            return PointCoordinates(ra=self._position, dec=Dec(0))
        return PointCoordinates(ra=Ha(0), dec=self._position)

    def command_monitor(self) -> dict[str, object]:
        return {
            "queue_size": self._queue_size,
            "processed": list(self._processed),
        }


class _StubPolarCompensator:
    STABLE_GUIDE_PULSES_COUNT = 5
    DROP_GUIDE_PULSES_COUNT_AFTER = Second(20)
    STOP_AXIS_AFTER = Second(4.1)

    def __init__(self) -> None:
        self.current_ha = Ha(7210)
        self.current_dec = Dec(1820)
        self.eps_E = None
        self.eps_N = None
        self.ra_speed = STELLAR_SPEED * 1.25
        self.dec_speed = DecPerSecond(0.5)
        self.last_guide_pulse = Second(10)
        self.last_ra_guide_pulse = Second(10)
        self.last_dec_guide_pulse = Second(10)
        self.stable_guide_ra_pulses_count = 2
        self.stable_guide_dec_pulses_count = 3
        self._ra_speeds = [STELLAR_SPEED * 1.10, STELLAR_SPEED * 1.25]
        self._dec_speeds = [DecPerSecond(0.25), DecPerSecond(0.50), DecPerSecond(0.75)]


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
                    power_v=None,
                ),
                Ha(3600),
                power_v=13.4,
            ),
            Ha(3600),
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
                    power_v=None,
                ),
                Dec(1800),
                power_v=12.8,
            ),
            Dec(1800),
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


def test_stdout_dashboard_render_fits_30x130(monkeypatch) -> None:
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

    assert len(lines) <= 30
    assert all(len(line) <= 100 for line in lines)
    assert lines[0].startswith("RA")
    assert any("-- AXIS " in line for line in lines)
    assert any("-- MOTOR " in line for line in lines)
    assert any("-- POLAR " in line for line in lines)
    assert any("-- STATE " in line for line in lines)
    assert any("mode" in line and "track" in line for line in lines)
    assert any("clock" in line and "12:34:56" in line for line in lines)
    assert any("guide" in line and "1m31" in line for line in lines)
    assert any("mount_1s" in line and "+10.000hs" in line for line in lines)
    assert any("mount_1s" in line and "+20.00as" in line for line in lines)
    assert any("motor_1s" in line and "+5.000hs" in line for line in lines)
    assert any("motor_1s" in line and "+15.00as" in line for line in lines)
    assert any("avg" in line and "+1.250x sid" in line for line in lines)
    assert any("avg" in line and "+0.50as" in line for line in lines)
    assert any("flags" in line and "e=n s=n a=n" in line for line in lines)
    assert any("current" in line and "02:00:10" in line for line in lines)
    assert any("current" in line and "+00*30:20" in line for line in lines)
    assert any("ra_track" in line and "+1.000hs" in line for line in lines)
    assert any("dec_track" in line and "+0.00as" in line for line in lines)
    assert any("ra_bat" in line and "13.40V" in line for line in lines)
    assert any("dec_bat" in line and "12.80V" in line for line in lines)
    assert all("log_1" not in line and "log_2" not in line for line in lines)
    assert all("GR" not in line and "GD" not in line for line in lines)


def test_stdout_dashboard_state_column_shows_goto_details(monkeypatch) -> None:
    lx200 = _StubLX200()
    combiner = _StubCombiner()
    combiner.ra._mode = AxisMotionMode.GOTO
    combiner.ra._goto_direction = SkyDirection.EAST
    combiner.ra._goto_target = Ha(3660)
    combiner.ra._position = Ha(3610)
    combiner.dec._mode = AxisMotionMode.GOTO
    combiner.dec._goto_direction = SkyDirection.NORTH
    combiner.dec._goto_target = Dec(1860)
    combiner.dec._position = Dec(1820)

    dashboard = StdoutDashboard(combiner, lx200, refresh_s=0.01)

    monkeypatch.setattr(stdout_dashboard_module.time, "strftime", lambda _fmt: "12:34:56")
    monkeypatch.setattr(stdout_dashboard_module.time, "monotonic", lambda: 101.0)
    frame = dashboard.render(clear_screen=False)
    lines = frame.rstrip("\n").splitlines()

    assert any("mode" in line and "goto" in line for line in lines)
    assert any("ra_dir" in line and "east" in line for line in lines)
    assert any("ra_tgt" in line and "01:01:00" in line for line in lines)
    assert any("ra_left" in line and "00:00:50" in line for line in lines)
    assert any("dec_dir" in line and "north" in line for line in lines)
    assert any("dec_tgt" in line and "+00*31:00" in line for line in lines)
    assert any("dec_left" in line and "+00*00:40" in line for line in lines)
    assert all("log_1" not in line and "log_2" not in line for line in lines)


def test_stdout_dashboard_clear_screen_mode_sticks_frame_to_top(monkeypatch) -> None:
    lx200 = _StubLX200()
    combiner = _StubCombiner()
    dashboard = StdoutDashboard(combiner, lx200, refresh_s=0.01)

    monkeypatch.setattr(stdout_dashboard_module.time, "strftime", lambda _fmt: "12:34:56")
    monkeypatch.setattr(stdout_dashboard_module.time, "monotonic", lambda: 100.0)
    monkeypatch.setattr(
        stdout_dashboard_module.shutil,
        "get_terminal_size",
        lambda *args, **kwargs: os.terminal_size((130, 40)),
    )

    first_frame = dashboard.render(clear_screen=True)
    second_frame = dashboard.render(clear_screen=True)

    assert first_frame.startswith("\x1b[31;40r")
    assert "\x1b[1;1H\x1b[2KRA" in first_frame
    assert first_frame.endswith("\x1b[31;1H")
    assert second_frame.startswith("\x1b[31;40r\x1b7")
    assert second_frame.endswith("\x1b8")


def test_stdout_dashboard_wraps_exception_traceback(monkeypatch) -> None:
    lx200 = _StubLX200()
    combiner = _StubCombiner()
    dashboard = StdoutDashboard(combiner, lx200, refresh_s=0.01)

    def _raise_error():
        raise AttributeError("very long dashboard error message " * 8)

    monkeypatch.setattr(combiner, "get_position", _raise_error)
    monkeypatch.setattr(stdout_dashboard_module.time, "strftime", lambda _fmt: "12:34:56")
    monkeypatch.setattr(stdout_dashboard_module.time, "monotonic", lambda: 100.0)

    frame = dashboard.render(clear_screen=False)
    lines = frame.rstrip("\n").splitlines()

    assert lines[0].startswith("DASHBOARD ERROR")
    assert any("AttributeError" in line for line in lines)
    assert sum("very long dashboard error message" in line for line in lines) >= 2
    assert all(len(line) <= 100 for line in lines)
