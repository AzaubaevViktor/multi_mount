import lx200.base as lx200_base_module
from lx200.base import LX200Handler
from sky.physics import Dec, Ha, Second


class _RecordingLX200(LX200Handler):
    def __init__(self) -> None:
        super().__init__()
        self.calls: list[str] = []

    def get_telescope_ra(self) -> Ha:
        return Ha(0)

    def sync_telescope(self, ra: Ha, dec: Dec) -> bool:
        return True

    def get_telescope_dec(self) -> Dec:
        return Dec(0)

    def slew_to(self, ra: Ha, dec: Dec) -> bool:
        return True

    def move_east(self) -> bool:
        self.calls.append("move_east")
        return True

    def move_north(self) -> bool:
        self.calls.append("move_north")
        return True

    def move_south(self) -> bool:
        self.calls.append("move_south")
        return True

    def move_west(self) -> bool:
        self.calls.append("move_west")
        return True

    def halt_all(self) -> bool:
        self.calls.append("halt_all")
        return True

    def stop_all(self) -> bool:
        self.calls.append("stop_all")
        return True

    def halt_east(self) -> bool:
        self.calls.append("halt_east")
        return True

    def halt_north(self) -> bool:
        self.calls.append("halt_north")
        return True

    def halt_south(self) -> bool:
        self.calls.append("halt_south")
        return True

    def halt_west(self) -> bool:
        self.calls.append("halt_west")
        return True

    def guide_east(self, ms: int) -> None:
        return None

    def guide_north(self, ms: int) -> None:
        return None

    def guide_south(self, ms: int) -> None:
        return None

    def guide_west(self, ms: int) -> None:
        return None


def test_double_halt_all_stops_all(monkeypatch) -> None:
    handler = _RecordingLX200()
    timestamps = iter((Second(10.0), Second(10.1)))
    monkeypatch.setattr(lx200_base_module.Second, "monotonic", classmethod(lambda cls: next(timestamps)))

    handler.handle("Q")
    handler.handle("Q")

    assert handler.calls == ["halt_all", "stop_all"]


def test_second_halt_all_after_window_is_regular_halt(monkeypatch) -> None:
    handler = _RecordingLX200()
    timestamps = iter((Second(20.0), Second(21.3)))
    monkeypatch.setattr(lx200_base_module.Second, "monotonic", classmethod(lambda cls: next(timestamps)))

    handler.handle("Q")
    handler.handle("Q")

    assert handler.calls == ["halt_all", "halt_all"]


def test_motion_resets_double_halt_all_window(monkeypatch) -> None:
    handler = _RecordingLX200()
    timestamps = iter((Second(30.0), Second(30.1), Second(30.2)))
    monkeypatch.setattr(lx200_base_module.Second, "monotonic", classmethod(lambda cls: next(timestamps)))

    handler.handle("Q")
    handler.handle("Me")
    handler.handle("Q")

    assert handler.calls == ["halt_all", "move_east", "halt_all"]


def test_ra_halt_commands_support_lx200_client_mapping() -> None:
    handler = _RecordingLX200()

    handler.handle("Me")
    handler.handle("Qw")
    handler.handle("Mw")
    handler.handle("Qe")

    assert handler.calls == [
        "move_east",
        "halt_east",
        "move_west",
        "halt_west",
    ]


def test_ra_halt_commands_keep_legacy_same_direction_mapping() -> None:
    handler = _RecordingLX200()

    handler.handle("Me")
    handler.handle("Qe")
    handler.handle("Mw")
    handler.handle("Qw")

    assert handler.calls == [
        "move_east",
        "halt_east",
        "move_west",
        "halt_west",
    ]


def test_new_ra_manual_move_replaces_stale_opposite_direction() -> None:
    handler = _RecordingLX200()

    handler.handle("Mw")
    handler.handle("Me")
    handler.handle("Qw")

    assert handler.calls == [
        "move_west",
        "move_east",
        "halt_east",
    ]
