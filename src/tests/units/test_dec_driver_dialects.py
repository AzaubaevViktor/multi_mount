"""The DEC driver against both firmware generations, and the price of the old one.

Three things are established here:

1. the driver finds out on its own which protocol the board speaks, and keeps
   working against a board that has not been reflashed;
2. a write is confirmed by what the controller echoed, not by the fact that the
   command was sent;
3. how much the framed protocol actually buys, in counted silent corruptions
   rather than in adjectives.

"Silent corruption" has one definition throughout: the driver returned a number
that disagrees with the simulator's own state. That is the only failure worth
this much machinery — an error is recoverable, a wrong position is not.
"""

import pytest
from sim import Chaos, ChaosProfile, Clock, SimSerialLine, TMC2209Sim, Transport
from sky.motor import MotionMode
from sky.physics import DecStepsPerSecond
from tmc2209.motor import (
    TMC2209Motor,
    TMC2209MotorEchoMismatchError,
    TMC2209MotorError,
    TMC2209MotorStaleResponseError,
    _Dialect,
)
from tmc2209.protocol import Op, encode_frame, to_centi

_LOSS = ChaosProfile(lose_read_byte=0.03)
_FLIP = ChaosProfile(flip_read_byte=0.01)
_SEEDS = 120
_READS_PER_SEED = 6
_TRUE_POSITION = 12345


def _make(clock: Clock, sim: TMC2209Sim, dialect: _Dialect, transport: Transport | None = None) -> TMC2209Motor:
    line = SimSerialLine(sim, clock, transport=transport, port="sim://dec", timeout_s=2, name="dec", terminator="\n")
    return TMC2209Motor(line, clock, dialect=dialect)


# ---------------------------------------------------------------------------
# 1. Which protocol is on the other end
# ---------------------------------------------------------------------------


def test_the_driver_detects_the_framed_firmware_on_connect() -> None:
    clock = Clock()
    motor = _make(clock, TMC2209Sim(clock), _Dialect.AUTO)

    motor.connect()

    assert motor._dialect is _Dialect.FRAMED


def test_a_board_that_was_not_reflashed_drops_the_driver_back_to_the_line_protocol() -> None:
    """The compatibility path: positive proof, and only positive proof, downgrades.

    Firmware 300a439 answers the HELLO frame with `0;error=unknown_cmd;` — a v2
    line. Nothing else may switch the driver down, because a downgrade means
    going back to a protocol that corrupts data quietly.
    """
    clock = Clock()
    sim = TMC2209Sim(clock, framed=False)
    motor = _make(clock, sim, _Dialect.AUTO)

    motor.connect()

    assert motor._dialect is _Dialect.LEGACY
    assert motor.status().steps == 0
    assert motor.set_speed(DecStepsPerSecond(1000)) == 1000
    assert sim.speed_sps == 1000.0


def test_a_silent_board_does_not_count_as_proof_of_old_firmware() -> None:
    """A chewed-up line looks identical on both firmwares, so it decides nothing.

    Guessing "legacy" here would be the worst outcome: the driver would go back
    to reporting damaged values as good ones on a board that had the checks.
    """
    clock = Clock()
    sim = TMC2209Sim(clock)
    motor = _make(clock, sim, _Dialect.AUTO, transport=Transport(on_read=lambda data: b"" if data.startswith(b"#") else data))

    motor.connect()

    assert motor._dialect is _Dialect.FRAMED


def test_the_dialect_can_be_pinned_without_a_probe() -> None:
    clock = Clock()
    sim = TMC2209Sim(clock, framed=False)
    motor = _make(clock, sim, _Dialect.LEGACY)

    motor.connect()

    assert motor.status().steps == 0


# ---------------------------------------------------------------------------
# 2. A write is confirmed by the echo
# ---------------------------------------------------------------------------


def test_set_speed_returns_what_the_controller_applied() -> None:
    clock = Clock()
    sim = TMC2209Sim(clock)
    motor = _make(clock, sim, _Dialect.AUTO)
    motor.connect()

    assert motor.set_speed(DecStepsPerSecond(1234)) == 1234
    assert sim.speed_sps == 1234.0


def test_a_controller_that_acknowledges_another_value_is_not_believed() -> None:
    """`set` answering `1;` has never been a confirmation (DEC_PROTOCOL.md §4).

    The board is made to acknowledge a well-formed, correctly checksummed speed
    that nobody asked for — the one damage a CRC cannot catch, because it is not
    damage. Only comparing the echo with the request catches it.
    """
    clock = Clock()
    sim = TMC2209Sim(clock)

    def acknowledge_another_speed(data: bytes) -> bytes:
        text = data.decode("ascii", errors="ignore")
        if text.startswith("#") and text[3:5] == f"{int(Op.SPEED):02X}":
            return encode_frame(Op.SPEED, sim._seq, to_centi(999.0).to_bytes(4, "big")).encode("ascii")
        return data

    motor = _make(clock, sim, _Dialect.FRAMED, transport=Transport(on_read=acknowledge_another_speed))
    motor.connect()

    with pytest.raises(TMC2209MotorEchoMismatchError, match="acknowledged speed="):
        motor.set_speed(DecStepsPerSecond(1000))


def test_an_answer_that_arrives_one_command_late_is_refused() -> None:
    """The sequence number, in the situation it exists for.

    The board went quiet long enough for the read to time out, the driver asked
    again, and now the *previous* answer is the first thing on the wire. It
    describes an axis that has moved since, and nothing in its shape says so —
    only the number it was stamped with.
    """
    clock = Clock()
    sim = TMC2209Sim(clock)
    transport = Transport()
    motor = _make(clock, sim, _Dialect.FRAMED, transport=transport)
    motor.connect()

    held: list[bytes] = []

    def deliver_one_reply_late(data: bytes) -> bytes:
        if not data:
            return data
        held.append(data)
        return held.pop(0) if len(held) > 1 else b""

    transport.on_read = deliver_one_reply_late

    sim.position = 1000.0
    with pytest.raises(TMC2209MotorError):
        motor.status()  # its answer is still in flight

    sim.position = 2000.0
    with pytest.raises(TMC2209MotorStaleResponseError):
        motor.status()


def test_run_and_stop_are_confirmed_too() -> None:
    clock = Clock()
    sim = TMC2209Sim(clock)
    motor = _make(clock, sim, _Dialect.AUTO)
    motor.connect()
    motor.set_motion_mode(MotionMode.RUN)

    assert motor.run() is True
    assert sim.running is True
    assert motor.stop() is True
    assert sim.stop_requested is True


# ---------------------------------------------------------------------------
# 3. What the framed protocol is worth, counted
# ---------------------------------------------------------------------------


def _count_silent_corruptions(dialect: _Dialect, profile: ChaosProfile) -> tuple[int, int]:
    """Read a known state through a damaged line; return (silent, raised).

    A read is *silent corruption* when it came back without an error and any of
    the three numbers the axis layer acts on — position, speed, acceleration —
    disagrees with the controller's own state.
    """
    silent = 0
    raised = 0
    for seed in range(_SEEDS):
        chaos = Chaos(seed=seed, profile=ChaosProfile())
        clock = Clock()
        sim = TMC2209Sim(clock, framed=dialect is _Dialect.FRAMED)
        line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://dec", timeout_s=2,
                             name="dec", terminator="\n")
        chaos.bind(line)
        motor = TMC2209Motor(line, clock, dialect=dialect)
        motor.connect()
        sim.position = float(_TRUE_POSITION)
        sim.speed_sps = 1000.0
        sim.accel_sps2 = 2500.0
        truth = (_TRUE_POSITION, 1000, 2500)

        chaos.profile = profile
        for _ in range(_READS_PER_SEED):
            try:
                status = motor.status()
            except (TMC2209MotorError, OSError, KeyError, ValueError):
                raised += 1
                continue
            if (status.steps, int(status.speed_sps), status.accel_sps) != truth:
                silent += 1
    return silent, raised


@pytest.mark.parametrize("profile_name,profile", [("byte loss", _LOSS), ("bit flip", _FLIP)])
def test_the_line_protocol_reports_positions_the_controller_never_had(profile_name: str, profile: ChaosProfile) -> None:
    """The defect, measured: this is the number the framed protocol has to beat."""
    silent, raised = _count_silent_corruptions(_Dialect.LEGACY, profile)

    assert silent > 0, f"{profile_name}: the run proves nothing if nothing was corrupted"
    total = _SEEDS * _READS_PER_SEED
    print(f"legacy/{profile_name}: {silent} silent corruptions, {raised} honest errors, "
          f"{total - raised - silent} good reads out of {total}")


@pytest.mark.parametrize("profile_name,profile", [("byte loss", _LOSS), ("bit flip", _FLIP)])
def test_the_framed_protocol_never_reports_a_position_the_controller_never_had(
    profile_name: str, profile: ChaosProfile
) -> None:
    silent, raised = _count_silent_corruptions(_Dialect.FRAMED, profile)

    assert raised > 0, f"{profile_name}: the run proves nothing if nothing was damaged"
    assert silent == 0, f"{profile_name}: {silent} damaged frames were reported as positions"
    total = _SEEDS * _READS_PER_SEED
    print(f"framed/{profile_name}: {silent} silent corruptions, {raised} honest errors, "
          f"{total - raised - silent} good reads out of {total}")


def test_a_damaged_status_never_escapes_as_a_bare_keyerror() -> None:
    """PLAN.md #33, in both dialects: everything that gets out is a driver error."""
    for dialect in (_Dialect.LEGACY, _Dialect.FRAMED):
        leaked: list[str] = []
        for seed in range(40):
            chaos = Chaos(seed=seed, profile=ChaosProfile())
            clock = Clock()
            sim = TMC2209Sim(clock, framed=dialect is _Dialect.FRAMED)
            line = SimSerialLine(sim, clock, transport=chaos.transport, port="sim://dec", timeout_s=2,
                                 name="dec", terminator="\n")
            chaos.bind(line)
            motor = TMC2209Motor(line, clock, dialect=dialect)
            motor.connect()

            chaos.profile = ChaosProfile(lose_read_byte=0.05, flip_read_byte=0.02)
            for _ in range(6):
                try:
                    motor.status()
                except (TMC2209MotorError, OSError):
                    continue
                except Exception as error:  # noqa: BLE001 - classifying is the point
                    leaked.append(f"seed={seed}: {type(error).__name__}: {error}")

        assert not leaked, f"{dialect}: {leaked[:3]}"


# ---------------------------------------------------------------------------
# 4. What a retry does to the line
# ---------------------------------------------------------------------------


class _OrderRecordingLine:
    """A line that fails once and remembers in which order it was handled."""

    terminator = b"\n"

    def __init__(self) -> None:
        self.events: list[str] = []
        self.attempts = 0

    def query(
        self,
        payload: str | None,
        timeout: float | None = None,
        response_prefixes: tuple[bytes, ...] | None = None,
        response_terminator: bytes | str | None = None,
    ) -> str:
        self.events.append("query")
        self.attempts += 1
        if self.attempts == 1:
            return "0;error=boom;\n"
        return "1;position=17;\n"

    def drop_buffers(self) -> None:
        self.events.append("drop_buffers")

    def read_all_data(self, timeout: float | None = None) -> list[str] | None:
        self.events.append("read_all_data")
        return ["leftover"]


def test_a_retry_reads_the_leftovers_before_it_throws_them_away() -> None:
    """The March defect, in the copy of the retry loop that still had it.

    `drop_buffers()` and then `read_all_data(timeout=.5)` cannot log anything:
    the bytes that confused the parser have just been discarded, so the read
    can only wait half a second for a board that has already said everything.
    That is where the 20 601 records of `['']` came from. The RA driver was
    fixed; this one is the second copy of the same loop, and it kept the bug
    long after the two protocols stopped having anything else in common.
    """
    clock = Clock()
    line = _OrderRecordingLine()
    motor = TMC2209Motor(line, clock, dialect=_Dialect.LEGACY)  # type: ignore[arg-type]

    response = motor._transact("status")

    assert response.values["position"] == "17"
    assert line.events == ["query", "read_all_data", "drop_buffers", "query"], (
        "the leftovers were dropped before anyone looked at them"
    )
