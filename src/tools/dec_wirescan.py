"""Live commentary on the TMC2209 link while somebody rewires it by hand.

The driver module's silkscreen is unreadable, so which of pins 8/9 is RX has to be
found by trying. The board half of this is ``telescope_dec/src/wirescan.cpp``: it
probes both orientations several times a second and prints one line per round. This
half watches those lines and *talks*, because the person doing the rewiring is
looking at the board, not at a terminal.

Speaking rules, which are most of the design
--------------------------------------------
Announcing every round would be unusable, and announcing nothing would leave the
user wondering whether the thing is alive. So:

* any **change** of state is spoken at once -- that is the whole point, the moment a
  wire lands right;
* an **unchanged** state is repeated only every ``CALM_INTERVAL_S``, as a sign of
  life;
* the same sentence is never said twice in a row.

Speech is fired off in the background: a blocking ``say`` would stretch a 150 ms
round into a second and blur the very timing the user is listening for.

The port is expected to come and go -- the board has been unplugged mid-session more
than once, and rewiring invites it -- so a disappearing device is a state to report,
not an error to crash on. The port is re-discovered by glob each time, because macOS
renumbers it (``usbserial-2110`` became ``usbserial-110``); the RA board is a PL2303
and cannot match this pattern.

Run it as::

    .venv/bin/python -m tools.dec_wirescan

Stop with Ctrl+C.
"""

import argparse
import glob
import logging
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from logging_setup import setup_logging  # noqa: E402
from serial_wrapper.recorder import JsonlRecorder, TraceEvent, TraceKind  # noqa: E402

LOGGER = logging.getLogger("dec_wirescan")

PORT_GLOB = "/dev/tty.usbserial*"
BAUD = 115200
# The sync byte the board sends; hearing it back means the wire is sound.
TMC_SYNC = 0x05
SAY_BINARY = "/usr/bin/say"
VOICE = "Milena"
SPEAKER = "Агент ДЕК."
# The LEDs carry the live state, but the voice is what the person rewiring actually
# has attention for -- they are looking at the board, not at it. So it also confirms
# an unchanged situation this often, as a sign of life. Set to 0 to silence that and
# leave only the changes.
CALM_INTERVAL_S = 25.0
# Rounds a state must survive before it is worth saying out loud. A hand resting on a
# wire makes the reading flap several times a second; without this the voice chases
# every twitch, which is precisely what made the first version unusable.
STABLE_ROUNDS = 4
LOG_DIR = Path(__file__).resolve().parents[2] / "logs" / "protocol"

# VERSION byte in IOIN, used only to name the chip once it answers. The board here
# carries a TMC2225, which is a TMC2208 variant and reports 0x20; requiring 0x21 was a
# bug that would have called a working link a silent one.
CHIP_NAMES = {0x20: "двадцать два ноль восемь или двадцать два двадцать пять", 0x21: "двадцать два ноль девять"}


@dataclass(frozen=True)
class Side:
    """One pin orientation as the board reported it."""

    linked: bool
    idle_high: bool
    echo: int
    decay_us: int
    raw: int
    version: int
    ifcnt: int
    state: int
    ever_version: int = 0
    ever_count: int = 0

    @property
    def replied(self) -> bool:
        return self.raw >= 8 and self.version != 0

    @property
    def stuck_low(self) -> bool:
        """The receive line is held down, so `raw` is noise rather than an answer.

        A UART line idles high. When it does not, the receiver reads a start bit that
        never ends and fills its buffer -- which arrives here as a healthy-looking
        `raw=12` and would otherwise be mistaken for a reply.
        """
        return not self.idle_high


@dataclass(frozen=True)
class Round:
    a: Side
    b: Side
    vm_centivolts: int = 0

    @property
    def powered(self) -> bool:
        """Motor supply present. A driver with no power is silent for a reason that
        has nothing to do with the wire, and that is worth saying out loud."""
        return self.vm_centivolts >= 600

    @property
    def rank(self) -> int:
        """Best of the two orientations, on the board's own 0..3 scale.

        Speaking is driven by this single number rather than by the full reading:
        which orientation is which, and whether `raw` is 8 or 12, is what the LEDs
        are for. The voice only reports that things got better or worse.
        """
        return max(self.a.state, self.b.state)


def parse_round(line: str) -> Round | None:
    values: dict[str, int] = {}
    for token in line.strip().split(";"):
        if "=" not in token:
            continue
        name, _, raw = token.partition("=")
        try:
            values[name.strip()] = int(raw)
        except ValueError:
            return None
    try:
        return Round(
            a=Side(bool(values["a_link"]), bool(values["a_idle"]), values["a_echo"],
                   values["a_decay"], values["a_raw"], values["a_ver"],
                   values["a_ifcnt"], values["a_state"],
                   values.get("a_everver", 0), values.get("a_evercount", 0)),
            b=Side(bool(values["b_link"]), bool(values["b_idle"]), values["b_echo"],
                   values["b_decay"], values["b_raw"], values["b_ver"],
                   values["b_ifcnt"], values["b_state"],
                   values.get("b_everver", 0), values.get("b_evercount", 0)),
            vm_centivolts=values.get("vm", 0),
        )
    except KeyError:
        return None


# Spoken names for the two orientations. Words rather than single letters: said
# aloud, "в первой" is unmistakable where bare letter names are not -- and naming the
# receive pin turns the announcement into the answer the user is actually after.
ORIENTATIONS = (("первой", "приём на восьмом"), ("второй", "приём на девятом"))


def describe(state: Round) -> str:
    """One short Russian sentence for the current situation, best news first."""
    sides = (state.a, state.b)
    # A reply that has *ever* landed outranks the live reading: the one real answer of
    # the session lasted a single round, and reporting only the present would have
    # thrown it away again.
    for (name, pin), side in zip(ORIENTATIONS, sides, strict=True):
        if side.ever_count and not side.replied:
            chip = CHIP_NAMES.get(side.ever_version, f"версия {side.ever_version:#04x}")
            return (f"Драйвер уже отвечал в {name} ориентации, {pin}. Это {chip}. "
                    f"Ответов всего {side.ever_count}. Ищи это положение снова.")
    for (name, pin), side in zip(ORIENTATIONS, sides, strict=True):
        if side.replied:
            chip = CHIP_NAMES.get(side.version, f"версия {side.version:#04x}")
            found = f"Драйвер отвечает в {name} ориентации, {pin}. Это {chip}."
            return f"{found} Запись проходит." if side.ifcnt > 0 else found
    if not state.powered:
        return "Силовое питание не подключено. Драйвер обесточен, поэтому и молчит."
    good = [name for (name, _), side in zip(ORIENTATIONS, sides, strict=True) if side.echo == TMC_SYNC]
    if good:
        return f"Провод хороший, эхо чистое в {' и '.join(good)} ориентации. Но драйвер молчит."
    grounded = [pin for (_, pin), side in zip(ORIENTATIONS, sides, strict=True) if side.echo == 0x00]
    if grounded:
        # A different fault from a line hanging in the air, and a different move.
        return f"Линия прижата к земле: {grounded[0]}. Провод сидит на земляном пятаке."
    return "Провода ни к чему не подключены. Эха нет совсем."


def say(text: str) -> None:
    """Fire and forget: a blocking say would blur the timing the user listens for."""
    try:
        subprocess.Popen(  # noqa: S603
            [SAY_BINARY, "-v", VOICE, f"{SPEAKER} {text}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except OSError as error:
        LOGGER.warning("say failed: %s", error)


def find_port() -> str | None:
    ports = sorted(glob.glob(PORT_GLOB))
    return ports[0] if ports else None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="serial port; found by glob when omitted")
    parser.add_argument("--quiet", action="store_true", help="print rounds but do not speak")
    parser.add_argument("--calm-interval", type=float, default=CALM_INTERVAL_S,
                        help="seconds between repeats of an unchanged state")
    args = parser.parse_args(argv)
    setup_logging()

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    trace = LOG_DIR / f"dec_wirescan-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"
    recorder = JsonlRecorder(trace)
    started = time.monotonic()

    def record(kind: TraceKind, data: bytes = b"", **detail: Any) -> None:
        recorder.record(TraceEvent(at_s=round(time.monotonic() - started, 6), name="dec",
                                   port=args.port or "?", kind=kind, data=data, detail=detail))

    def announce(text: str) -> None:
        LOGGER.info("SAY: %s", text)
        record(TraceKind.MARK, spoken=text)
        if not args.quiet:
            say(text)

    LOGGER.info("trace: %s", trace)
    announce("Слушаю линию. Состояние на светодиодах, говорить буду только по делу.")

    port: serial.Serial | None = None
    spoken_key: int | None = None
    spoken_at = 0.0
    pending_rank = -1
    pending_rounds = 0
    was_connected = False

    try:
        while True:
            if port is None:
                name = args.port or find_port()
                if name is None:
                    if was_connected:
                        announce("Плата отключилась от USB.")
                        was_connected = False
                        spoken_key = None
                        pending_rank = -1
                    time.sleep(1.0)
                    continue
                try:
                    port = serial.Serial(name, BAUD, timeout=1.0)
                except (OSError, serial.SerialException):
                    time.sleep(1.0)
                    continue
                record(TraceKind.OPEN, baud=BAUD, resolved_port=name)
                if not was_connected:
                    announce("Плата на связи. Смотри на светодиоды.")
                    was_connected = True
                    spoken_key = None
                    pending_rank = -1
                continue

            try:
                raw = port.readline()
            except (OSError, serial.SerialException):
                record(TraceKind.ERROR, reason="read failed")
                port.close()
                port = None
                continue

            if not raw:
                continue
            record(TraceKind.RX, raw)
            state = parse_round(raw.decode("ascii", "replace"))
            if state is None:
                continue

            LOGGER.info(
                "VM=%5.2fV | A echo=0x%02X decay=%4d raw=%2d ver=0x%02X | "
                "B echo=0x%02X decay=%4d raw=%2d ver=0x%02X",
                state.vm_centivolts / 100, state.a.echo, state.a.decay_us, state.a.raw, state.a.version,
                state.b.echo, state.b.decay_us, state.b.raw, state.b.version,
            )

            now = time.monotonic()
            if state.rank == pending_rank:
                pending_rounds += 1
            else:
                pending_rank = state.rank
                pending_rounds = 1

            settled = pending_rounds == STABLE_ROUNDS and pending_rank != spoken_key
            stale = args.calm_interval > 0 and now - spoken_at >= args.calm_interval
            if settled or stale:
                announce(describe(state))
                spoken_key = pending_rank
                spoken_at = now
    except KeyboardInterrupt:
        LOGGER.info("stopped by the operator")
    finally:
        if port is not None:
            record(TraceKind.CLOSE)
            port.close()
        recorder.close()
        LOGGER.info("trace: %s", trace)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
