"""The RA step-2 experiments: what a table of cases cannot express.

``tools.ra_conformance`` pins claims that are one request and one reply. The
three questions left over from ``docs/protocol/RA_PROTOCOL_STEP_2.md`` are not
of that shape — they are *scenarios*:

``probe``  what ``:m1``, ``:d1`` and ``:h1`` are functions **of** (§3.1 read the
           board twice with position and target both at ``0x800000``, so it
           could not tell them apart), and whether the position moves on its own
           on a standing axis;
``goto``   the vendor GOTO of ``RA_SA_CONSOLE_PROTOCOL.md`` §6 end to end:
           ``:f1`` → ``:j1`` → ``:G12<dir>`` → ``:H1<steps>`` → ``:M1AC0D00`` →
           ``:J1``, polled at 100 ms — acceleration profile, braking, arrival
           error, status at every step;
``load``   both supply voltages (§6.8) at rest and while the axis runs at 1x,
           16x, 64x and 100x sidereal in both directions;
``clamp``  the driver's own minimum-period measurement (``probe_board``) against
           the live board: does it read 1103 and does it put the period back.

Safety, in the same three layers the conformance runner uses:

1. every payload goes through :class:`ra_conformance.HardwareWire`, which checks
   the whitelist of ``RA_PROTOCOL_STEP_2.md`` §1.1 **before** the write;
2. every leg that turns the axis has a deadline; when it expires the leg is
   abandoned with ``:K1`` instead of being waited out;
3. the axis is stopped and the stop is *confirmed* before the port is closed,
   on every exit path including an exception.

Usage::

    PYTHONPATH=src .venv/bin/python -m tools.ra_step2 probe --port /dev/tty.X
    PYTHONPATH=src .venv/bin/python -m tools.ra_step2 goto  --port /dev/tty.X
    PYTHONPATH=src .venv/bin/python -m tools.ra_step2 load  --port /dev/tty.X
    PYTHONPATH=src .venv/bin/python -m tools.ra_step2 clamp --port /dev/tty.X
"""

import argparse
import dataclasses
import datetime
import itertools
import json
import logging
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from clock import REAL_CLOCK
from logging_setup import setup_logging
from ra_conformance import HardwareWire, Safety, Wire, stop_axis
from serial_wrapper.recorder import JsonlRecorder
from serial_wrapper.wrapper import SerialLine

LOGGER = logging.getLogger("ra_step2")

BAUD = 115200
TIMEOUT_S = 0.5

POSITION_OFFSET = 0x800000
TIMER_FREQ = 16_000_000
PERIOD_1X = 110_359
PERIOD_16X = 6_897
PERIOD_64X = 1_724
PERIOD_100X = 1_103
# `RA_SA_CONSOLE_PROTOCOL.md` §6: the vendor app always sends this break
# increment before `:J1`, whatever the distance — 0x0DAC = 3500 steps.
VENDOR_BRAKE_INCREMENT = 0x0DAC
# How many GOTO legs of `_goto_legs()` one run is allowed to attempt. A module
# constant rather than a flag: the point of it is to be edited deliberately,
# after looking at what the previous run did to the board.
GOTO_LEG_LIMIT = 1

BATTERY_ADDRESS = 0x0004
USB_ADDRESS = 0x001C


def revu24(value: int) -> str:
    """110359 -> ``17AF01``: three bytes, least significant first."""
    value &= 0xFFFFFF
    return f"{value & 0xFF:02X}{(value >> 8) & 0xFF:02X}{(value >> 16) & 0xFF:02X}"


def unrevu24(body: str) -> int:
    return int(body[4:6] + body[2:4] + body[0:2], 16)


@dataclasses.dataclass(frozen=True)
class StatusBits:
    """`:f1` -> three hex chars, §10.1."""

    raw: str

    @property
    def tracking(self) -> bool:
        return bool(int(self.raw[0], 16) & 0b001)

    @property
    def ccw(self) -> bool:
        return bool(int(self.raw[0], 16) & 0b010)

    @property
    def fast(self) -> bool:
        return bool(int(self.raw[0], 16) & 0b100)

    @property
    def running(self) -> bool:
        return bool(int(self.raw[1], 16) & 0b001)

    @property
    def initialized(self) -> bool:
        return bool(int(self.raw[2], 16) & 0b001)

    def as_dict(self) -> dict[str, Any]:
        return {
            "raw": self.raw,
            "mode": "tracking" if self.tracking else "goto",
            "dir": "ccw" if self.ccw else "cw",
            "fast": self.fast,
            "running": self.running,
            "init": self.initialized,
        }


class BoardSatDown(RuntimeError):
    """The board stopped answering mid-experiment — the failure §12 is about.

    Raised instead of retried on purpose: a board that has just gone silent
    under load is not to be prodded further, it is to be reported.
    """


class Board:
    """Request/response over a guarded wire, with the decoders step 2 needs."""

    def __init__(self, wire: Wire) -> None:
        self._wire = wire

    # -- transport ----------------------------------------------------------

    def ask(self, payload: str, safety: Safety = Safety.READ) -> str:
        reply = self._wire.exchange(f"{payload}\r".encode("ascii"), safety)
        return reply.decode("ascii", errors="replace").rstrip("\r")

    def value(self, payload: str) -> str:
        reply = self.ask(payload)
        if not reply.startswith("="):
            raise BoardSatDown(f"{payload} ответила {reply!r}, а не значением")
        return reply[1:]

    def set(self, payload: str, safety: Safety = Safety.WRITE) -> str:
        return self.ask(payload, safety)

    def now(self) -> float:
        return self._wire.now()

    def pause(self, seconds: float) -> None:
        self._wire.pause(seconds)

    # -- inquiries ----------------------------------------------------------

    def status(self) -> StatusBits:
        return StatusBits(self.value(":f1"))

    def position(self) -> int:
        return unrevu24(self.value(":j1"))

    def target(self) -> int:
        return unrevu24(self.value(":h1"))

    def brake_point(self) -> int:
        return unrevu24(self.value(":m1"))

    def tele_position(self) -> int:
        return unrevu24(self.value(":d1"))

    def period(self) -> int:
        return unrevu24(self.value(":i1"))

    def window_byte(self, address: int) -> int:
        self.set(f":C1{address & 0xFF:02X}{(address >> 8) & 0xFF:02X}")
        return int(self.value(":n1"), 16)

    def window_int16(self, address: int) -> int:
        return self.window_byte(address) | (self.window_byte(address + 1) << 8)

    def voltages(self) -> dict[str, float]:
        battery = self.window_int16(BATTERY_ADDRESS)
        usb = self.window_int16(USB_ADDRESS)
        return {"battery_v": battery / 100, "usb_v": usb / 100, "battery_raw": battery, "usb_raw": usb}

    # -- motion -------------------------------------------------------------

    def stop_and_wait(self, budget_s: float = 8.0) -> bool:
        """`:K1` and poll until the Running bit drops. §10.3: it is a ramp."""
        self.set(":K1", Safety.MOTION)
        deadline = self.now() + budget_s
        while self.now() < deadline:
            if not self.status().running:
                return True
            self.pause(0.1)
        return not self.status().running


# --------------------------------------------------------------------------- #
# probe: what `:m1`, `:d1` and `:h1` are functions of
# --------------------------------------------------------------------------- #


def phase_probe(board: Board, out: list[dict[str, Any]]) -> None:
    def snapshot(label: str, **extra: Any) -> dict[str, Any]:
        row = {
            "step": label,
            "j1": board.position(),
            "d1": board.tele_position(),
            "h1": board.target(),
            "m1": board.brake_point(),
            "f1": board.status().as_dict(),
            **extra,
        }
        out.append(row)
        LOGGER.info(
            "%s: j1=0x%06X d1=0x%06X h1=0x%06X m1=0x%06X f1=%s",
            label, row["j1"], row["d1"], row["h1"], row["m1"], row["f1"]["raw"],
        )
        return row

    brake_steps = unrevu24(board.value(":c1"))
    LOGGER.info("`:c1` brake steps = %d", brake_steps)
    out.append({"step": "c1", "brake_steps": brake_steps})

    # 1. Does the position move on a standing axis? Ten seconds of watching.
    started = board.now()
    drift: list[dict[str, float]] = []
    while board.now() - started < 10.0:
        drift.append({"at_s": round(board.now() - started, 3), "j1": board.position()})
        board.pause(0.5)
    positions = {row["j1"] for row in drift}
    LOGGER.info("покой 10 с: позиций %d, диапазон %s", len(positions), sorted(positions))
    out.append({"step": "rest_drift", "samples": drift, "distinct": sorted(positions)})

    snapshot("as_found")

    # 2. Does the brake point follow the *target*?
    board.set(f":H1{revu24(50_000)}")
    snapshot("after_H_plus_50000", brake_steps=brake_steps)

    board.set(f":M1{revu24(VENDOR_BRAKE_INCREMENT)}")
    snapshot("after_M_3500")

    board.set(f":H1{revu24(0)}")
    snapshot("after_H_zero")

    # 3. Does `:E` (set position) drag `:d1`, `:h1` and `:m1` with it?
    board.set(f":E1{revu24(POSITION_OFFSET)}")
    snapshot("after_E_logical_zero")

    board.set(f":M1{revu24(0)}")
    snapshot("after_M_zero")


# --------------------------------------------------------------------------- #
# goto: the vendor sequence, end to end
# --------------------------------------------------------------------------- #


@dataclasses.dataclass(frozen=True)
class Leg:
    steps: int
    period: int
    # Second char of `:G1`. The mapping is *not* the intuitive one, it is the
    # one `skywatcher.codec.MotionMode` encodes: '1' slew+slow (tracking),
    # '3' slew+fast, '2' goto+slow — the vendor's — and '0' goto+fast.
    mode: str
    direction: str  # third char, '0' CW / '1' CCW
    label: str


def _goto_legs() -> list[Leg]:
    """Shortest and slowest first: every later leg is a bigger bet on the board."""
    return [
        Leg(1_000, PERIOD_1X, "2", "0", "вендорский `:G120`, 1000 шагов, период 1×, CW"),
        Leg(1_000, PERIOD_1X, "2", "1", "вендорский `:G120`, 1000 шагов, период 1×, CCW"),
        Leg(20_000, PERIOD_16X, "2", "0", "вендорский `:G120`, 20 000 шагов, период 16×, CW"),
        Leg(20_000, PERIOD_16X, "2", "1", "вендорский `:G120`, 20 000 шагов, период 16×, CCW"),
        Leg(100_000, PERIOD_64X, "2", "0", "вендорский `:G120`, 100 000 шагов, период 64×, CW"),
        Leg(100_000, PERIOD_64X, "0", "1", "быстрый goto `:G101`, 100 000 шагов, период 64×, CCW"),
    ]


def _run_goto(board: Board, leg: Leg) -> dict[str, Any]:
    expected_s = leg.steps * leg.period / TIMER_FREQ
    deadline_s = expected_s * 3 + 5
    LOGGER.info("GOTO %s: ожидаемая длительность %.1f с, дедлайн %.1f с", leg.label, expected_s, deadline_s)

    board.set(f":I1{revu24(leg.period)}")
    row: dict[str, Any] = {
        "leg": leg.label,
        "steps": leg.steps,
        "period": leg.period,
        "mode": leg.mode,
        "direction": leg.direction,
        "expected_s": round(expected_s, 3),
        "status_before": board.status().as_dict(),
    }
    start_position = board.position()
    row["position_before"] = start_position

    row["reply_G"] = board.set(f":G1{leg.mode}{leg.direction}")
    row["status_after_G"] = board.status().as_dict()
    row["reply_H"] = board.set(f":H1{revu24(leg.steps)}")
    row["target_after_H"] = board.target()
    row["brake_after_H"] = board.brake_point()
    row["reply_M"] = board.set(f":M1{revu24(VENDOR_BRAKE_INCREMENT)}")
    row["target_after_M"] = board.target()
    row["brake_after_M"] = board.brake_point()

    started = board.now()
    row["reply_J"] = board.set(":J1", Safety.MOTION)
    # Acceleration profile: poll as fast as the wire allows for the first
    # 0.6 s, then fall back to the vendor's own 100 ms cadence.
    profile: list[dict[str, Any]] = []
    stopped_samples = 0
    while True:
        elapsed = board.now() - started
        status = board.status()
        position = board.position()
        profile.append(
            {
                "at_s": round(elapsed, 4),
                "j1": position,
                "delta": (position - start_position) % 0x1000000,
                "f1": status.raw,
                "running": status.running,
            }
        )
        # Two consecutive stopped samples, and not before the axis has had time
        # to start at all: one `running == False` right after `:J1` would
        # otherwise be read as an arrival that never happened.
        stopped_samples = 0 if status.running else stopped_samples + 1
        if stopped_samples >= 2 and elapsed > 0.3:
            row["outcome"] = "arrived"
            break
        if elapsed > deadline_s:
            row["outcome"] = "deadline"
            LOGGER.error("GOTO %s не уложился в дедлайн — торможу", leg.label)
            board.stop_and_wait()
            break
        if elapsed > 0.6:
            board.pause(0.1)
    row["elapsed_s"] = round(board.now() - started, 3)
    row["profile"] = profile

    board.pause(0.3)
    final = board.position()
    row["position_after"] = final
    row["travelled"] = (final - start_position) % 0x1000000
    row["target_final"] = board.target()
    row["brake_final"] = board.brake_point()
    row["tele_final"] = board.tele_position()
    row["status_after"] = board.status().as_dict()
    signed = row["travelled"] if row["travelled"] < 0x800000 else row["travelled"] - 0x1000000
    wanted = leg.steps if leg.direction == "0" else -leg.steps
    row["error_steps"] = signed - wanted
    LOGGER.info(
        "GOTO %s: прошло %d шагов из %d, ошибка %+d, за %.2f с (%s)",
        leg.label, signed, wanted, row["error_steps"], row["elapsed_s"], row["outcome"],
    )
    return row


def phase_goto(board: Board, out: list[dict[str, Any]]) -> None:
    board.set(":F1")
    for leg in _goto_legs()[:GOTO_LEG_LIMIT]:
        row = _run_goto(board, leg)
        out.append(row)
        board.stop_and_wait()
        if not board.status().initialized:
            # Nobody sent `:E`, so §11 cannot explain this: the board reset
            # under the GOTO ramp (§12.2). One occurrence ends the phase.
            out.append({"step": "board_reset_detected", "after_leg": leg.label})
            LOGGER.critical("Флаг инициализации сброшен после плеча «%s» — плата села. Фаза прервана", leg.label)
            break
    # Back to the power-on state of §12.1 on a standing axis.
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")
    board.set(f":M1{revu24(0)}")


# --------------------------------------------------------------------------- #
# load: both supply voltages against speed
# --------------------------------------------------------------------------- #


_LOAD_PERIODS = (
    (PERIOD_1X, "1×"),
    (PERIOD_16X, "16×"),
    (PERIOD_64X, "64×"),
    (PERIOD_100X, "100×"),
)
_LOAD_SAMPLES = 8
_LOAD_RUN_S = 4.0


def _sample_voltage(board: Board, phase: str, label: str, count: int) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for _ in range(count):
        row = {"phase": phase, "speed": label, **board.voltages(), "f1": board.status().raw}
        rows.append(row)
    return rows


def phase_load(board: Board, out: list[dict[str, Any]]) -> None:
    board.set(":F1")
    out.extend(_sample_voltage(board, "rest", "—", 15))

    for period, label in _LOAD_PERIODS:
        for direction in ("0", "1"):
            board.set(f":I1{revu24(period)}")
            board.set(f":G11{direction}")
            position_before = board.position()
            board.set(":J1", Safety.MOTION)
            started = board.now()
            # The ramp is where the current spike is, so it is sampled first and
            # separately: waiting a second would measure the cruise, not the load.
            out.extend(_sample_voltage(board, f"ramp_dir{direction}", label, 4))
            board.pause(1.0)
            out.extend(_sample_voltage(board, f"run_dir{direction}", label, _LOAD_SAMPLES))
            while board.now() - started < _LOAD_RUN_S:
                board.pause(0.2)
            travelled = (board.position() - position_before) % 0x1000000
            stopped = board.stop_and_wait()
            status = board.status()
            LOGGER.info(
                "%s dir=%s: остановлена=%s, прошла %d отсчётов, статус %s",
                label, direction, stopped, min(travelled, 0x1000000 - travelled), status.raw,
            )
            out.extend(_sample_voltage(board, f"after_dir{direction}", label, 3))
            if not status.initialized:
                # §12.2: the flag went down without anybody sending `:E` — the
                # board reset under the load. Stop here rather than repeat it.
                out.append({"step": "board_reset_detected", "speed": label, "f1": status.as_dict()})
                LOGGER.critical("Флаг инициализации сброшен на %s — плата села. Фаза прервана", label)
                board.set(f":I1{revu24(PERIOD_1X)}")
                board.set(":G110")
                return
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")

    summary: dict[str, dict[str, Any]] = {}
    for row in out:
        if "battery_v" not in row:
            continue
        key = f"{row['phase']}/{row['speed']}"
        bucket = summary.setdefault(key, {"battery": [], "usb": []})
        bucket["battery"].append(row["battery_v"])
        bucket["usb"].append(row["usb_v"])
    for key, bucket in sorted(summary.items()):
        LOGGER.info(
            "%s: батарея %.2f..%.2f В, USB %.2f..%.2f В",
            key, min(bucket["battery"]), max(bucket["battery"]), min(bucket["usb"]), max(bucket["usb"]),
        )


# --------------------------------------------------------------------------- #
# ramp: what §10.4 measures when it says "speed = TMR_Freq / period"
# --------------------------------------------------------------------------- #

_RAMP_RUN_S = 5.0
_RAMP_POLL_S = 0.05


def phase_ramp(board: Board, out: list[dict[str, Any]]) -> None:
    """Acceleration, cruise and braking of *tracking* motion, densely sampled.

    §10.4 states the cruise rate and §10.3 the braking, but neither was measured
    with a clock against the position, so the conformance cases had to guess how
    much of a 3-second run is spent accelerating. This measures it.
    """
    board.set(":F1")
    for period, label in _LOAD_PERIODS[1:]:
        board.set(f":I1{revu24(period)}")
        board.set(":G110")
        samples: list[dict[str, Any]] = []
        start = board.position()
        started = board.now()
        board.set(":J1", Safety.MOTION)
        stop_sent_at: float | None = None
        while True:
            elapsed = board.now() - started
            position = board.position()
            status = board.status()
            samples.append(
                {
                    "at_s": round(elapsed, 4),
                    "delta": (position - start) % 0x1000000,
                    "f1": status.raw,
                }
            )
            if stop_sent_at is None and elapsed >= _RAMP_RUN_S:
                board.set(":K1", Safety.MOTION)
                stop_sent_at = board.now() - started
            if stop_sent_at is not None and not status.running:
                break
            if elapsed > _RAMP_RUN_S + 10:
                board.stop_and_wait()
                break
            board.pause(_RAMP_POLL_S)
        out.append({"step": "ramp", "speed": label, "period": period, "stop_sent_at_s": stop_sent_at, "samples": samples})
        cruise = [
            (b["delta"] - a["delta"]) / (b["at_s"] - a["at_s"])
            for a, b in itertools.pairwise(samples)
            if a["at_s"] > 2.0 and b["at_s"] < _RAMP_RUN_S
        ]
        LOGGER.info(
            "%s (период %d): установившаяся скорость %.0f шаг/с (формула %.0f), выбег после `:K1` %.2f с",
            label, period, sum(cruise) / len(cruise) if cruise else 0.0, TIMER_FREQ / period,
            samples[-1]["at_s"] - (stop_sent_at or 0.0),
        )
        board.stop_and_wait()
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")


# --------------------------------------------------------------------------- #
# coldboot: what the board looks like straight after power-on
# --------------------------------------------------------------------------- #


_IDENTITY_QUERIES = (
    ("e1_version", ":e1"),
    ("a1_cpr", ":a1"),
    ("b1_timer_freq", ":b1"),
    ("g1_highspeed_ratio", ":g1"),
    ("c1_brake_steps", ":c1"),
    ("D1_1x_period", ":D1"),
    ("s1_pec_period", ":s1"),
    ("q1_status_ex", ":q1010000"),
    ("q1_indexer", ":q1000000"),
)


def _identity(board: Board) -> dict[str, Any]:
    """Everything the board says about itself that a read cannot change.

    Tolerant on purpose: half of these are optional in the command set, and a
    board that answers `!0` to `:s1` has told us something worth writing down —
    it is not a reason to abort the snapshot.
    """
    row: dict[str, Any] = {}
    for name, payload in _IDENTITY_QUERIES:
        reply = board.ask(payload)
        row[name] = reply
        if reply.startswith("=") and len(reply) == 7:
            row[f"{name}_int"] = unrevu24(reply[1:])
    return row


def _snapshot(board: Board, label: str) -> dict[str, Any]:
    return {
        "step": label,
        "j1": board.position(),
        "h1": board.target(),
        "m1": board.brake_point(),
        "d1": board.tele_position(),
        "i1": board.period(),
        "f1": board.status().as_dict(),
        **board.voltages(),
    }


def phase_coldboot(board: Board, out: list[dict[str, Any]]) -> None:
    """The reference picture of a *quiet* power-on, for §7 reboot detection.

    Three reboots are on record, all of them under a GOTO ramp, where the axis
    coasted 448…1241 counts past the logical zero while the controller came up.
    What the same board looks like when it boots standing still has never been
    written down, and without it there is no way to tell how much of that offset
    is the reboot and how much is the coasting.

    Nothing here moves the axis and nothing here writes: the value of the
    snapshot is that it is taken *before* this session touches anything.
    """
    identity = _identity(board)
    out.append({"step": "identity", **identity})
    LOGGER.info("плата: %s", identity)

    for index in range(3):
        row = _snapshot(board, f"as_powered_on_{index}")
        out.append(row)
        LOGGER.info(
            "%s: j1=0x%06X h1=0x%06X m1=0x%06X i1=%d f1=%s батарея %.2f В, второй канал %.2f В",
            row["step"], row["j1"], row["h1"], row["m1"], row["i1"], row["f1"]["raw"],
            row["battery_v"], row["usb_v"],
        )
        board.pause(0.3)


# --------------------------------------------------------------------------- #
# gotoprobe: what actually sets the GOTO speed (no motion at all)
# --------------------------------------------------------------------------- #

# `references/skywatcher.cpp` `SlewTo()` sets the motion mode **first** and the
# step period **second**; `tools.ra_step2 goto` did it the other way round, and
# the board then ran a GOTO at ~5500 steps/s with `:I1` set to 145 steps/s. If
# `:G1` reloads the period, that alone explains "GOTO ignores `:I1`" — and it is
# answerable with reads, on a standing axis, for free.
_PROBE_MODES = ("0", "2", "1", "3", "8", "A")


def phase_gotoprobe(board: Board, out: list[dict[str, Any]]) -> None:
    board.set(":F1")
    out.append(_snapshot(board, "before_probe"))

    # 1. Does `:G1<mode>` overwrite the step period?
    for mode in _PROBE_MODES:
        board.set(f":I1{revu24(PERIOD_16X)}")
        before = board.period()
        reply = board.set(f":G1{mode}0")
        after = board.period()
        status = board.status().as_dict()
        row = {
            "step": "G_over_I",
            "mode": mode,
            "reply_G": reply,
            "period_before_G": before,
            "period_after_G": after,
            "clobbered": before != after,
            "f1": status,
        }
        out.append(row)
        LOGGER.info(
            "`:G1%s0` -> %r: период %d -> %d (%s), статус %s",
            mode, reply, before, after, "СБРОШЕН" if before != after else "сохранён", status["raw"],
        )

    # 2. Does `:I1` still take effect once the board is in GOTO mode?
    for mode in ("2", "0"):
        board.set(f":G1{mode}0")
        for period in (PERIOD_1X, PERIOD_16X, PERIOD_64X):
            reply = board.set(f":I1{revu24(period)}")
            read_back = board.period()
            row = {
                "step": "I_after_G",
                "mode": mode,
                "period_requested": period,
                "reply_I": reply,
                "period_read_back": read_back,
                "accepted": read_back == period,
            }
            out.append(row)
            LOGGER.info(
                "goto `:G1%s0`, затем `:I1` %d -> %r, `:i1` = %d (%s)",
                mode, period, reply, read_back, "принят" if read_back == period else "НЕ принят",
            )

    # 3. `:M1` against `:c1`: which of the two the brake point follows, with the
    #    target set the way a real GOTO sets it (§2.2 only ever read `:M1` with
    #    the target left at the logical zero).
    board.set(":G120")
    for distance, brake in ((1_000, 200), (20_000, 200), (20_000, 3_500), (20_000, 16_980)):
        board.set(f":H1{revu24(distance)}")
        target_before = board.target()
        brake_before = board.brake_point()
        reply = board.set(f":M1{revu24(brake)}")
        row = {
            "step": "M_vs_c",
            "distance": distance,
            "brake_requested": brake,
            "reply_M": reply,
            "target": target_before,
            "m1_before_M": brake_before,
            "m1_after_M": board.brake_point(),
            "target_minus_m1_before": (target_before - brake_before) % 0x1000000,
        }
        out.append(row)
        LOGGER.info(
            "`:H1` %d, `:M1` %d: h1=0x%06X, m1 %06X -> %06X, цель−m1 = %d",
            distance, brake, row["target"], row["m1_before_M"], row["m1_after_M"],
            row["target_minus_m1_before"],
        )

    # Back to the power-on shape of §12.1.
    board.set(f":H1{revu24(0)}")
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")
    out.append(_snapshot(board, "after_probe"))


# --------------------------------------------------------------------------- #
# gotoladder: the gentlest GOTO the board will accept, then one knob at a time
# --------------------------------------------------------------------------- #

# The board is not to be knocked over more than this in one session, whatever
# the ladder still has left to try.
SESSION_REBOOT_LIMIT = 3


@dataclasses.dataclass(frozen=True)
class Rung:
    steps: int
    mode: str
    direction: str
    # None: do not touch `:I1` at all — the vendor sequence never does (§6 of
    # `RA_SA_CONSOLE_PROTOCOL.md`). A number: write it, in `period_order`.
    period: int | None
    period_order: str  # "after_G" (skywatcher.cpp) or "before_G" (the old runs)
    brake: int | None
    label: str


def _ladder() -> list[Rung]:
    """Every rung differs from the one above it in exactly one knob.

    The first rung is **20 000** steps, not the 1 000 of the runs that fell, and
    that is the whole point of the order. `phase_gotoprobe` showed `:M1` to be
    inert at every value: the brake point is *always* the target minus `:c1` =
    16 980, so any GOTO shorter than 16 980 steps has its brake point behind its
    own starting position. All three recorded crashes were 1 000-step moves,
    i.e. every one of them was that degenerate case. A ladder that started there
    again would spend the whole reboot budget re-measuring what is already
    known; starting past `:c1` buys the one fact nobody has — whether GOTO works
    at all when its geometry makes sense.
    """
    return [
        # 1. Sane geometry (arm > `:c1`), slow-goto mode, the slowest period the
        #    board takes, written the way `references/skywatcher.cpp` writes it
        #    (mode first, period second), INDI's brake for a short move.
        Rung(20_000, "2", "0", PERIOD_1X, "after_G", 200, "20 000 шагов (> `:c1`), `:G120`, период 1×, `:M1`=200"),
        # 2. Only the direction changes.
        Rung(20_000, "2", "1", PERIOD_1X, "after_G", 200, "то же, CCW"),
        # 3. Only the period changes.
        Rung(20_000, "2", "0", PERIOD_16X, "after_G", 200, "20 000 шагов, период 16×"),
        # 4. Only the period changes.
        Rung(20_000, "2", "0", PERIOD_64X, "after_G", 200, "20 000 шагов, период 64×"),
        # 5. Only the brake increment changes — INDI's high-speed value.
        Rung(20_000, "2", "0", PERIOD_64X, "after_G", 3_200, "20 000 шагов, период 64×, `:M1`=3200"),
        # 6. Only the mode changes: slow goto -> fast goto.
        Rung(20_000, "0", "0", PERIOD_64X, "after_G", 3_200, "20 000 шагов, быстрый goto `:G100`"),
        # 7. Only the arm grows.
        Rung(100_000, "0", "0", PERIOD_64X, "after_G", 3_200, "100 000 шагов, быстрый goto"),
        # 8. Only the arm shrinks — below `:c1`, back into the degenerate
        #    geometry, but with everything else as gentle as rung 1.
        Rung(1_000, "2", "0", PERIOD_1X, "after_G", 200, "1 000 шагов (< `:c1`), всё прочее как в 1"),
        # 9. The run that fell three times out of three, byte for byte: the
        #    vendor order of `RA_SA_CONSOLE_PROTOCOL.md` §6 with the period
        #    written *before* `:G1` and the vendor's constant 3500 brake. Last,
        #    because it is the known bad one.
        Rung(1_000, "2", "0", PERIOD_1X, "before_G", VENDOR_BRAKE_INCREMENT, "прошлый сценарий целиком (период до `:G1`, `:M1`=3500)"),
    ]


def _voltage_row(board: Board, at_s: float, where: str) -> dict[str, Any]:
    return {"at_s": round(at_s, 3), "where": where, **board.voltages()}


def _signed(delta: int) -> int:
    return delta if delta < 0x800000 else delta - 0x1000000


def _run_rung(board: Board, rung: Rung) -> dict[str, Any]:
    # What the requested period would take, if the board honoured it, doubled
    # and capped: a GOTO that overshoots this is abandoned with `:K1` rather
    # than waited out. Capped because the board has ignored `:I1` in GOTO
    # before, and a 1x period over a long arm would otherwise mean minutes.
    if rung.period is None:
        expected_s = rung.steps / 500
    else:
        expected_s = rung.steps * rung.period / TIMER_FREQ
    deadline_s = min(45.0, max(8.0, expected_s * 2 + 5.0))
    row: dict[str, Any] = {
        "step": "rung",
        "label": rung.label,
        "steps": rung.steps,
        "mode": rung.mode,
        "direction": rung.direction,
        "period": rung.period,
        "period_order": rung.period_order,
        "brake": rung.brake,
    }
    volts: list[dict[str, Any]] = []

    if rung.period is not None and rung.period_order == "before_G":
        board.set(f":I1{revu24(rung.period)}")
    row["status_before"] = board.status().as_dict()
    row["voltage_before"] = board.voltages()
    start_position = board.position()
    row["position_before"] = start_position

    row["reply_G"] = board.set(f":G1{rung.mode}{rung.direction}")
    if rung.period is not None and rung.period_order == "after_G":
        row["reply_I"] = board.set(f":I1{revu24(rung.period)}")
    row["period_at_start"] = board.period()
    row["reply_H"] = board.set(f":H1{revu24(rung.steps)}")
    row["target"] = board.target()
    if rung.brake is not None:
        row["reply_M"] = board.set(f":M1{revu24(rung.brake)}")
    row["brake_point"] = board.brake_point()

    started = board.now()
    volts.append(_voltage_row(board, 0.0, "before_J"))
    row["reply_J"] = board.set(":J1", Safety.MOTION)

    profile: list[dict[str, Any]] = []
    stopped_samples = 0
    polls = 0
    outcome = "arrived"
    while True:
        elapsed = board.now() - started
        status = board.status()
        position = board.position()
        profile.append(
            {
                "at_s": round(elapsed, 4),
                "delta": _signed((position - start_position) % 0x1000000),
                "f1": status.raw,
                "running": status.running,
            }
        )
        polls += 1
        # Both channels every sixth poll: often enough to catch a sag that lasts
        # a tenth of a second, rare enough not to blind the position trace (a
        # voltage sample costs four round trips, a position sample one).
        if polls % 6 == 0:
            volts.append(_voltage_row(board, elapsed, "moving"))
        stopped_samples = 0 if status.running else stopped_samples + 1
        if stopped_samples >= 2 and elapsed > 0.3:
            break
        if elapsed > deadline_s:
            outcome = "deadline"
            LOGGER.error("плечо «%s» не уложилось в дедлайн — торможу", rung.label)
            board.stop_and_wait()
            break
        # A pause on *every* iteration, not only after the first second: the
        # virtual clock of `src/sim` moves on sleeps and on read timeouts alone,
        # so a loop that only ever exchanges never advances it and never ends.
        # On the wire 2 ms is below the round-trip time anyway.
        board.pause(0.05 if elapsed > 1.0 else 0.002)
    row["outcome"] = outcome
    row["elapsed_s"] = round(board.now() - started, 3)
    row["profile"] = profile

    board.pause(0.3)
    final = board.position()
    signed = _signed((final - start_position) % 0x1000000)
    wanted = rung.steps if rung.direction == "0" else -rung.steps
    row["position_after"] = final
    row["travelled"] = signed
    row["error_steps"] = signed - wanted
    row["target_after"] = board.target()
    row["period_after"] = board.period()
    row["status_after"] = board.status().as_dict()
    volts.append(_voltage_row(board, board.now() - started, "after"))
    row["voltages"] = volts
    row["battery_min_v"] = min(sample["battery_v"] for sample in volts)
    row["second_min_v"] = min(sample["usb_v"] for sample in volts)

    peak = 0.0
    for first, second in itertools.pairwise(profile):
        span = second["at_s"] - first["at_s"]
        if span > 0:
            peak = max(peak, abs(second["delta"] - first["delta"]) / span)
    row["peak_speed_sps"] = round(peak)

    LOGGER.info(
        "плечо «%s»: прошло %+d из %+d (ошибка %+d) за %.2f с, пик %d шаг/с, %s; "
        "батарея %.2f В мин, второй канал %.2f В мин, статус %s",
        rung.label, signed, wanted, row["error_steps"], row["elapsed_s"], row["peak_speed_sps"],
        outcome, row["battery_min_v"], row["second_min_v"], row["status_after"]["raw"],
    )
    return row


def _looks_rebooted(board: Board, row: dict[str, Any]) -> str | None:
    """Why this leg reads as a reboot, or None. Nobody in this phase sends `:E`."""
    status = board.status()
    if not status.initialized:
        return "флаг инициализации сброшен"
    if status.tracking and row["mode"] in {"0", "2"} and row["outcome"] != "arrived":
        return "режим вернулся к трекингу, хотя ход не завершился"
    if row["target_after"] == POSITION_OFFSET and row["target"] != POSITION_OFFSET:
        return "цель `:h1` вернулась к 0x800000"
    if row["period"] is not None and row["period_after"] != row["period_at_start"]:
        return f"период сам вернулся к {row['period_after']}"
    return None


def phase_gotoladder(board: Board, out: list[dict[str, Any]]) -> None:
    reboots = 0
    board.set(":F1")
    for rung in _ladder():
        if not board.status().initialized:
            board.set(":F1")
        try:
            row = _run_rung(board, rung)
        except BoardSatDown as error:
            reboots += 1
            out.append({"step": "board_sat_down", "label": rung.label, "error": str(error), "reboots": reboots})
            LOGGER.critical("ПЛАТА ЗАМОЛЧАЛА на плече «%s»: %s. Лестница остановлена", rung.label, error)
            break
        reason = _looks_rebooted(board, row)
        row["reboot_reason"] = reason
        out.append(row)
        board.stop_and_wait()
        if reason is not None:
            reboots += 1
            LOGGER.critical(
                "ПЕРЕЗАГРУЗКА на плече «%s» (%s). Перезагрузок за сессию: %d. Лестница остановлена",
                rung.label, reason, reboots,
            )
            out.append({"step": "board_reset_detected", "label": rung.label, "reason": reason, "reboots": reboots})
            break
        if reboots >= SESSION_REBOOT_LIMIT:
            break
    board.set(f":H1{revu24(0)}")
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")
    out.append(_snapshot(board, "after_ladder"))
    out.append({"step": "reboots", "count": reboots})


# --------------------------------------------------------------------------- #
# gotobrake: a GOTO that is never allowed to arrive
# --------------------------------------------------------------------------- #

# (arm, where to send `:K1`). The first pair stops the axis far from the
# target, the second one close to it: if only the second falls, what kills the
# board is *being near the target*; if neither falls, it is the board's own
# arrival, and a GOTO cut short by `:K1` is a usable move.
_BRAKE_RUNS = ((30_000, 12_000), (30_000, 28_000))


def phase_gotobrake(board: Board, out: list[dict[str, Any]]) -> None:
    """Does the board survive a GOTO it is not allowed to finish?

    Four recorded crashes, and every one of them landed within ~200 steps of the
    target: 1 000-step moves died at 0.06…0.22 s (arrival due at ~0.25 s), and
    the 20 000-step move of the ladder died at 19 792 of 20 000 after three
    seconds of healthy running. The acceleration is not the suspect any more —
    it tops out at ~8 600 steps/s over a full second and the board is fine
    throughout — the *arrival* is.

    This is the experiment that separates the two, and it is the cheap direction
    to be wrong in: if arrival is the killer, the board is not knocked over at
    all here.
    """
    board.set(":F1")
    out.append(_snapshot(board, "before_brake_runs"))

    for index, (steps, stop_at) in enumerate(_BRAKE_RUNS):
        row: dict[str, Any] = {"step": "brake_run", "steps": steps, "stop_at": stop_at}
        board.set(":G120")
        board.set(f":I1{revu24(PERIOD_1X)}")
        start_position = board.position()
        board.set(f":H1{revu24(steps)}")
        board.set(f":M1{revu24(200)}")
        row["position_before"] = start_position
        row["target"] = board.target()
        row["brake_point"] = board.brake_point()
        row["voltage_before"] = board.voltages()

        profile: list[dict[str, Any]] = []
        volts: list[dict[str, Any]] = [_voltage_row(board, 0.0, "before_J")]
        started = board.now()
        board.set(":J1", Safety.MOTION)
        stop_sent_at: float | None = None
        polls = 0
        while True:
            elapsed = board.now() - started
            status = board.status()
            delta = _signed((board.position() - start_position) % 0x1000000)
            profile.append({"at_s": round(elapsed, 4), "delta": delta, "f1": status.raw})
            polls += 1
            if polls % 6 == 0:
                volts.append(_voltage_row(board, elapsed, "moving"))
            if stop_sent_at is None and abs(delta) >= stop_at:
                volts.append(_voltage_row(board, elapsed, "at_K"))
                board.set(":K1", Safety.MOTION)
                stop_sent_at = elapsed
            if stop_sent_at is not None and not status.running:
                break
            if elapsed > 30.0:
                LOGGER.error("прогон %d не уложился в дедлайн — торможу", index)
                board.stop_and_wait()
                break
            board.pause(0.05 if elapsed > 1.0 else 0.002)

        row["stop_sent_at_s"] = stop_sent_at
        row["elapsed_s"] = round(board.now() - started, 3)
        row["profile"] = profile
        board.pause(0.5)
        row["position_after"] = board.position()
        row["travelled"] = _signed((row["position_after"] - start_position) % 0x1000000)
        row["target_after"] = board.target()
        row["status_after"] = board.status().as_dict()
        volts.append(_voltage_row(board, board.now() - started, "after"))
        row["voltages"] = volts
        row["battery_min_v"] = min(sample["battery_v"] for sample in volts)
        row["second_min_v"] = min(sample["usb_v"] for sample in volts)
        # The initialization flag alone, on purpose: nothing in this phase sends
        # `:E`, so on this board only a restart can clear it. `:h1` is not usable
        # here — `:K1` moves it by itself — and the voltage window recovers from
        # its post-restart zeros within a second, i.e. before the sample below.
        row["target_reset"] = row["target_after"] != row["target"]
        row["survived"] = bool(row["status_after"]["init"])
        out.append(row)
        LOGGER.info(
            "прогон %d: `:K1` на %d из %d, прошло %+d, статус %s, цель %s, "
            "батарея %.2f В мин, второй канал %.2f В мин — %s",
            index, stop_at, steps, row["travelled"], row["status_after"]["raw"],
            "цела" if row["target_after"] == row["target"] else "сброшена",
            row["battery_min_v"], row["second_min_v"],
            "ПЛАТА ЖИВА" if row["survived"] else "ПЕРЕЗАГРУЗКА",
        )
        board.stop_and_wait()
        if not row["survived"]:
            out.append({"step": "board_reset_detected", "at_run": index})
            LOGGER.critical("Прогон %d уронил плату — фаза остановлена", index)
            break

    board.set(f":H1{revu24(0)}")
    board.set(f":I1{revu24(PERIOD_1X)}")
    board.set(":G110")
    out.append(_snapshot(board, "after_brake_runs"))


PHASES = {
    "probe": phase_probe,
    "goto": phase_goto,
    "load": phase_load,
    "ramp": phase_ramp,
    "coldboot": phase_coldboot,
    "gotoprobe": phase_gotoprobe,
    "gotoladder": phase_gotoladder,
    "gotobrake": phase_gotobrake,
}


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #


def _stamp() -> str:
    return datetime.datetime.now(datetime.UTC).strftime("%Y%m%dT%H%M%SZ")


def _run_clamp(port: str, baud: int, timeout_s: float, trace: Path) -> list[dict[str, Any]]:
    """The driver's own minimum-period measurement, on the live board.

    Imported here rather than at module scope: this is the only phase that
    speaks through the driver instead of through the guarded wire.
    """
    from skywatcher.codec import Command
    from skywatcher.session import SkyWatcherSession

    recorder = JsonlRecorder(trace)
    line = SerialLine(
        port=port, baud=baud, timeout_s=timeout_s, name="ra", terminator="\r", clock=REAL_CLOCK, recorder=recorder
    )
    session = SkyWatcherSession(line, REAL_CLOCK)
    rows: list[dict[str, Any]] = []
    try:
        line.connect()
        before = unrevu24(line.query(":i1\r").strip("=\r"))
        line.close(reason="clamp_probe_reopen")
        board = session.open()
        after = unrevu24(session.transact(Command.INQUIRE_STEP_PERIOD, None))
        rows.append(
            {
                "period_before_connect": before,
                "min_period_measured": board.min_period,
                "period_after_connect": after,
                "restored": before == after,
                "board": dataclasses.asdict(board),
            }
        )
        LOGGER.info(
            "период до подключения %s, драйвер измерил предел %s, после подключения %s (восстановлен: %s)",
            before, board.min_period, after, before == after,
        )
    finally:
        session.close()
        recorder.close()
    return rows


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="tools.ra_step2", description="Сценарные эксперименты шага 2 по плате RA.")
    parser.add_argument("phase", choices=[*PHASES, "clamp"])
    parser.add_argument("--port", default=None)
    parser.add_argument(
        "--sim",
        action="store_true",
        help="прогнать фазу по src/sim вместо железа: новый сценарий не должен впервые исполняться на плате",
    )
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--timeout", type=float, default=TIMEOUT_S)
    parser.add_argument("--out", default=None)
    parser.add_argument("--logs-root", default="logs")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    setup_logging(logs_root=args.logs_root)
    stamp = _stamp()
    trace = Path("logs/protocol") / f"ra_step2_{args.phase}_{stamp}.jsonl"
    trace.parent.mkdir(parents=True, exist_ok=True)
    out = Path(args.out) if args.out else Path("logs/protocol") / f"ra_step2_{args.phase}_{stamp}.json"

    rows: list[dict[str, Any]] = []
    if args.phase == "clamp":
        if args.port is None:
            raise SystemExit("фазе clamp нужен --port")
        rows = _run_clamp(args.port, int(args.baud), float(args.timeout), trace)
        out.write_text(json.dumps(rows, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
        LOGGER.info("результаты: %s, трасса: %s", out, trace)
        return 0

    recorder = JsonlRecorder(trace)
    line: SerialLine
    if args.sim:
        # Imported here, not at module scope: everything above this line has to
        # keep working on a machine that only has the hardware path installed.
        from sim import Clock as VirtualClock
        from sim import SimSerialLine, SkyWatcherSim

        clock: Any = VirtualClock()
        line = SimSerialLine(
            SkyWatcherSim(clock), clock, port="sim://ra", baud=int(args.baud),
            timeout_s=float(args.timeout), name="ra", terminator="\r", recorder=recorder,
        )
    else:
        if args.port is None:
            raise SystemExit("живому прогону нужен --port (или --sim)")
        clock = REAL_CLOCK
        line = SerialLine(
            port=args.port, baud=int(args.baud), timeout_s=float(args.timeout), name="ra",
            terminator="\r", clock=REAL_CLOCK, recorder=recorder,
        )
    wire = HardwareWire(line, clock, {Safety.READ, Safety.WRITE, Safety.MOTION})
    board = Board(wire)
    line.connect()
    try:
        PHASES[args.phase](board, rows)
    except BoardSatDown as error:
        rows.append({"step": "board_sat_down", "error": str(error)})
        LOGGER.critical("ПЛАТА ПЕРЕСТАЛА ОТВЕЧАТЬ: %s. Фаза прервана, ось торможу", error)
    finally:
        if not stop_axis(wire):
            LOGGER.critical("ОСЬ МОЖЕТ ПРОДОЛЖАТЬ ДВИГАТЬСЯ: остановить не удалось, снимайте питание")
        out.write_text(json.dumps(rows, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
        line.close(reason="ra_step2_finished")
        recorder.close()
        LOGGER.info("результаты: %s, трасса: %s", out, trace)
    return 0


if __name__ == "__main__":
    sys.exit(main())
