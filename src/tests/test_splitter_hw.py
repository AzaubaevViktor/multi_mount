import time
from collections.abc import Iterator

import pytest

from lx200.protocols import LX200Dec, LX200Ha
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher_lx200 import SkyWatcherLX200, SkyWatcherMount
from tmc2209.tmc2209_adapter import TMC2209Adapter
from tmc2209.tmc2209_lx200 import TMC2209LX200


SW_PORT_PATTERN = "PL2303G"
SW_BAUD = 115200
SW_TIMEOUT_S = 0.05
SW_SERIAL_NAME = "sw"

DEC_PORT_PATTERN = "tty.usbserial"
DEC_BAUD = 115200
DEC_TIMEOUT_S = 2.0
DEC_SERIAL_NAME = "tmc"
DEC_TERMINATOR = "\n"

POLL_INTERVAL_S = 0.2
SETTLE_S = 0.5

SYNC_RA_TOLERANCE_S = 8.0
SYNC_DEC_TOLERANCE_DEG = 0.3
SLEW_RA_TOLERANCE_S = 20.0
SLEW_DEC_TOLERANCE_DEG = 0.8

MANUAL_MOVE_DURATION_S = 2.0
RA_MANUAL_MIN_DELTA_S = 2.0
DEC_MANUAL_MIN_DELTA_DEG = 0.5

GUIDE_SETTLE_EXTRA_S = 1.2
RA_GUIDE_MARGIN_S = 0.8
DEC_GUIDE_MARGIN_DEG = 0.2
GUIDE_PULSE_MS_VALUES = (2500, 5000)


def _cmd(splitter: LX200Splitter, command: str):
    return splitter.handle(command)


def _set_target_ra(splitter: LX200Splitter, value: LX200Ha) -> None:
    assert _cmd(splitter, f"Sr{value}") is True


def _set_target_dec(splitter: LX200Splitter, value: LX200Dec) -> None:
    assert _cmd(splitter, f"Sd{value}") is True


def _sync(splitter: LX200Splitter) -> None:
    assert _cmd(splitter, "CM") == "OK"


def _set_slew_to_find(splitter: LX200Splitter) -> None:
    assert _cmd(splitter, "RM") is None


def _slew(splitter: LX200Splitter) -> None:
    _cmd(splitter, "MS")


def _halt_all(splitter: LX200Splitter) -> None:
    assert _cmd(splitter, "Q") is None


def _guide(splitter: LX200Splitter, direction: str, ms: int) -> None:
    assert _cmd(splitter, f"Mg{direction}{ms}") is None


def _get_ra(splitter: LX200Splitter) -> LX200Ha:
    response = _cmd(splitter, "GR")
    assert isinstance(response, LX200Ha)
    return response


def _get_dec(splitter: LX200Splitter) -> LX200Dec:
    response = _cmd(splitter, "GD")
    assert isinstance(response, LX200Dec)
    return response


def _ra_distance_seconds(a_seconds: float, b_seconds: float) -> float:
    circle_seconds = LX200Ha.SECONDS_PER_CIRCLE
    delta = abs(a_seconds - b_seconds)
    return min(delta, circle_seconds - delta)


def _signed_ra_delta_seconds(start_seconds: float, end_seconds: float) -> float:
    circle_seconds = LX200Ha.SECONDS_PER_CIRCLE
    half_circle_seconds = circle_seconds / 2
    return ((end_seconds - start_seconds + half_circle_seconds) % circle_seconds) - half_circle_seconds


def _wait_until(predicate, timeout_s: float, error_message: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(error_message)


def _wait_ra_close(
    splitter: LX200Splitter,
    target: LX200Ha,
    tolerance_s: float,
    timeout_s: float,
) -> None:
    target_seconds = target.to_seconds()
    last_ra = _get_ra(splitter).to_seconds()

    def _is_close() -> bool:
        nonlocal last_ra
        last_ra = _get_ra(splitter).to_seconds()
        return _ra_distance_seconds(last_ra, target_seconds) <= tolerance_s

    _wait_until(
        _is_close,
        timeout_s,
        (
            f"RA did not reach target in time: current={LX200Ha.from_seconds(last_ra)} "
            f"target={target} distance={_ra_distance_seconds(last_ra, target_seconds):.2f}s"
        ),
    )


def _wait_dec_close(
    splitter: LX200Splitter,
    target: LX200Dec,
    tolerance_deg: float,
    timeout_s: float,
) -> None:
    target_deg = target.to_degrees()
    last_dec = _get_dec(splitter).to_degrees()

    def _is_close() -> bool:
        nonlocal last_dec
        last_dec = _get_dec(splitter).to_degrees()
        return abs(last_dec - target_deg) <= tolerance_deg

    _wait_until(
        _is_close,
        timeout_s,
        (
            f"DEC did not reach target in time: current={LX200Dec.from_degrees(last_dec)} "
            f"target={target} distance={abs(last_dec - target_deg):.3f}deg"
        ),
    )


def _sync_known_position(splitter: LX200Splitter, ra: LX200Ha, dec: LX200Dec) -> None:
    _halt_all(splitter)
    time.sleep(SETTLE_S)
    _set_target_ra(splitter, ra)
    _set_target_dec(splitter, dec)
    _sync(splitter)
    _wait_ra_close(splitter, ra, SYNC_RA_TOLERANCE_S, 10.0)
    _wait_dec_close(splitter, dec, SYNC_DEC_TOLERANCE_DEG, 10.0)


def _wait_ra_moved(
    splitter: LX200Splitter,
    start_seconds: float,
    expected_sign: int,
    min_delta_s: float,
    timeout_s: float,
) -> float:
    last_ra = start_seconds
    last_delta = 0.0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        last_ra = _get_ra(splitter).to_seconds()
        last_delta = _signed_ra_delta_seconds(start_seconds, last_ra)
        if expected_sign > 0 and last_delta >= min_delta_s:
            return last_delta
        if expected_sign < 0 and last_delta <= -min_delta_s:
            return last_delta
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        "RA did not move in expected direction: "
        f"start={LX200Ha.from_seconds(start_seconds)} current={LX200Ha.from_seconds(last_ra)} "
        f"delta={last_delta:.2f}s expected_sign={expected_sign}"
    )


def _wait_dec_moved(
    splitter: LX200Splitter,
    start_deg: float,
    expected_sign: int,
    min_delta_deg: float,
    timeout_s: float,
) -> float:
    last_dec = start_deg
    last_delta = 0.0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        last_dec = _get_dec(splitter).to_degrees()
        last_delta = last_dec - start_deg
        if expected_sign > 0 and last_delta >= min_delta_deg:
            return last_delta
        if expected_sign < 0 and last_delta <= -min_delta_deg:
            return last_delta
        time.sleep(POLL_INTERVAL_S)
    pytest.fail(
        "DEC did not move in expected direction: "
        f"start={LX200Dec.from_degrees(start_deg)} current={LX200Dec.from_degrees(last_dec)} "
        f"delta={last_delta:.3f}deg expected_sign={expected_sign}"
    )


def _measure_ra_delta(splitter: LX200Splitter, duration_s: float) -> float:
    start = _get_ra(splitter).to_seconds()
    time.sleep(duration_s)
    end = _get_ra(splitter).to_seconds()
    return _signed_ra_delta_seconds(start, end)


def _measure_dec_delta(splitter: LX200Splitter, duration_s: float) -> float:
    start = _get_dec(splitter).to_degrees()
    time.sleep(duration_s)
    end = _get_dec(splitter).to_degrees()
    return end - start


@pytest.fixture(scope="module")
def splitter() -> Iterator[LX200Splitter]:
    sw_path = SerialLine.search(SW_PORT_PATTERN)
    sw_serial = SerialLine(sw_path, SW_BAUD, SW_TIMEOUT_S, SW_SERIAL_NAME)
    sw_mount = SkyWatcherMount(sw_serial)
    sw_lx200 = SkyWatcherLX200(sw_mount)

    dec_path = SerialLine.search(DEC_PORT_PATTERN)
    dec_serial = SerialLine(dec_path, DEC_BAUD, DEC_TIMEOUT_S, DEC_SERIAL_NAME, terminator=DEC_TERMINATOR)
    dec_adapter = TMC2209Adapter(dec_serial)
    dec_lx200 = TMC2209LX200(dec_adapter)

    splitter = LX200Splitter(ra=sw_lx200, dec=dec_lx200)
    splitter.connect()

    try:
        yield splitter
    finally:
        try:
            _halt_all(splitter)
        except Exception:
            pass

        splitter._thread_work = False
        if splitter._guide_thread and splitter._guide_thread.is_alive():
            splitter._guide_thread.join(timeout=2.0)

        sw_lx200._working = False
        if sw_lx200._check_ra_thread and sw_lx200._check_ra_thread.is_alive():
            sw_lx200._check_ra_thread.join(timeout=2.0)
        if sw_lx200._check_goto_thread and sw_lx200._check_goto_thread.is_alive():
            sw_lx200._check_goto_thread.join(timeout=2.0)

        try:
            sw_serial.close()
        except Exception:
            pass

        try:
            dec_adapter.close()
        except Exception:
            pass


@pytest.fixture(autouse=True)
def _ensure_halted(splitter: LX200Splitter) -> Iterator[None]:
    try:
        _halt_all(splitter)
    except Exception:
        pass
    time.sleep(SETTLE_S)
    yield
    try:
        _halt_all(splitter)
    except Exception:
        pass
    time.sleep(SETTLE_S)


def test_hw_splitter_sync_ra_dec_multiple_times(splitter: LX200Splitter) -> None:
    points = (
        ("11:58:00", "+20*00:00"),
        ("12:00:30", "+35*30:00"),
        ("12:03:15", "+49*45:00"),
    )

    for ra_text, dec_text in points:
        target_ra = LX200Ha.from_string(ra_text)
        target_dec = LX200Dec.from_string(dec_text)

        _set_target_ra(splitter, target_ra)
        _set_target_dec(splitter, target_dec)
        _sync(splitter)

        _wait_ra_close(splitter, target_ra, SYNC_RA_TOLERANCE_S, 10.0)
        _wait_dec_close(splitter, target_dec, SYNC_DEC_TOLERANCE_DEG, 10.0)


def test_hw_splitter_manual_move_all_directions(splitter: LX200Splitter) -> None:
    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )

    start_ra = _get_ra(splitter).to_seconds()
    assert _cmd(splitter, "Me") is None
    _wait_ra_moved(splitter, start_ra, expected_sign=1, min_delta_s=RA_MANUAL_MIN_DELTA_S, timeout_s=8.0)
    time.sleep(MANUAL_MOVE_DURATION_S)
    assert _cmd(splitter, "Qe") is None
    time.sleep(SETTLE_S)
    ra_after_east = _get_ra(splitter).to_seconds()
    assert _signed_ra_delta_seconds(start_ra, ra_after_east) > RA_MANUAL_MIN_DELTA_S

    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )
    start_ra = _get_ra(splitter).to_seconds()
    assert _cmd(splitter, "Mw") is None
    _wait_ra_moved(splitter, start_ra, expected_sign=-1, min_delta_s=RA_MANUAL_MIN_DELTA_S, timeout_s=8.0)
    time.sleep(MANUAL_MOVE_DURATION_S)
    assert _cmd(splitter, "Qw") is None
    time.sleep(SETTLE_S)
    ra_after_west = _get_ra(splitter).to_seconds()
    assert _signed_ra_delta_seconds(start_ra, ra_after_west) < -RA_MANUAL_MIN_DELTA_S

    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )
    start_dec = _get_dec(splitter).to_degrees()
    assert _cmd(splitter, "Mn") is None
    _wait_dec_moved(
        splitter,
        start_dec,
        expected_sign=1,
        min_delta_deg=DEC_MANUAL_MIN_DELTA_DEG,
        timeout_s=8.0,
    )
    time.sleep(MANUAL_MOVE_DURATION_S)
    assert _cmd(splitter, "Qn") is None
    time.sleep(SETTLE_S)
    dec_after_north = _get_dec(splitter).to_degrees()
    assert (dec_after_north - start_dec) > DEC_MANUAL_MIN_DELTA_DEG

    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )
    start_dec = _get_dec(splitter).to_degrees()
    assert _cmd(splitter, "Ms") is None
    _wait_dec_moved(
        splitter,
        start_dec,
        expected_sign=-1,
        min_delta_deg=DEC_MANUAL_MIN_DELTA_DEG,
        timeout_s=8.0,
    )
    time.sleep(MANUAL_MOVE_DURATION_S)
    assert _cmd(splitter, "Qs") is None
    time.sleep(SETTLE_S)
    dec_after_south = _get_dec(splitter).to_degrees()
    assert (dec_after_south - start_dec) < -DEC_MANUAL_MIN_DELTA_DEG


def test_hw_splitter_slew_to_target_reaches_goal(splitter: LX200Splitter) -> None:
    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+25*00:00"),
    )

    start_ra = _get_ra(splitter).to_seconds()
    start_dec = _get_dec(splitter).to_degrees()

    target_ra = LX200Ha.from_seconds(start_ra + 180)
    target_dec = LX200Dec.from_degrees(start_dec + 3.0)

    _set_slew_to_find(splitter)
    _set_target_ra(splitter, target_ra)
    _set_target_dec(splitter, target_dec)
    _slew(splitter)

    _wait_ra_moved(splitter, start_ra, expected_sign=1, min_delta_s=2.0, timeout_s=10.0)
    _wait_dec_moved(splitter, start_dec, expected_sign=1, min_delta_deg=0.2, timeout_s=10.0)

    _wait_ra_close(splitter, target_ra, SLEW_RA_TOLERANCE_S, 30.0)
    _wait_dec_close(splitter, target_dec, SLEW_DEC_TOLERANCE_DEG, 30.0)


def test_hw_splitter_slew_halt_all_stops_early(splitter: LX200Splitter) -> None:
    _sync_known_position(
        splitter,
        LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+20*00:00"),
    )

    start_ra = _get_ra(splitter).to_seconds()
    start_dec = _get_dec(splitter).to_degrees()

    target_ra = LX200Ha.from_seconds(start_ra + 1200)
    target_dec = LX200Dec.from_degrees(start_dec + 20.0)

    _set_slew_to_find(splitter)
    _set_target_ra(splitter, target_ra)
    _set_target_dec(splitter, target_dec)
    _slew(splitter)

    _wait_ra_moved(splitter, start_ra, expected_sign=1, min_delta_s=2.0, timeout_s=10.0)
    _wait_dec_moved(splitter, start_dec, expected_sign=1, min_delta_deg=0.2, timeout_s=10.0)

    time.sleep(1.0)
    _halt_all(splitter)
    time.sleep(SETTLE_S)

    stopped_ra = _get_ra(splitter).to_seconds()
    stopped_dec = _get_dec(splitter).to_degrees()

    assert _ra_distance_seconds(stopped_ra, target_ra.to_seconds()) > 60.0
    assert abs(stopped_dec - target_dec.to_degrees()) > 2.0


def test_hw_splitter_guiding_pulses_all_directions_vs_tracking(splitter: LX200Splitter) -> None:
    # TODO: Move shared hardware-measurement helpers into a reusable utility module if more hw suites appear.
    directions = {
        "e": ("ra", 1),
        "w": ("ra", -1),
        "n": ("dec", 1),
        "s": ("dec", -1),
    }
    by_direction: dict[str, dict[int, float]] = {direction: {} for direction in directions}

    for direction, (axis, expected_sign) in directions.items():
        for pulse_ms in GUIDE_PULSE_MS_VALUES:
            _sync_known_position(
                splitter,
                LX200Ha.from_string("12:00:00"),
                LX200Dec.from_string("+35*00:00"),
            )

            duration_s = pulse_ms / 1000.0 + GUIDE_SETTLE_EXTRA_S
            if axis == "ra":
                baseline_delta = _measure_ra_delta(splitter, duration_s)
            else:
                baseline_delta = _measure_dec_delta(splitter, duration_s)

            _sync_known_position(
                splitter,
                LX200Ha.from_string("12:00:00"),
                LX200Dec.from_string("+35*00:00"),
            )

            if axis == "ra":
                start_value = _get_ra(splitter).to_seconds()
            else:
                start_value = _get_dec(splitter).to_degrees()

            _guide(splitter, direction, pulse_ms)
            time.sleep(duration_s)

            if axis == "ra":
                current_value = _get_ra(splitter).to_seconds()
                guide_delta = _signed_ra_delta_seconds(start_value, current_value)
                assert guide_delta * expected_sign > RA_GUIDE_MARGIN_S, (
                    f"RA guide pulse did not move in expected direction: "
                    f"direction={direction} pulse_ms={pulse_ms} delta={guide_delta:.2f}s"
                )
                assert abs(guide_delta) > abs(baseline_delta) + RA_GUIDE_MARGIN_S, (
                    f"RA guide pulse is too close to plain tracking: "
                    f"direction={direction} pulse_ms={pulse_ms} "
                    f"guided={guide_delta:.2f}s tracking={baseline_delta:.2f}s"
                )
            else:
                current_value = _get_dec(splitter).to_degrees()
                guide_delta = current_value - start_value
                assert guide_delta * expected_sign > DEC_GUIDE_MARGIN_DEG, (
                    f"DEC guide pulse did not move in expected direction: "
                    f"direction={direction} pulse_ms={pulse_ms} delta={guide_delta:.3f}deg"
                )
                assert abs(guide_delta) > abs(baseline_delta) + DEC_GUIDE_MARGIN_DEG, (
                    f"DEC guide pulse is too close to plain tracking: "
                    f"direction={direction} pulse_ms={pulse_ms} "
                    f"guided={guide_delta:.3f}deg tracking={baseline_delta:.3f}deg"
                )

            by_direction[direction][pulse_ms] = abs(guide_delta)
            _halt_all(splitter)
            time.sleep(SETTLE_S)

    short_ms, long_ms = GUIDE_PULSE_MS_VALUES
    for direction in directions:
        short_abs_delta = by_direction[direction][short_ms]
        long_abs_delta = by_direction[direction][long_ms]
        assert long_abs_delta > short_abs_delta, (
            f"Long guide pulse should move further than short pulse: direction={direction} "
            f"{short_ms}ms={short_abs_delta:.3f}, {long_ms}ms={long_abs_delta:.3f}"
        )
