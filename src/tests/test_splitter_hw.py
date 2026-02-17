import logging
import time
from collections.abc import Iterator
from typing import Callable

import pytest

from lx200 import base
from lx200.protocols import LX200Dec, LX200Ha
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher_lx200 import SkyWatcherLX200, SkyWatcherMount
from tmc2209.tmc2209_adapter import Phase, TMC2209Adapter, TMC2209Status
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

MANUAL_MOVE_DURATION_S = 2.0
RA_MANUAL_MIN_DELTA_S = 2.0
DEC_MANUAL_MIN_DELTA_DEG = 0.1

GUIDE_SETTLE_EXTRA_S = 1.2
RA_GUIDE_MARGIN_S = 0.1
DEC_GUIDE_MARGIN_DEG = 0.2
RA_SHOWED_MARGIN_S = 1.1
DEC_SHOWED_MARGIN_S = 1.1
GUIDE_PULSE_MS_VALUES = (2500, 5000)
DEC_STOP_PHASES = {Phase.IDLE, Phase.HOLD}
STOP_CHECK_TIMEOUT_S = 12.0
RA_STOP_CHECK_WINDOW_S = 1.5
RA_STOP_MAX_DELTA_S = 5.0
DEC_STOP_CHECK_WINDOW_S = 1.5
DEC_STOP_MAX_DELTA_DEG = 0.1
SLEW_REACH_RA_TOLERANCE_S = 25.0
SLEW_REACH_DEC_TOLERANCE_DEG = 1.0


class SplitterController:
    def __init__(
        self,
        splitter: LX200Splitter,
        ra: SkyWatcherLX200,
        dec: TMC2209LX200,
    ) -> None:
        self._splitter = splitter
        self.ra = ra
        self.dec = dec

    def _cmd(self, command: str):
        return self._splitter.handle(command)

    def set_target_ra(self, value: LX200Ha) -> None:
        assert self._cmd(f"Sr{value}") is True

    def set_target_dec(self, value: LX200Dec) -> None:
        assert self._cmd(f"Sd{value}") is True

    def sync(self) -> None:
        assert self._cmd("CM") == "OK"

    def set_slew_to_find(self) -> None:
        assert self._cmd("RM") is None

    def slew(self) -> None:
        self._cmd("MS")

    def halt_all(self) -> None:
        assert self._cmd("Q") is None

    def move_east(self) -> None:
        assert self._cmd("Me") is None

    def move_west(self) -> None:
        assert self._cmd("Mw") is None

    def move_north(self) -> None:
        assert self._cmd("Mn") is None

    def move_south(self) -> None:
        assert self._cmd("Ms") is None

    def halt_east(self) -> None:
        assert self._cmd("Qe") is None

    def halt_west(self) -> None:
        assert self._cmd("Qw") is None

    def halt_north(self) -> None:
        assert self._cmd("Qn") is None

    def halt_south(self) -> None:
        assert self._cmd("Qs") is None

    def guide(self, direction: str, ms: int) -> None:
        assert self._cmd(f"Mg{direction}{ms}") is None

    def get_ra(self) -> LX200Ha:
        response = self._cmd("GR")
        assert isinstance(response, LX200Ha)
        return response

    def get_dec(self) -> LX200Dec:
        response = self._cmd("GD")
        assert isinstance(response, LX200Dec)
        return response
    
    def get_telescope_raw_position(self) -> tuple[float, float]:
        return self._splitter.get_telescope_raw_position()

    def is_ra_goto_active(self) -> bool:
        return self.ra._goto_to is not None

    def get_dec_status(self) -> TMC2209Status:
        return self.dec._adapter.status()

    def _wait_until(self, predicate: Callable[[], bool], timeout_s: float, error_message: str) -> None:
        logging.warning(
            "\n================ POLLING START ================\n"
            "TIMEOUT: %.1fs\n"
            "CONDITION: %s\n"
            "===============================================",
            timeout_s,
            error_message,
        )
        polling_start = time.monotonic()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                logging.warning(
                    "\n================ POLLING END ==================\n"
                    "RESULT: SUCCESS\n"
                    "ELAPSED: %.2fs\n"
                    "===============================================",
                    time.monotonic() - polling_start,
                )
                return
            time.sleep(POLL_INTERVAL_S)
        pytest.fail(error_message)

    def wait_ra_close(
        self,
        target: LX200Ha,
        tolerance_s: float,
        timeout_s: float,
    ) -> None:
        target_seconds = target.to_seconds()
        last_ra = self.get_ra().to_seconds()

        def _is_close() -> bool:
            nonlocal last_ra
            last_ra = self.get_ra().to_seconds()
            return _ra_distance_seconds(last_ra, target_seconds) <= tolerance_s

        self._wait_until(
            _is_close,
            timeout_s,
            (
                f"RA did not reach target in time: current={LX200Ha.from_seconds(last_ra)} "
                f"target={target} distance={_ra_distance_seconds(last_ra, target_seconds):.2f}s"
            ),
        )

    def wait_dec_close(
        self,
        target: LX200Dec,
        tolerance_deg: float,
        timeout_s: float,
    ) -> None:
        target_deg = target.to_degrees()
        last_dec = self.get_dec().to_degrees()

        def _is_close() -> bool:
            nonlocal last_dec
            last_dec = self.get_dec().to_degrees()
            return abs(last_dec - target_deg) <= tolerance_deg

        self._wait_until(
            _is_close,
            timeout_s,
            (
                f"DEC did not reach target in time: current={LX200Dec.from_degrees(last_dec)} "
                f"target={target} distance={abs(last_dec - target_deg):.3f}deg"
            ),
        )

    def sync_known_position(self, ra: LX200Ha, dec: LX200Dec) -> None:
        logging.warning(
            "\n================ SYNC START ===================\n"
            "TARGET RA: %s\n"
            "TARGET DEC: %s\n"
            "===============================================",
            ra,
            dec,
        )
        self.halt_all()
        self.assert_ra_explicitly_stopped()
        self.assert_dec_explicitly_stopped()
        time.sleep(SETTLE_S)
        self.set_target_ra(ra)
        self.set_target_dec(dec)
        self.sync()
        self.wait_ra_close(ra, SYNC_RA_TOLERANCE_S, 10.0)
        self.wait_dec_close(dec, SYNC_DEC_TOLERANCE_DEG, 10.0)
        logging.warning(
            "\n================ SYNC END =====================\n"
            "RA: %s\n"
            "DEC: %s\n"
            "===============================================",
            self.get_ra(),
            self.get_dec(),
        )

    def wait_ra_moved(
        self,
        start_seconds: float,
        expected_sign: int,
        min_delta_s: float,
        timeout_s: float,
    ) -> float:
        logging.warning(
            "\n================ POLLING START ================\n"
            "WAIT RA MOVED\n"
            "START: %s\n"
            "EXPECTED SIGN: %s\n"
            "MIN DELTA: %.2fs\n"
            "TIMEOUT: %.1fs\n"
            "===============================================",
            LX200Ha.from_seconds(start_seconds),
            expected_sign,
            min_delta_s,
            timeout_s,
        )
        polling_start = time.monotonic()
        last_ra = start_seconds
        last_delta = 0.0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            last_ra = self.get_ra().to_seconds()
            last_delta = _signed_ra_delta_seconds(start_seconds, last_ra)
            if expected_sign > 0 and last_delta >= min_delta_s:
                logging.warning(
                    "\n================ POLLING END ==================\n"
                    "WAIT RA MOVED DONE\n"
                    "CURRENT: %s\n"
                    "DELTA: %.2fs\n"
                    "ELAPSED: %.2fs\n"
                    "===============================================",
                    LX200Ha.from_seconds(last_ra),
                    last_delta,
                    time.monotonic() - polling_start,
                )
                return last_delta
            if expected_sign < 0 and last_delta <= -min_delta_s:
                logging.warning(
                    "\n================ POLLING END ==================\n"
                    "WAIT RA MOVED DONE\n"
                    "CURRENT: %s\n"
                    "DELTA: %.2fs\n"
                    "ELAPSED: %.2fs\n"
                    "===============================================",
                    LX200Ha.from_seconds(last_ra),
                    last_delta,
                    time.monotonic() - polling_start,
                )
                return last_delta
            time.sleep(POLL_INTERVAL_S)
        pytest.fail(
            "RA did not move in expected direction: "
            f"start={LX200Ha.from_seconds(start_seconds)} current={LX200Ha.from_seconds(last_ra)} "
            f"delta={last_delta:.2f}s expected_sign={expected_sign}"
        )

    def wait_dec_moved(
        self,
        start_deg: float,
        expected_sign: int,
        min_delta_deg: float,
        timeout_s: float,
    ) -> float:
        logging.warning(
            "\n================ POLLING START ================\n"
            "WAIT DEC MOVED\n"
            "START: %s\n"
            "EXPECTED SIGN: %s\n"
            "MIN DELTA: %.3fdeg\n"
            "TIMEOUT: %.1fs\n"
            "===============================================",
            LX200Dec.from_degrees(start_deg),
            expected_sign,
            min_delta_deg,
            timeout_s,
        )
        polling_start = time.monotonic()
        last_dec = start_deg
        last_delta = 0.0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            last_dec = self.get_dec().to_degrees()
            last_delta = last_dec - start_deg
            if expected_sign > 0 and last_delta >= min_delta_deg:
                logging.warning(
                    "\n================ POLLING END ==================\n"
                    "WAIT DEC MOVED DONE\n"
                    "CURRENT: %s\n"
                    "DELTA: %.3fdeg\n"
                    "ELAPSED: %.2fs\n"
                    "===============================================",
                    LX200Dec.from_degrees(last_dec),
                    last_delta,
                    time.monotonic() - polling_start,
                )
                return last_delta
            if expected_sign < 0 and last_delta <= -min_delta_deg:
                logging.warning(
                    "\n================ POLLING END ==================\n"
                    "WAIT DEC MOVED DONE\n"
                    "CURRENT: %s\n"
                    "DELTA: %.3fdeg\n"
                    "ELAPSED: %.2fs\n"
                    "===============================================",
                    LX200Dec.from_degrees(last_dec),
                    last_delta,
                    time.monotonic() - polling_start,
                )
                return last_delta
            time.sleep(POLL_INTERVAL_S)
        pytest.fail(
            "DEC did not move in expected direction: "
            f"start={LX200Dec.from_degrees(start_deg)} current={LX200Dec.from_degrees(last_dec)} "
            f"delta={last_delta:.3f}deg expected_sign={expected_sign}"
        )

    def measure_ra_delta(self, duration_s: float) -> float:
        start = self.get_ra().to_seconds()
        time.sleep(duration_s)
        end = self.get_ra().to_seconds()
        return _signed_ra_delta_seconds(start, end)

    def measure_dec_delta(self, duration_s: float) -> float:
        start = self.get_dec().to_degrees()
        time.sleep(duration_s)
        end = self.get_dec().to_degrees()
        return end - start

    def _wait_dec_stopped(self, timeout_s: float) -> None:
        last_status = self.get_dec_status()

        def _is_stopped() -> bool:
            nonlocal last_status
            last_status = self.get_dec_status()
            return last_status.phase in DEC_STOP_PHASES

        self._wait_until(
            _is_stopped,
            timeout_s,
            (
                "DEC did not stop in time: "
                f"phase={last_status.phase} target_set={last_status.target_set} "
                f"speed={last_status.actual_speed_sps:.1f}"
            ),
        )

    def assert_ra_explicitly_stopped(self) -> None:
        delta = self.measure_ra_delta(RA_STOP_CHECK_WINDOW_S)
        abs_delta = abs(delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: RA stop drift\n"
            "delta_s: %.6f\n"
            "abs_delta_s: %.6f\n"
            "limit_s: %.6f\n"
            "expr_abs_delta_le_limit: %s\n"
            "===============================================",
            delta,
            abs_delta,
            RA_STOP_MAX_DELTA_S,
            abs_delta <= RA_STOP_MAX_DELTA_S,
        )
        assert abs_delta <= RA_STOP_MAX_DELTA_S, (
            "RA still looks like active manual/goto motion after stop command: "
            f"delta={delta:.2f}s for {RA_STOP_CHECK_WINDOW_S:.1f}s"
        )

    def assert_dec_explicitly_stopped(self) -> None:
        self._wait_dec_stopped(STOP_CHECK_TIMEOUT_S)
        delta = self.measure_dec_delta(DEC_STOP_CHECK_WINDOW_S)
        abs_delta = abs(delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: DEC stop drift\n"
            "delta_deg: %.6f\n"
            "abs_delta_deg: %.6f\n"
            "limit_deg: %.6f\n"
            "expr_abs_delta_le_limit: %s\n"
            "===============================================",
            delta,
            abs_delta,
            DEC_STOP_MAX_DELTA_DEG,
            abs_delta <= DEC_STOP_MAX_DELTA_DEG,
        )
        assert abs_delta <= DEC_STOP_MAX_DELTA_DEG, (
            "DEC still changes after stop command: "
            f"delta={delta:.3f}deg for {DEC_STOP_CHECK_WINDOW_S:.1f}s"
        )

    def wait_slew_finished(
        self,
        has_ra_target: bool,
        has_dec_target: bool,
        timeout_s: float,
    ) -> None:
        last_dec = self.get_dec_status()

        def _is_finished() -> bool:
            nonlocal last_dec
            ra_done = True
            dec_done = True
            if has_ra_target:
                ra_done = not self.is_ra_goto_active()
            if has_dec_target:
                last_dec = self.get_dec_status()
                dec_done = (
                    last_dec.phase in DEC_STOP_PHASES
                    and not last_dec.target_set
                )
            return ra_done and dec_done

        self._wait_until(
            _is_finished,
            timeout_s,
            (
                "Slew did not finish in time: "
                f"ra_active={self.is_ra_goto_active()} "
                f"dec_phase={last_dec.phase} dec_target_set={last_dec.target_set}"
            ),
        )


def _ra_distance_seconds(a_seconds: float, b_seconds: float) -> float:
    circle_seconds = LX200Ha.SECONDS_PER_CIRCLE
    delta = abs(a_seconds - b_seconds)
    return min(delta, circle_seconds - delta)


def _signed_ra_delta_seconds(start_seconds: float, end_seconds: float) -> float:
    circle_seconds = LX200Ha.SECONDS_PER_CIRCLE
    half_circle_seconds = circle_seconds / 2
    return ((end_seconds - start_seconds + half_circle_seconds) % circle_seconds) - half_circle_seconds

@pytest.fixture(scope="module")
def sc() -> Iterator[SplitterController]:
    logging.warning(
        "\n================ FIXTURE START ================\n"
        "BUILD HW SPLITTER STAND\n"
        "==============================================="
    )
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
    logging.warning(
        "\n================ FIXTURE READY ================\n"
        "HW SPLITTER CONNECTED\n"
        "==============================================="
    )
    sc = SplitterController(
        splitter=splitter,
        ra=sw_lx200,
        dec=dec_lx200,
    )

    try:
        yield sc
    finally:
        logging.warning(
            "\n================ FIXTURE STOP =================\n"
            "STOP HW SPLITTER STAND\n"
            "==============================================="
        )
        try:
            sc.halt_all()
        except Exception:
            pass
        time.sleep(SETTLE_S)
        del sc
        splitter.stop()

        del splitter
        del sw_lx200
        del dec_lx200

        try:
            sw_serial.close()
        except Exception:
            pass

        try:
            dec_adapter.close()
        except Exception:
            pass
        logging.warning(
            "\n================ FIXTURE END ==================\n"
            "HW SPLITTER STAND RELEASED\n"
            "==============================================="
        )


@pytest.fixture(autouse=True)
def _ensure_halted(sc: SplitterController) -> Iterator[None]:
    logging.warning(
        "\n================ TEST PREP START ==============\n"
        "ENSURE BOTH AXES HALTED\n"
        "==============================================="
    )
    try:
        sc.halt_all()
    except Exception:
        pass
    time.sleep(SETTLE_S)
    logging.warning(
        "\n================ TEST PREP END ================\n"
        "AXES HALTED BEFORE TEST\n"
        "==============================================="
    )
    yield
    logging.warning(
        "\n================ TEST CLEANUP START ===========\n"
        "HALT AFTER TEST\n"
        "==============================================="
    )
    try:
        sc.halt_all()
    except Exception:
        pass
    time.sleep(SETTLE_S)
    logging.warning(
        "\n================ TEST CLEANUP END =============\n"
        "HALT AFTER TEST DONE\n"
        "==============================================="
    )


def test_hw_splitter_sync_ra_dec_multiple_times(sc: SplitterController) -> None:
    logging.warning(
        "\n================ TEST START ===================\n"
        "SYNC RA/DEC MULTIPLE TIMES\n"
        "==============================================="
    )
    points = (
        ("11:58:00", "+20*00:00"),
        ("12:00:30", "+35*30:00"),
        ("12:03:15", "+49*45:00"),
    )

    for ra_text, dec_text in points:
        logging.warning(
            "\n================ SYNC POINT ===================\n"
            "RA: %s\n"
            "DEC: %s\n"
            "===============================================",
            ra_text,
            dec_text,
        )
        target_ra = LX200Ha.from_string(ra_text)
        target_dec = LX200Dec.from_string(dec_text)

        sc.set_target_ra(target_ra)
        sc.set_target_dec(target_dec)
        sc.sync()

        sc.wait_ra_close(target_ra, SYNC_RA_TOLERANCE_S, 10.0)
        sc.wait_dec_close(target_dec, SYNC_DEC_TOLERANCE_DEG, 10.0)
    logging.warning(
        "\n================ TEST END =====================\n"
        "SYNC RA/DEC MULTIPLE TIMES COMPLETE\n"
        "==============================================="
    )


@pytest.mark.parametrize(
    ("move_command", "halt_command", "axis", "expected_sign"),
    (
        pytest.param("move_east", "halt_east", "ra", -1, id="east"),
        pytest.param("move_west", "halt_west", "ra", 1, id="west"),
        pytest.param("move_north", "halt_north", "dec", 1, id="north"),
        pytest.param("move_south", "halt_south", "dec", -1, id="south"),
    ),
)
def test_hw_splitter_manual_move_all_directions(
    sc: SplitterController,
    move_command: str,
    halt_command: str,
    axis: str,
    expected_sign: int,
) -> None:
    logging.warning(
        "\n================ TEST START ===================\n"
        "MANUAL MOVE ALL DIRECTIONS\n"
        "AXIS: %s\n"
        "MOVE: %s\n"
        "HALT: %s\n"
        "===============================================",
        axis,
        move_command,
        halt_command,
    )
    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )

    if axis == "ra":
        start_value = sc.get_ra().to_seconds()
    else:
        start_value = sc.get_dec().to_degrees()

    logging.warning(
        "\n================ MANUAL START =================\n"
        "COMMAND: %s\n"
        "===============================================",
        move_command,
    )
    getattr(sc, move_command)()

    if axis == "ra":
        sc.wait_ra_moved(start_value,
            expected_sign=expected_sign,
            min_delta_s=RA_MANUAL_MIN_DELTA_S,
            timeout_s=8.0,
        )
    else:
        sc.wait_dec_moved(start_value,
            expected_sign=expected_sign,
            min_delta_deg=DEC_MANUAL_MIN_DELTA_DEG,
            timeout_s=8.0,
        )

    time.sleep(MANUAL_MOVE_DURATION_S)
    logging.warning(
        "\n================ MANUAL HALT ==================\n"
        "COMMAND: %s\n"
        "===============================================",
        halt_command,
    )
    getattr(sc, halt_command)()

    if axis == "ra":
        sc.assert_ra_explicitly_stopped()
    else:
        sc.assert_dec_explicitly_stopped()

    time.sleep(SETTLE_S)
    if axis == "ra":
        final_value = sc.get_ra().to_seconds()
        delta = _signed_ra_delta_seconds(start_value, final_value)
        signed_delta = delta * expected_sign
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: manual RA direction and min delta\n"
            "delta_s: %.6f\n"
            "expected_sign: %d\n"
            "signed_delta_s: %.6f\n"
            "min_delta_s: %.6f\n"
            "expr_signed_delta_gt_min: %s\n"
            "===============================================",
            delta,
            expected_sign,
            signed_delta,
            RA_MANUAL_MIN_DELTA_S,
            signed_delta > RA_MANUAL_MIN_DELTA_S,
        )
        assert signed_delta > RA_MANUAL_MIN_DELTA_S, (
            "RA manual move did not reach min delta in expected direction: "
            f"delta={delta:.2f}s expected_sign={expected_sign}"
        )
    else:
        final_value = sc.get_dec().to_degrees()
        delta = final_value - start_value
        signed_delta = delta * expected_sign
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: manual DEC direction and min delta\n"
            "delta_deg: %.6f\n"
            "expected_sign: %d\n"
            "signed_delta_deg: %.6f\n"
            "min_delta_deg: %.6f\n"
            "expr_signed_delta_gt_min: %s\n"
            "===============================================",
            delta,
            expected_sign,
            signed_delta,
            DEC_MANUAL_MIN_DELTA_DEG,
            signed_delta > DEC_MANUAL_MIN_DELTA_DEG,
        )
        assert signed_delta > DEC_MANUAL_MIN_DELTA_DEG, (
            "DEC manual move did not reach min delta in expected direction: "
            f"delta={delta:.3f}deg expected_sign={expected_sign}"
        )
    logging.warning(
        "\n================ TEST END =====================\n"
        "MANUAL MOVE ALL DIRECTIONS COMPLETE\n"
        "==============================================="
    )


@pytest.mark.parametrize(
    ("ra_delta_s", "dec_delta_deg"),
    (
        pytest.param(500.0, 0.0, id="ra_plus"),  # TODO: Add more variatives at ra/dec
        pytest.param(-500.0, 0.0, id="ra_minus"),
        pytest.param(0.0, 3.0, id="dec_plus"),
        pytest.param(0.0, -3.0, id="dec_minus"),
        pytest.param(500.0, 3.0, id="ra_plus_dec_plus"),
        pytest.param(500.0, -3.0, id="ra_plus_dec_minus"),
        pytest.param(-500.0, 3.0, id="ra_minus_dec_plus"),
        pytest.param(-500.0, -3.0, id="ra_minus_dec_minus"),
    ),
)
def test_hw_splitter_slew_to_target_reaches_goal(
    sc: SplitterController,
    ra_delta_s: float,
    dec_delta_deg: float,
) -> None:
    logging.warning(
        "\n================ TEST START ===================\n"
        "SLEW TO TARGET REACHES GOAL\n"
        "RA DELTA: %.2fs\n"
        "DEC DELTA: %.3fdeg\n"
        "===============================================",
        ra_delta_s,
        dec_delta_deg,
    )
    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+25*00:00"),
    )

    start_ra = sc.get_ra().to_seconds()
    start_dec = sc.get_dec().to_degrees()

    target_ra = LX200Ha.from_seconds(start_ra + ra_delta_s)
    target_dec = LX200Dec.from_degrees(start_dec + dec_delta_deg)

    logging.warning(
        "\n================ SLEW START ===================\n"
        "TARGET RA: %s\n"
        "TARGET DEC: %s\n"
        "===============================================",
        target_ra,
        target_dec,
    )
    sc.set_slew_to_find()
    sc.set_target_ra(target_ra)
    sc.set_target_dec(target_dec)
    sc.slew()

    if ra_delta_s != 0:
        sc.wait_ra_moved(start_ra,
            expected_sign=1 if ra_delta_s > 0 else -1,
            min_delta_s=2.0,
            timeout_s=10.0,
        )
    if dec_delta_deg != 0:
        sc.wait_dec_moved(start_dec,
            expected_sign=1 if dec_delta_deg > 0 else -1,
            min_delta_deg=0.2,
            timeout_s=10.0,
        )

    sc.wait_slew_finished(has_ra_target=ra_delta_s != 0,
        has_dec_target=dec_delta_deg != 0,
        timeout_s=45.0,
    )
    logging.warning(
        "\n================ SLEW END =====================\n"
        "SLEW FINISHED, COLLECT FINAL POSITION\n"
        "==============================================="
    )

    final_ra = sc.get_ra().to_seconds()
    final_dec = sc.get_dec().to_degrees()

    if ra_delta_s != 0:
        actual_ra_delta = _signed_ra_delta_seconds(start_ra, final_ra)
        ra_direction_product = actual_ra_delta * ra_delta_s
        expected_ra_abs = abs(ra_delta_s)
        actual_ra_abs = abs(actual_ra_delta)
        min_allowed_ra_abs = expected_ra_abs - SLEW_REACH_RA_TOLERANCE_S
        max_allowed_ra_abs = expected_ra_abs + SLEW_REACH_RA_TOLERANCE_S
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew RA direction\n"
            "actual_ra_delta_s: %.6f\n"
            "requested_ra_delta_s: %.6f\n"
            "direction_product: %.6f\n"
            "expr_direction_product_gt_0: %s\n"
            "===============================================",
            actual_ra_delta,
            ra_delta_s,
            ra_direction_product,
            ra_direction_product > 0,
        )
        assert ra_direction_product > 0, (
            "RA slew moved in wrong direction: "
            f"actual_ra_delta={actual_ra_delta:.2f}s requested_ra_delta={ra_delta_s:.2f}s"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew RA lower bound\n"
            "actual_abs_delta_s: %.6f\n"
            "min_allowed_abs_delta_s: %.6f\n"
            "expr_actual_abs_ge_min_allowed: %s\n"
            "===============================================",
            actual_ra_abs,
            min_allowed_ra_abs,
            actual_ra_abs >= min_allowed_ra_abs,
        )
        assert actual_ra_abs >= min_allowed_ra_abs, (
            "RA slew undershoot: "
            f"actual={actual_ra_abs:.2f}s min_allowed={min_allowed_ra_abs:.2f}s"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew RA upper bound\n"
            "actual_abs_delta_s: %.6f\n"
            "max_allowed_abs_delta_s: %.6f\n"
            "expr_actual_abs_le_max_allowed: %s\n"
            "===============================================",
            actual_ra_abs,
            max_allowed_ra_abs,
            actual_ra_abs <= max_allowed_ra_abs,
        )
        assert actual_ra_abs <= max_allowed_ra_abs, (
            "RA slew overshoot: "
            f"actual={actual_ra_abs:.2f}s max_allowed={max_allowed_ra_abs:.2f}s"
        )

    if dec_delta_deg != 0:
        actual_dec_delta = final_dec - start_dec
        dec_direction_product = actual_dec_delta * dec_delta_deg
        expected_dec_abs = abs(dec_delta_deg)
        actual_dec_abs = abs(actual_dec_delta)
        min_allowed_dec_abs = expected_dec_abs - SLEW_REACH_DEC_TOLERANCE_DEG
        max_allowed_dec_abs = expected_dec_abs + SLEW_REACH_DEC_TOLERANCE_DEG
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew DEC direction\n"
            "actual_dec_delta_deg: %.6f\n"
            "requested_dec_delta_deg: %.6f\n"
            "direction_product: %.6f\n"
            "expr_direction_product_gt_0: %s\n"
            "===============================================",
            actual_dec_delta,
            dec_delta_deg,
            dec_direction_product,
            dec_direction_product > 0,
        )
        assert dec_direction_product > 0, (
            "DEC slew moved in wrong direction: "
            f"actual_dec_delta={actual_dec_delta:.3f}deg requested_dec_delta={dec_delta_deg:.3f}deg"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew DEC lower bound\n"
            "actual_abs_delta_deg: %.6f\n"
            "min_allowed_abs_delta_deg: %.6f\n"
            "expr_actual_abs_ge_min_allowed: %s\n"
            "===============================================",
            actual_dec_abs,
            min_allowed_dec_abs,
            actual_dec_abs >= min_allowed_dec_abs,
        )
        assert actual_dec_abs >= min_allowed_dec_abs, (
            "DEC slew undershoot: "
            f"actual={actual_dec_abs:.3f}deg min_allowed={min_allowed_dec_abs:.3f}deg"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: slew DEC upper bound\n"
            "actual_abs_delta_deg: %.6f\n"
            "max_allowed_abs_delta_deg: %.6f\n"
            "expr_actual_abs_le_max_allowed: %s\n"
            "===============================================",
            actual_dec_abs,
            max_allowed_dec_abs,
            actual_dec_abs <= max_allowed_dec_abs,
        )
        assert actual_dec_abs <= max_allowed_dec_abs, (
            "DEC slew overshoot: "
            f"actual={actual_dec_abs:.3f}deg max_allowed={max_allowed_dec_abs:.3f}deg"
        )
    logging.warning(
        "\n================ TEST END =====================\n"
        "SLEW TO TARGET REACHES GOAL COMPLETE\n"
        "==============================================="
    )


def test_hw_splitter_slew_halt_all_stops_early(sc: SplitterController) -> None:
    logging.warning(
        "\n================ TEST START ===================\n"
        "SLEW HALT_ALL STOPS EARLY\n"
        "==============================================="
    )
    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+20*00:00"),
    )

    start_ra = sc.get_ra().to_seconds()
    start_dec = sc.get_dec().to_degrees()

    target_ra = LX200Ha.from_seconds(start_ra + 1200)
    target_dec = LX200Dec.from_degrees(start_dec + 20.0)

    logging.warning(
        "\n================ SLEW START ===================\n"
        "TARGET RA: %s\n"
        "TARGET DEC: %s\n"
        "===============================================",
        target_ra,
        target_dec,
    )
    sc.set_slew_to_find()
    sc.set_target_ra(target_ra)
    sc.set_target_dec(target_dec)
    sc.slew()

    sc.wait_ra_moved(start_ra, expected_sign=1, min_delta_s=2.0, timeout_s=10.0)
    sc.wait_dec_moved(start_dec, expected_sign=1, min_delta_deg=0.2, timeout_s=10.0)

    time.sleep(1.0)
    logging.warning(
        "\n================ HALT START ===================\n"
        "CALL HALT_ALL DURING SLEW\n"
        "==============================================="
    )
    sc.halt_all()
    sc.assert_ra_explicitly_stopped()
    sc.assert_dec_explicitly_stopped()
    time.sleep(SETTLE_S)
    logging.warning(
        "\n================ HALT END =====================\n"
        "AXES STOPPED, CHECK DISTANCE TO TARGET\n"
        "==============================================="
    )

    stopped_ra = sc.get_ra().to_seconds()
    stopped_dec = sc.get_dec().to_degrees()

    remaining_ra_distance = _ra_distance_seconds(stopped_ra, target_ra.to_seconds())
    remaining_dec_distance = abs(stopped_dec - target_dec.to_degrees())
    logging.warning(
        "\n================ ASSERT INPUT =================\n"
        "CHECK: slew halt RA remains far from target\n"
        "remaining_ra_distance_s: %.6f\n"
        "min_remaining_ra_distance_s: %.6f\n"
        "expr_remaining_gt_min: %s\n"
        "===============================================",
        remaining_ra_distance,
        60.0,
        remaining_ra_distance > 60.0,
    )
    assert remaining_ra_distance > 60.0, (
        "RA got too close to target after halt_all: "
        f"remaining_distance={remaining_ra_distance:.2f}s"
    )
    logging.warning(
        "\n================ ASSERT INPUT =================\n"
        "CHECK: slew halt DEC remains far from target\n"
        "remaining_dec_distance_deg: %.6f\n"
        "min_remaining_dec_distance_deg: %.6f\n"
        "expr_remaining_gt_min: %s\n"
        "===============================================",
        remaining_dec_distance,
        2.0,
        remaining_dec_distance > 2.0,
    )
    assert remaining_dec_distance > 2.0, (
        "DEC got too close to target after halt_all: "
        f"remaining_distance={remaining_dec_distance:.3f}deg"
    )
    logging.warning(
        "\n================ TEST END =====================\n"
        "SLEW HALT_ALL STOPS EARLY COMPLETE\n"
        "==============================================="
    )


@pytest.mark.parametrize(("axis"), ("ra", "dec"))
def test_hw_splitter_baseline_delta(
    sc: SplitterController,
    axis: str,
):
    logging.warning(
        "\n================ TEST START ===================\n"
        "BASELINE DELTA\n"
        "AXIS: %s\n"
        "===============================================",
        axis,
    )
    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )

    duration_s = 5

    if axis == "ra":
        baseline_delta = sc.measure_ra_delta(duration_s)
        baseline_abs = abs(baseline_delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: baseline RA drift\n"
            "baseline_delta_s: %.6f\n"
            "baseline_abs_s: %.6f\n"
            "baseline_limit_s: %.6f\n"
            "expr_baseline_abs_lt_limit: %s\n"
            "===============================================",
            baseline_delta,
            baseline_abs,
            0.1,
            baseline_abs < 0.1,
        )
        assert baseline_abs < 0.1, (
            "RA baseline drift is too high: "
            f"drift={baseline_abs:.3f}s limit=0.1s"
        )
    else:
        baseline_delta = sc.measure_dec_delta(duration_s)
        baseline_abs = abs(baseline_delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: baseline DEC drift\n"
            "baseline_delta_deg: %.6f\n"
            "baseline_abs_deg: %.6f\n"
            "baseline_limit_deg: %.6f\n"
            "expr_baseline_abs_lt_limit: %s\n"
            "===============================================",
            baseline_delta,
            baseline_abs,
            0.1,
            baseline_abs < 0.1,
        )
        assert baseline_abs < 0.1, (
            "DEC baseline drift is too high: "
            f"drift={baseline_abs:.3f}deg limit=0.1deg"
        )
    logging.warning(
        "\n================ TEST END =====================\n"
        "BASELINE DELTA COMPLETE\n"
        "==============================================="
    )


@pytest.mark.parametrize(
    ("direction", "pulse_ms", "axis", "expected_sign"),
    (
        pytest.param("e", GUIDE_PULSE_MS_VALUES[0], "ra", -1, id="guide_e_2500"),
        pytest.param("e", GUIDE_PULSE_MS_VALUES[1], "ra", -1, id="guide_e_5000"),
        pytest.param("w", GUIDE_PULSE_MS_VALUES[0], "ra", 1, id="guide_w_2500"),
        pytest.param("w", GUIDE_PULSE_MS_VALUES[1], "ra", 1, id="guide_w_5000"),
        pytest.param("n", GUIDE_PULSE_MS_VALUES[0], "dec", 1, id="guide_n_2500"),
        pytest.param("n", GUIDE_PULSE_MS_VALUES[1], "dec", 1, id="guide_n_5000"),
        pytest.param("s", GUIDE_PULSE_MS_VALUES[0], "dec", -1, id="guide_s_2500"),
        pytest.param("s", GUIDE_PULSE_MS_VALUES[1], "dec", -1, id="guide_s_5000"),
    ),
)
def test_hw_splitter_guiding_pulses_all_directions_vs_tracking(
    sc: SplitterController,
    direction: str,
    pulse_ms: int,
    axis: str,
    expected_sign: int,
) -> None:
    logging.warning(
        "\n================ TEST START ===================\n"
        "GUIDING PULSES VS TRACKING\n"
        "DIRECTION: %s\n"
        "PULSE: %sms\n"
        "AXIS: %s\n"
        "===============================================",
        direction,
        pulse_ms,
        axis,
    )
    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )

    duration_s = pulse_ms / 1000.0 + GUIDE_SETTLE_EXTRA_S
    logging.warning(
        "\n================ GUIDE BASELINE START =========\n"
        "DURATION: %.2fs\n"
        "===============================================",
        duration_s,
    )
    baseline_raw_ra_start, baseline_raw_dec_start = sc.get_telescope_raw_position()
    time.sleep(duration_s)
    baseline_raw_ra_end, baseline_raw_dec_end = sc.get_telescope_raw_position()

    baseline_rate_ra = _signed_ra_delta_seconds(
        baseline_raw_ra_start,
        baseline_raw_ra_end,
    ) / duration_s
    baseline_rate_dec = (
        (baseline_raw_dec_end - baseline_raw_dec_start) / sc.dec.steps_per_degree
    ) / duration_s
    logging.warning(
        "\n================ GUIDE BASELINE END ===========\n"
        "BASELINE RAW RATE RA: %.3fs/s\n"
        "BASELINE RAW RATE DEC: %.3fdeg/s\n"
        "===============================================",
        baseline_rate_ra,
        baseline_rate_dec,
    )

    sc.sync_known_position(LX200Ha.from_string("12:00:00"),
        LX200Dec.from_string("+35*00:00"),
    )

    start_ra = sc.get_ra().to_seconds()
    start_dec = sc.get_dec().to_degrees()
    start_raw_ra, start_raw_dec = sc.get_telescope_raw_position()

    logging.warning(
        "\n================ GUIDE START ==================\n"
        "SEND GUIDE PULSE\n"
        "==============================================="
    )
    sc.guide(direction, pulse_ms)
    time.sleep(duration_s)

    current_ra = sc.get_ra().to_seconds()
    current_dec = sc.get_dec().to_degrees()
    current_raw_ra, current_raw_dec = sc.get_telescope_raw_position()

    guide_rate_ra = _signed_ra_delta_seconds(start_raw_ra, current_raw_ra) / duration_s
    guide_rate_dec = ((current_raw_dec - start_raw_dec) / sc.dec.steps_per_degree) / duration_s
    logging.warning(
        "\n================ GUIDE RESULT =================\n"
        "GUIDE RAW RATE RA: %.3fs/s\n"
        "GUIDE RAW RATE DEC: %.3fdeg/s\n"
        "===============================================",
        guide_rate_ra,
        guide_rate_dec,
    )

    if axis == "ra":
        rate_delta = guide_rate_ra - baseline_rate_ra
        signed_rate_delta = rate_delta * expected_sign
        abs_rate_delta = abs(rate_delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: guide RA direction vs tracking\n"
            "guide_rate_ra_s_per_s: %.6f\n"
            "baseline_rate_ra_s_per_s: %.6f\n"
            "expected_sign: %d\n"
            "signed_rate_delta_s_per_s: %.6f\n"
            "margin_s_per_s: %.6f\n"
            "expr_signed_rate_delta_gt_margin: %s\n"
            "===============================================",
            guide_rate_ra,
            baseline_rate_ra,
            expected_sign,
            signed_rate_delta,
            RA_GUIDE_MARGIN_S,
            signed_rate_delta > RA_GUIDE_MARGIN_S,
        )
        assert signed_rate_delta > RA_GUIDE_MARGIN_S, (
            f"RA guide pulse did not move in expected direction: "
            f"direction={direction} pulse_ms={pulse_ms} raw_rate={guide_rate_ra:.3f}s/s "
            f"baseline={baseline_rate_ra:.3f}s/s"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: guide RA differs from plain tracking\n"
            "rate_delta_s_per_s: %.6f\n"
            "abs_rate_delta_s_per_s: %.6f\n"
            "margin_s_per_s: %.6f\n"
            "expr_abs_rate_delta_gt_margin: %s\n"
            "===============================================",
            rate_delta,
            abs_rate_delta,
            RA_GUIDE_MARGIN_S,
            abs_rate_delta > RA_GUIDE_MARGIN_S,
        )
        assert abs_rate_delta > RA_GUIDE_MARGIN_S, (
            f"RA guide pulse is too close to plain tracking: "
            f"direction={direction} pulse_ms={pulse_ms} "
            f"guided={guide_rate_ra:.3f}s/s tracking={baseline_rate_ra:.3f}s/s "
            f"delta={guide_rate_ra - baseline_rate_ra:.3f}s/s"
        )
    else:
        rate_delta = guide_rate_dec - baseline_rate_dec
        signed_rate_delta = rate_delta * expected_sign
        abs_rate_delta = abs(rate_delta)
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: guide DEC direction vs tracking\n"
            "guide_rate_dec_deg_per_s: %.6f\n"
            "baseline_rate_dec_deg_per_s: %.6f\n"
            "expected_sign: %d\n"
            "signed_rate_delta_deg_per_s: %.6f\n"
            "margin_deg_per_s: %.6f\n"
            "expr_signed_rate_delta_gt_margin: %s\n"
            "===============================================",
            guide_rate_dec,
            baseline_rate_dec,
            expected_sign,
            signed_rate_delta,
            DEC_GUIDE_MARGIN_DEG,
            signed_rate_delta > DEC_GUIDE_MARGIN_DEG,
        )
        assert signed_rate_delta > DEC_GUIDE_MARGIN_DEG, (
            f"DEC guide pulse did not move in expected direction: "
            f"direction={direction} pulse_ms={pulse_ms} raw_rate={guide_rate_dec:.3f}deg/s "
            f"baseline={baseline_rate_dec:.3f}deg/s"
        )
        logging.warning(
            "\n================ ASSERT INPUT =================\n"
            "CHECK: guide DEC differs from plain tracking\n"
            "rate_delta_deg_per_s: %.6f\n"
            "abs_rate_delta_deg_per_s: %.6f\n"
            "margin_deg_per_s: %.6f\n"
            "expr_abs_rate_delta_gt_margin: %s\n"
            "===============================================",
            rate_delta,
            abs_rate_delta,
            DEC_GUIDE_MARGIN_DEG,
            abs_rate_delta > DEC_GUIDE_MARGIN_DEG,
        )
        assert abs_rate_delta > DEC_GUIDE_MARGIN_DEG, (
            f"DEC guide pulse is too close to plain tracking: "
            f"direction={direction} pulse_ms={pulse_ms} "
            f"guided={guide_rate_dec:.3f}deg/s tracking={baseline_rate_dec:.3f}deg/s "
            f"delta={guide_rate_dec - baseline_rate_dec:.3f}deg/s"
        )

    displayed_ra_delta = _signed_ra_delta_seconds(start_ra, current_ra)
    displayed_dec_delta = current_dec - start_dec
    displayed_ra_abs = abs(displayed_ra_delta)
    displayed_dec_abs = abs(displayed_dec_delta)
    logging.warning(
        "\n================ ASSERT INPUT =================\n"
        "CHECK: guide keeps displayed RA stable\n"
        "displayed_ra_delta_s: %.6f\n"
        "displayed_ra_abs_s: %.6f\n"
        "displayed_limit_s: %.6f\n"
        "expr_displayed_ra_abs_lt_limit: %s\n"
        "===============================================",
        displayed_ra_delta,
        displayed_ra_abs,
        RA_SHOWED_MARGIN_S,
        displayed_ra_abs < RA_SHOWED_MARGIN_S,
    )
    assert displayed_ra_abs < RA_SHOWED_MARGIN_S, (
        "Guide pulse changed displayed RA while raw telemetry moved: "
        f"direction={direction} pulse_ms={pulse_ms} displayed_ra_delta={displayed_ra_delta:.3f}s "
        f"raw_ra_rate={guide_rate_ra:.3f}s/s"
    )
    logging.warning(
        "\n================ ASSERT INPUT =================\n"
        "CHECK: guide keeps displayed DEC stable\n"
        "displayed_dec_delta_deg: %.6f\n"
        "displayed_dec_abs_deg: %.6f\n"
        "displayed_limit_deg: %.6f\n"
        "expr_displayed_dec_abs_lt_limit: %s\n"
        "===============================================",
        displayed_dec_delta,
        displayed_dec_abs,
        DEC_SHOWED_MARGIN_S,
        displayed_dec_abs < DEC_SHOWED_MARGIN_S,
    )
    assert displayed_dec_abs < DEC_SHOWED_MARGIN_S, (
        "Guide pulse changed displayed DEC while raw telemetry moved: "
        f"direction={direction} pulse_ms={pulse_ms} displayed_dec_delta={displayed_dec_delta:.3f}deg "
        f"raw_dec_rate={guide_rate_dec:.3f}deg/s"
    )

    logging.warning(
        "\n================ HALT START ===================\n"
        "GUIDE FINISHED, HALT ALL\n"
        "==============================================="
    )
    sc.halt_all()
    sc.assert_ra_explicitly_stopped()
    sc.assert_dec_explicitly_stopped()
    logging.warning(
        "\n================ TEST END =====================\n"
        "GUIDING PULSES VS TRACKING COMPLETE\n"
        "==============================================="
    )
