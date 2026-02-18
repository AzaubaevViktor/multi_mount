from contextlib import contextmanager
from dataclasses import dataclass
import logging
import time

import pytest
from lx200.base import LX200Dec, LX200Handler
from lx200.protocols import LX200Ha
from lx200.splitter import LX200Splitter
from serial_wrapper.wrapper import SerialLine
from skywatcher.skywatcher import Axis, SkyWatcherMount
from skywatcher.skywatcher_lx200 import SkyWatcherLX200
from tmc2209.tmc2209_adapter import TMC2209Adapter
from tmc2209.tmc2209_lx200 import TMC2209LX200


@dataclass
class Position:
    mount: float
    motor: float


@dataclass
class AxisInfo:
    start: Position
    end: Position
    delta: Position
    delay_s: float
    rate_per_s: Position
    tracking_rate_tick_per_s: float


@dataclass
class Deltas:
    ra: AxisInfo
    dec: AxisInfo


class SplitterController:
    # TODO: Make more common interface
    def __init__(
        self,
        splitter: LX200Splitter,
        ra: SkyWatcherLX200,
        dec: TMC2209LX200,
    ) -> None:
        self._splitter = splitter
        self.ra = ra
        self.dec = dec

        self.logger = logging.getLogger("check")

    # LX200 COMMANDS

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
    
    # Raw data from mounts

    def _get_motor_position(self) -> tuple[float, float]:
        return self._splitter.motor_position()

    def _get_mount_position(self):
        return self.ra._mount_position_raw, self.dec._mount_position_raw
    
    def _get_tracking_rates(self):
        return self.ra._get_default_tracking_speed() * self.ra._current_track_rate_coef, \
                self.dec._get_default_tracking_speed() * self.dec._current_track_rate_coef
    
    @contextmanager
    def _with_position_locks(self):
        self.logger.debug("Wait for position locks...")
        start = time.monotonic()
        with self.ra._position_update_lock:
            with self.dec._position_update_lock:
                self.logger.debug("Position locks aquired by %.3fs", time.monotonic() - start)
                yield

    def get_deltas(self, delay_s: float) -> Deltas:
        with self._with_position_locks():
            real_start = time.monotonic()
            start_mount = self._get_mount_position()
            start_motor = self._get_motor_position()
            start_tracking_rates = self._get_tracking_rates()

        time.sleep(delay_s)

        with self._with_position_locks():
            real_end = time.monotonic()
            end_mount = self._get_mount_position()
            end_motor = self._get_motor_position()
            end_tracking_rates = self._get_tracking_rates()

        real_delay_s = real_end - real_start

        return Deltas(
            ra=AxisInfo(
                start=Position(
                    mount=start_mount[0],
                    motor=start_motor[0],
                ),
                end=Position(
                    mount=end_mount[0],
                    motor=end_motor[0],
                ),
                delta=Position(
                    mount=end_mount[0] - start_mount[0],
                    motor=end_motor[0] - start_motor[0],
                ),
                delay_s=real_delay_s,
                rate_per_s=Position(
                    mount=(end_mount[0] - start_mount[0]) / real_delay_s,
                    motor=(end_motor[0] - start_motor[0]) / real_delay_s,
                ),
                tracking_rate_tick_per_s=start_tracking_rates[0],
            ),
            dec=AxisInfo(
                start=Position(
                    mount=start_mount[1],
                    motor=start_motor[1],
                ),
                end=Position(
                    mount=end_mount[1],
                    motor=end_motor[1],
                ),
                delta=Position(
                    mount=end_mount[1] - start_mount[1],
                    motor=end_motor[1] - start_motor[1],
                ),
                delay_s=real_delay_s,
                rate_per_s=Position(
                    mount=(end_mount[1] - start_mount[1]) / real_delay_s,
                    motor=(end_motor[1] - start_motor[1]) / real_delay_s,
                ),
                tracking_rate_tick_per_s=start_tracking_rates[1],
            ),
        )

    # WAITS AND CHECKS

    TRACKING_MODE_TOLERANCE = (.1, .1)
    """ (s, arcsec)/s """
    TRACKING_MODE_MOTOR_TOLERANCE = (0.1, 0.1)
    """ (ticks, ticks)/s """

    def wait_while_mount_in_tracking(self, timeout_s: float = 5., times: int = 20):
        self.logger.warning("\n==== WAIT WHILE MOUNT IN TRACKING MODE ====")
        start = time.monotonic()
        while True:
            deltas = self.get_deltas((timeout_s * .9) / 10)
            if (abs(deltas.ra.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[0]) and \
                (abs(deltas.dec.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[1]):
                self.logger.warning("\n==== MOUNT STABILIZES IN TRACKING MODE ====")
                self.logger.warning("After %.3fs", time.monotonic() - start)
                self.logger.debug("%s", deltas)
                return True
            
            if time.monotonic() - start > timeout_s:
                break
        self.logger.warning("\n==== MOUNT NOT STABILIZES ====")
        self.logger.warning("For %.3fs (real %.3fs)", timeout_s, time.monotonic() - start)
        
        self.logger.warning("%s", deltas)

        pytest.fail(f"Mount not stablizes in {time.monotonic() - start:.3f}s")

    def check_mount_in_tracking_mode(self, delta_s: float = 2.):
        self.logger.warning("\n==== CHECK MOUNT AND MOTOR IN TRACKING MODE ====")
        deltas = self.get_deltas(delta_s)

        self.logger.debug("\nDELTAS: %s", deltas)

        assert deltas.ra.tracking_rate_tick_per_s > 0
        assert deltas.dec.tracking_rate_tick_per_s == 0

        assert (abs(deltas.ra.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[0]) and \
                (abs(deltas.dec.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[1])

        self.logger.warning(
            "\n==== MOUNT IN TRACKING MODE ====\n" 
            "RA: |%.5f| < %.2f\n"
            "DEC: |%.5f| < %.2f",
            deltas.ra.rate_per_s.mount, self.TRACKING_MODE_TOLERANCE[0],
            deltas.dec.rate_per_s.mount, self.TRACKING_MODE_TOLERANCE[1],
        )

        # TODO: Calculate from motor tracking rate ticks
        assert (abs(deltas.ra.rate_per_s.motor - deltas.ra.tracking_rate_tick_per_s) < self.TRACKING_MODE_MOTOR_TOLERANCE[0]) and \
                (abs(deltas.dec.rate_per_s.mount) < self.TRACKING_MODE_MOTOR_TOLERANCE[1])

        self.logger.warning(
            "\n==== MOTOR IN TRACKING MODE ====\n" 
            "RA: |%.5f - %.5f| = %.5f < %.2f\n"
            "DEC: |%.5f| < %.2f",
            deltas.ra.rate_per_s.motor, deltas.ra.tracking_rate_tick_per_s,
            abs(deltas.ra.rate_per_s.motor - deltas.ra.tracking_rate_tick_per_s), self.TRACKING_MODE_MOTOR_TOLERANCE[0],
            deltas.dec.rate_per_s.motor, self.TRACKING_MODE_MOTOR_TOLERANCE[1],
        )

        return True


fixture_logger = logging.getLogger("fixtures")

SW_PORT_PATTERN = "PL2303G"
SW_BAUD = 115200
SW_TIMEOUT_S = 0.05
SW_SERIAL_NAME = "sw"

DEC_PORT_PATTERN = "tty.usbserial"
DEC_BAUD = 115200
DEC_TIMEOUT_S = 2.0
DEC_SERIAL_NAME = "tmc"
DEC_TERMINATOR = "\n"

SETTLE_S = 0.5

@pytest.fixture(scope='module')
def sc():

    fixture_logger.warning("\n===== CONNECTING HARDWARE =====")
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

    fixture_logger.warning("\n===== HARDWARE CONNECTED =====")

    sc = SplitterController(
        splitter=splitter,
        ra=sw_lx200,
        dec=dec_lx200,
    )

    try:
        yield sc
    finally:
        fixture_logger.warning("\n==== STOP AND DISCONNECT HARDWARE ====\n")
        try:
            sc.halt_all()
        except Exception:
            fixture_logger.exception("While halt all")
        time.sleep(SETTLE_S)
        del sc
        splitter.stop()

        del splitter
        del sw_lx200
        del dec_lx200

        try:
            sw_serial.close()
        except Exception:
            fixture_logger.exception("While sw serial disconnect")

        try:
            dec_adapter.close()
        except Exception:
            fixture_logger.exception("While dec adapter disconnect")

        fixture_logger.warning("\n==== HARDWARE DISCONNECTED ====\n")


@pytest.fixture(autouse=True)
def _ensure_halted(sc: SplitterController):
    fixture_logger.warning(
        "\n===== TURN MOUNT INTO TRACKING MODE =====\n" \
        "TEST STARTS\n"
    )
    try:
        sc.halt_all()
    except Exception:
        fixture_logger.exception("While dec adapter disconnect")

    sc.wait_while_mount_in_tracking(timeout_s=5.)

    yield

    fixture_logger.warning(
        "\n===== TURN MOUNT INTO TRACKING MODE =====\n" \
        "TEST ENDS\n"
    )
    try:
        sc.halt_all()
    except Exception:
        fixture_logger.exception("While dec adapter disconnect")
    
    sc.wait_while_mount_in_tracking(timeout_s=5.)


def test_mount_in_tracking_mode_by_default(sc: SplitterController):
    assert sc.check_mount_in_tracking_mode(delta_s=5.)
