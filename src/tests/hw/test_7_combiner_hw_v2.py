from contextlib import contextmanager
from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable

import pytest
from serial_wrapper.wrapper import SerialLine
from sky.axis import AxisDEC, AxisMotionMode, AxisRA
from sky.combiner import Combiner
from sky.constants import STELLAR_SPEED
from sky.lx200 import SkyLX200
from sky.motor import MotionMode, MotorDirection
from sky.physics import Dec, DecPerSecond, Ha, SkyDirection
from sky.polar_compensator import PolarCompensator
from skywatcher.motor import SkyWatcherMotor
from tmc2209.motor import TMC2209Motor


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


@dataclass
class PolarSnapshot:
    status: str
    eps_n: float
    eps_e: float
    ra_sky_rate: float
    dec_sky_rate: float


SYNC_TIMEOUT_S = 10.0
SYNC_POLL_INTERVAL_S = 0.1


class CombinerController:
    def __init__(
        self,
        lx200: SkyLX200,
        combiner: Combiner,
    ) -> None:
        self._lx200 = lx200
        self._combiner = combiner
        self.ra = combiner.ra
        self.dec = combiner.dec

        self.logger = logging.getLogger("check")

    # LX200 COMMANDS

    def _cmd(self, command: str):
        return self._lx200.handle(command)

    def set_target_ra(self, value: Ha) -> None:
        assert self._cmd(f"Sr{value}") is True

    def set_target_dec(self, value: Dec) -> None:
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

    def get_ra(self) -> Ha:
        response = self._cmd("GR")
        assert isinstance(response, Ha)
        return response

    def get_dec(self) -> Dec:
        response = self._cmd("GD")
        assert isinstance(response, Dec)
        return response

    @staticmethod
    def ra_distance_seconds(a_seconds: float, b_seconds: float) -> float:
        circle_seconds = 24 * 60 * 60
        delta = abs(a_seconds - b_seconds)
        return min(delta, circle_seconds - delta)

    def sync_known_position(self, ra_text: str, dec_text: str) -> tuple[Ha, Dec]:
        target_ra = Ha.from_string(ra_text)
        target_dec = Dec.from_string(dec_text)
        self.set_target_ra(target_ra)
        self.set_target_dec(target_dec)
        self.sync()

        self.wait_until_target_reached(
            target_ra=target_ra,
            target_dec=target_dec,
            ra_tolerance_s=SYNC_RA_TOLERANCE_S,
            dec_tolerance_arcsec=SYNC_DEC_TOLERANCE_ARCSEC,
            timeout_s=SYNC_TIMEOUT_S,
            poll_interval_s=SYNC_POLL_INTERVAL_S,
        )

        synced_ra = float(self.get_ra())
        synced_dec = float(self.get_dec())

        assert self.ra_distance_seconds(synced_ra, float(target_ra)) <= SYNC_RA_TOLERANCE_S
        assert abs(synced_dec - float(target_dec)) <= SYNC_DEC_TOLERANCE_ARCSEC

        return target_ra, target_dec
    
    # Raw data from mounts

    def _get_motor_position(self) -> tuple[float, float]:
        with self.ra._motor_lock:
            ra_motor_position = self.ra._motor.convert_steps_to_position(self.ra._motor.status().steps)
        with self.dec._motor_lock:
            dec_motor_position = self.dec._motor.convert_steps_to_position(self.dec._motor.status().steps)
        return float(ra_motor_position), float(dec_motor_position)

    def _get_mount_position(self):
        position = self._combiner.get_position()
        return position.ra, position.dec
    
    def _get_tracking_rates(self):
        return self.ra._sky_speed, self.dec._sky_speed

    def polar_compensator(self) -> PolarCompensator:
        return self._combiner._polar_compensator

    def disable_polar_compensator(self) -> None:
        polar_compensator = self.polar_compensator()
        polar_compensator.eps_E = None
        polar_compensator.eps_N = None
        polar_compensator.ra_speed = STELLAR_SPEED
        polar_compensator.dec_speed = DecPerSecond(0)
        polar_compensator.stable_guide_ra_pulses_count = 0
        polar_compensator.stable_guide_dec_pulses_count = 0
        polar_compensator.last_guide_pulse = type(polar_compensator.last_guide_pulse)(0)
        self._combiner.set_sky_speed(STELLAR_SPEED, DecPerSecond(0), update_polar_compensator=False)

    def get_polar_snapshot(self) -> PolarSnapshot:
        polar_compensator = self.polar_compensator()
        return PolarSnapshot(
            status="guiding" if polar_compensator.eps_E is not None and polar_compensator.eps_N is not None else "disabled",
            eps_n=float(polar_compensator.eps_N or Dec(0)),
            eps_e=float(polar_compensator.eps_E or Ha(0)),
            ra_sky_rate=float(self.ra._sky_speed),
            dec_sky_rate=float(self.dec._sky_speed),
        )

    def wait_for_polar_condition(
        self,
        predicate: Callable[[PolarSnapshot], bool],
        timeout_s: float,
        description: str,
        poll_interval_s: float = 0.1,
    ) -> PolarSnapshot:
        start = time.monotonic()
        last_snapshot = self.get_polar_snapshot()
        while True:
            last_snapshot = self.get_polar_snapshot()
            if predicate(last_snapshot):
                self.logger.warning(
                    "\n==== POLAR CONDITION REACHED ====\n"
                    "DESCRIPTION: %s\n"
                    "AFTER: %.3fs\n"
                    "SNAPSHOT: %s\n",
                    description,
                    time.monotonic() - start,
                    last_snapshot,
                )
                return last_snapshot

            if time.monotonic() - start > timeout_s:
                break

            time.sleep(poll_interval_s)

        pytest.fail(
            f"Polar condition not reached in {timeout_s:.3f}s: {description}; "
            f"last_snapshot={last_snapshot}"
        )
    
    @contextmanager
    def _with_position_locks(self):
        self.logger.debug("Wait for position locks...")
        start = time.monotonic()
        count = 0
        while True:
            count += 1
            if count % 10 == 0:
                self.logger.warning("Try to get locks for %d times!", count)
            if count >= 1000:
                raise RuntimeError("Too much lock tryies: %d", count)
            
            locks: list[threading.RLock] = []
            try:
                if self.ra._motor_lock.acquire(timeout=.1):
                    locks.append(self.ra._motor_lock)
                else:
                    continue

                if self.dec._motor_lock.acquire(timeout=.1):
                    locks.append(self.dec._motor_lock)
                else:
                    continue
            
                self.logger.debug("Position locks aquired by %.3fs with %d try", time.monotonic() - start, count)

                yield

                break
            finally:
                for lock in locks:
                    lock.release()  
            

    def _capture_deltas_state(self):
        with self._with_position_locks():
            return (
                time.monotonic(),
                self._get_mount_position(),
                self._get_motor_position(),
                self._get_tracking_rates(),
            )

    def _build_deltas(self, start_state, end_state) -> Deltas:
        real_start, start_mount, start_motor, start_tracking_rates = start_state
        real_end, end_mount, end_motor, _end_tracking_rates = end_state
        real_delay_s = real_end - real_start
        start_mount_ra = float(start_mount[0])
        end_mount_ra = float(end_mount[0])
        ra_mount_delta = end_mount_ra - start_mount_ra
        if ra_mount_delta > 12 * 60 * 60:
            ra_mount_delta -= 24 * 60 * 60
        elif ra_mount_delta < -12 * 60 * 60:
            ra_mount_delta += 24 * 60 * 60

        start_motor_ra = float(start_motor[0])
        end_motor_ra = float(end_motor[0])
        ra_motor_delta = end_motor_ra - start_motor_ra
        if ra_motor_delta > 12 * 60 * 60:
            ra_motor_delta -= 24 * 60 * 60
        elif ra_motor_delta < -12 * 60 * 60:
            ra_motor_delta += 24 * 60 * 60

        start_mount_dec = float(start_mount[1])
        end_mount_dec = float(end_mount[1])
        dec_mount_delta = end_mount_dec - start_mount_dec

        start_motor_dec = float(start_motor[1])
        end_motor_dec = float(end_motor[1])
        dec_motor_delta = end_motor_dec - start_motor_dec

        return Deltas(
            ra=AxisInfo(
                start=Position(
                    mount=start_mount_ra,
                    motor=start_motor_ra,
                ),
                end=Position(
                    mount=end_mount_ra,
                    motor=end_motor_ra,
                ),
                delta=Position(
                    mount=ra_mount_delta,
                    motor=ra_motor_delta,
                ),
                delay_s=real_delay_s,
                rate_per_s=Position(
                    mount=ra_mount_delta / real_delay_s,
                    motor=ra_motor_delta / real_delay_s,
                ),
                tracking_rate_tick_per_s=float(start_tracking_rates[0]),
            ),
            dec=AxisInfo(
                start=Position(
                    mount=start_mount_dec,
                    motor=start_motor_dec,
                ),
                end=Position(
                    mount=end_mount_dec,
                    motor=end_motor_dec,
                ),
                delta=Position(
                    mount=dec_mount_delta,
                    motor=dec_motor_delta,
                ),
                delay_s=real_delay_s,
                rate_per_s=Position(
                    mount=dec_mount_delta / real_delay_s,
                    motor=dec_motor_delta / real_delay_s,
                ),
                tracking_rate_tick_per_s=float(start_tracking_rates[1]),
            ),
        )

    def get_deltas(self, delay_s: float) -> Deltas:
        with self.with_get_deltas() as deltas_items:
            time.sleep(delay_s)
        assert len(deltas_items) == 1
        delta = deltas_items[0]
        self.logger.debug("Deltas: %s", delta)
        return delta

    @contextmanager
    def with_get_deltas(self):
        deltas_items: list[Deltas] = []
        start_state = self._capture_deltas_state()
        try:
            yield deltas_items
        finally:
            end_state = self._capture_deltas_state()
            deltas_items.append(self._build_deltas(start_state, end_state))

    # WAITS AND CHECKS

    TRACKING_MODE_TOLERANCE = (1, 1)
    """ (s, arcsec)/s """
    TRACKING_MODE_MOTOR_TOLERANCE = (0.25, 0.25)
    """ (ticks, ticks)/s """

    def wait_while_mount_in_tracking(self, timeout_s: float = 5., times: int = 20):
        self.logger.warning("\n==== WAIT WHILE MOUNT IN TRACKING MODE ====")
        start = time.monotonic()
        while True:
            deltas = self.get_deltas(max(2., (timeout_s * .9) / 10))
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
        self.logger.warning("RA mount rate: %.3f", deltas.ra.rate_per_s.mount)
        self.logger.warning("DEC mount rate: %.3f", deltas.dec.rate_per_s.mount)

        
        self.logger.warning("%s", deltas)

        pytest.fail(f"Mount not stablizes in {time.monotonic() - start:.3f}s")

    def check_mount_in_tracking_mode(self, delta_s: float = 2.):
        self.logger.warning("\n==== CHECK MOUNT AND MOTOR IN TRACKING MODE ====")
        deltas = self.get_deltas(delta_s)
        ra_mode = self.ra.mode()
        dec_mode = self.dec.mode()

        with self._with_position_locks():
            ra_motor_status = self.ra._motor.status()
            dec_motor_status = self.dec._motor.status()

        self.logger.debug("\nDELTAS: %s", deltas)

        self.logger.warning(
            "\n==== TRACKING STATE SNAPSHOT ====\n"
            "RA AXIS MODE: %s\n"
            "DEC AXIS MODE: %s\n"
            "RA MOTOR: mode=%s direction=%s speed_sps=%s\n"
            "DEC MOTOR: mode=%s direction=%s speed_sps=%s",
            ra_mode,
            dec_mode,
            ra_motor_status.motion_mode,
            ra_motor_status.direction,
            ra_motor_status.speed_sps,
            dec_motor_status.motion_mode,
            dec_motor_status.direction,
            dec_motor_status.speed_sps,
        )

        assert ra_mode == AxisMotionMode.TRACK
        assert dec_mode == AxisMotionMode.TRACK

        assert deltas.ra.tracking_rate_tick_per_s > 0
        assert deltas.dec.tracking_rate_tick_per_s == 0

        assert ra_motor_status.motion_mode == MotionMode.RUN
        assert ra_motor_status.direction != MotorDirection.STOP
        assert dec_motor_status.motion_mode == MotionMode.IDLE
        assert dec_motor_status.direction == MotorDirection.STOP

        assert (abs(deltas.ra.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[0]) and \
                (abs(deltas.dec.rate_per_s.mount) < self.TRACKING_MODE_TOLERANCE[1])

        self.logger.warning(
            "\n==== MOUNT IN TRACKING MODE ====\n" 
            "RA: |%.5f| < %.2f\n"
            "DEC: |%.5f| < %.2f",
            deltas.ra.rate_per_s.mount, self.TRACKING_MODE_TOLERANCE[0],
            deltas.dec.rate_per_s.mount, self.TRACKING_MODE_TOLERANCE[1],
        )

        assert (abs(deltas.ra.rate_per_s.motor - deltas.ra.tracking_rate_tick_per_s) < self.TRACKING_MODE_MOTOR_TOLERANCE[0]) and \
                (abs(deltas.dec.rate_per_s.motor) < self.TRACKING_MODE_MOTOR_TOLERANCE[1])

        self.logger.warning(
            "\n==== MOTOR IN TRACKING MODE ====\n" 
            "RA: |%.5f - %.5f| = %.5f < %.2f\n"
            "DEC: |%.5f| < %.2f",
            deltas.ra.rate_per_s.motor, deltas.ra.tracking_rate_tick_per_s,
            abs(deltas.ra.rate_per_s.motor - deltas.ra.tracking_rate_tick_per_s), self.TRACKING_MODE_MOTOR_TOLERANCE[0],
            deltas.dec.rate_per_s.motor, self.TRACKING_MODE_MOTOR_TOLERANCE[1],
        )

        return True

    def wait_until_target_reached(
        self,
        target_ra: Ha,
        target_dec: Dec,
        ra_tolerance_s: float,
        dec_tolerance_arcsec: float,
        timeout_s: float,
        poll_interval_s: float,
    ) -> bool:
        self.logger.warning(
            "\n==== WAIT UNTIL TARGET REACHED ====\n"
            "TARGET RA: %s\n"
            "TARGET DEC: %s\n"
            "RA TOLERANCE: %.3fs\n"
            "DEC TOLERANCE: %.3f arcsec\n"
            "TIMEOUT: %.3fs\n",
            target_ra,
            target_dec,
            ra_tolerance_s,
            dec_tolerance_arcsec,
            timeout_s,
        )

        target_ra_seconds = float(target_ra)
        target_dec_arcsec = float(target_dec)

        start = time.monotonic()
        last_ra_seconds = target_ra_seconds
        last_dec_arcsec = target_dec_arcsec
        last_ra_distance = 0.0
        last_dec_distance = 0.0
        while True:
            last_ra_seconds = float(self.get_ra())
            last_dec_arcsec = float(self.get_dec())
            last_ra_distance = self.ra_distance_seconds(last_ra_seconds, target_ra_seconds)
            last_dec_distance = abs(last_dec_arcsec - target_dec_arcsec)

            if (last_ra_distance <= ra_tolerance_s) and (last_dec_distance <= dec_tolerance_arcsec):
                self.logger.warning(
                    "\n==== TARGET REACHED ====\n"
                    "AFTER: %.3fs\n"
                    "RA DISTANCE: %.3fs\n"
                    "DEC DISTANCE: %.3f arcsec\n",
                    time.monotonic() - start,
                    last_ra_distance,
                    last_dec_distance,
                )
                return True

            if time.monotonic() - start > timeout_s:
                break

            time.sleep(poll_interval_s)

        self.logger.warning(
            "\n==== TARGET NOT REACHED ====\n"
            "AFTER: %.3fs\n"
            "LAST RA: %s\n"
            "LAST DEC: %s\n"
            "RA DISTANCE: %.3fs (limit %.3fs)\n"
            "DEC DISTANCE: %.3f arcsec (limit %.3f arcsec)\n",
            time.monotonic() - start,
            Ha(last_ra_seconds),
            Dec(last_dec_arcsec),
            last_ra_distance,
            ra_tolerance_s,
            last_dec_distance,
            dec_tolerance_arcsec,
        )

        pytest.fail(
            "GOTO did not reach target in time: "
            f"target_ra={target_ra} current_ra={Ha(last_ra_seconds)} "
            f"target_dec={target_dec} current_dec={Dec(last_dec_arcsec)} "
            f"ra_distance={last_ra_distance:.3f}s dec_distance={last_dec_distance:.3f}arcsec"
        )

    def wait_until_goto_started(
        self,
        timeout_s: float,
        sample_s: float,
        ra_min_mount_rate: float,
        dec_min_mount_rate: float,
    ) -> bool:
        self.logger.warning(
            "\n==== WAIT UNTIL GOTO STARTED ====\n"
            "TIMEOUT: %.3fs\n"
            "SAMPLE: %.3fs\n"
            "RA MIN RATE: %.3f\n"
            "DEC MIN RATE: %.3f\n",
            timeout_s,
            sample_s,
            ra_min_mount_rate,
            dec_min_mount_rate,
        )

        start = time.monotonic()
        last_slewing: Deltas | None = None
        while True:
            slewing = self.get_deltas(sample_s)
            last_slewing = slewing
            if (
                abs(slewing.ra.rate_per_s.mount) > ra_min_mount_rate
                or abs(slewing.dec.rate_per_s.mount) > dec_min_mount_rate
            ):
                self.logger.warning(
                    "\n==== GOTO STARTED ====\n"
                    "AFTER: %.3fs\n"
                    "RA RATE: %.5f\n"
                    "DEC RATE: %.5f\n",
                    time.monotonic() - start,
                    slewing.ra.rate_per_s.mount,
                    slewing.dec.rate_per_s.mount,
                )
                self.logger.debug("%s", slewing)
                return True

            if time.monotonic() - start > timeout_s:
                break

        self.logger.warning(
            "\n==== GOTO NOT STARTED ====\n"
            "AFTER: %.3fs\n"
            "LAST DELTAS: %s\n",
            time.monotonic() - start,
            last_slewing,
        )
        pytest.fail("GOTO did not start movement before HALT check")


SYNC_RA_TOLERANCE_S = 8.0
SYNC_DEC_TOLERANCE_ARCSEC = 120.0
SYNC_TRACKING_SETTLE_S = 1.0
SYNC_TRACKING_SAMPLE_S = 3.0

fixture_logger = logging.getLogger("fixtures")

SW_PORT_PATTERN = "PL2303G"
SW_BAUD = 115200
SW_TIMEOUT_S = 0.2
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
    sw_serial = SerialLine(sw_path, SW_BAUD, SW_TIMEOUT_S, SW_SERIAL_NAME, terminator="\r")
    axis_ra = AxisRA(SkyWatcherMotor(sw_serial))

    dec_path = SerialLine.search(DEC_PORT_PATTERN)
    dec_serial = SerialLine(dec_path, DEC_BAUD, DEC_TIMEOUT_S, DEC_SERIAL_NAME, terminator=DEC_TERMINATOR)
    axis_dec = AxisDEC(TMC2209Motor(dec_serial))

    combiner = Combiner(axis_ra, axis_dec)
    lx200 = SkyLX200(combiner)
    lx200.connect()

    fixture_logger.warning("\n===== HARDWARE CONNECTED =====")

    sc = CombinerController(
        lx200=lx200,
        combiner=combiner,
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
        lx200.stop()

        del lx200
        del combiner

        try:
            sw_serial.close()
        except Exception:
            fixture_logger.exception("While sw serial disconnect")

        try:
            dec_serial.close()
        except Exception:
            fixture_logger.exception("While dec adapter disconnect")

        fixture_logger.warning("\n==== HARDWARE DISCONNECTED ====\n")


@pytest.fixture(autouse=True)
def _ensure_halted(sc: CombinerController):
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


@pytest.mark.parametrize(
    ("target_ra_text", "target_dec_text"),
    (
        pytest.param("11:58:00", "+20*00:00", id="sync-ra-11h58m-dec-20d"),
        pytest.param("12:12:34", "+23*15:20", id="sync-ra-12h12m34s-dec-23d15m20s"),
        pytest.param("12:40:10", "+35*30:00", id="sync-ra-12h40m10s-dec-35d30m"),
    ),
)
def test_sync_command_updates_mount_coordinates(
    sc: CombinerController,
    target_ra_text: str,
    target_dec_text: str,
):
    assert sc.check_mount_in_tracking_mode(delta_s=SYNC_TRACKING_SAMPLE_S)
    sc.sync_known_position(target_ra_text, target_dec_text)

    # Let the SkyWatcher position cache expire before sampling motor tracking again.
    time.sleep(SYNC_TRACKING_SETTLE_S)

    assert sc.check_mount_in_tracking_mode(delta_s=SYNC_TRACKING_SAMPLE_S)


def test_mount_in_tracking_mode_by_default(sc: CombinerController):
    assert sc.check_mount_in_tracking_mode(delta_s=5.)


GOTO_TIMEOUT_S = 70.0
GOTO_POLL_INTERVAL_S = 0.5
GOTO_RA_TOLERANCE_S = 30.0
GOTO_DEC_TOLERANCE_ARCSEC = 180.0
GOTO_MIN_RA_MOVE_S = 60.0
GOTO_MIN_DEC_MOVE_ARCSEC = 300.0

GOTO_DIRECTION_DELTAS = (
    pytest.param(400.0, 0.0, id="goto-ra-plus"),
    pytest.param(-400.0, 0.0, id="goto-ra-minus"),
    pytest.param(0.0, 1200.0, id="goto-dec-plus"),
    pytest.param(0.0, -1200.0, id="goto-dec-minus"),
    pytest.param(400.0, 1200.0, id="goto-ra-plus-dec-plus"),
    pytest.param(400.0, -1200.0, id="goto-ra-plus-dec-minus"),
    pytest.param(-400.0, 1200.0, id="goto-ra-minus-dec-plus"),
    pytest.param(-400.0, -1200.0, id="goto-ra-minus-dec-minus"),
)


@pytest.mark.parametrize(
    ("ra_sign", "dec_sign"),
    (
        pytest.param(1, 0, id="east"),
        pytest.param(-1, 0, id="west"),
        pytest.param(0, 1, id="north"),
        pytest.param(0, -1, id="south"),
        pytest.param(1, 1, id="east-north"),
        pytest.param(1, -1, id="east-south"),
        pytest.param(-1, 1, id="west-north"),
        pytest.param(-1, -1, id="west-south"),
    ),
)
def test_coordinate_system_slew_directions(
    sc: CombinerController,
    ra_sign: int,
    dec_sign: int,
):
    baseline = sc.get_deltas(MOTION_SAMPLE_S)

    if ra_sign > 0:
        sc.move_west()
    elif ra_sign < 0:
        sc.move_east()

    if dec_sign > 0:
        sc.move_north()
    elif dec_sign < 0:
        sc.move_south()

    time.sleep(MOTION_SETTLE_S)
    moving = sc.get_deltas(MOTION_SAMPLE_S)

    if ra_sign != 0:
        assert ra_sign * moving.ra.rate_per_s.mount > RA_SLEW_MIN_MOUNT_RATE
        assert abs(moving.ra.rate_per_s.motor) > abs(baseline.ra.rate_per_s.motor) + RA_MANUAL_EXTRA_MOTOR_RATE
    else:
        assert abs(moving.ra.rate_per_s.mount) < RA_STABLE_MOUNT_TOLERANCE
        assert abs(moving.ra.rate_per_s.motor - moving.ra.tracking_rate_tick_per_s) < RA_STABLE_MOTOR_TOLERANCE

    if dec_sign != 0:
        assert dec_sign * moving.dec.rate_per_s.mount > DEC_SLEW_MIN_MOUNT_RATE
        assert abs(moving.dec.rate_per_s.motor) > abs(baseline.dec.rate_per_s.motor) + DEC_MANUAL_EXTRA_MOTOR_RATE
    else:
        assert abs(moving.dec.rate_per_s.mount) < sc.TRACKING_MODE_TOLERANCE[1]
        assert abs(moving.dec.rate_per_s.motor) < DEC_STABLE_MOTOR_TOLERANCE


@pytest.mark.parametrize(
    ("ra_delta_s", "dec_delta_arcsec"),
    GOTO_DIRECTION_DELTAS,
)
def test_goto_command_moves_mount_to_target_coordinates(
    sc: CombinerController,
    ra_delta_s: float,
    dec_delta_arcsec: float,
):
    start_ra, start_dec = sc.sync_known_position("12:00:00", "+20*00:00")
    target_ra = Ha(float(start_ra) + ra_delta_s)
    target_dec = Dec(float(start_dec) + dec_delta_arcsec)

    sc.set_slew_to_find()
    sc.set_target_ra(target_ra)
    sc.set_target_dec(target_dec)
    sc.slew()

    sc.wait_until_target_reached(
        target_ra=target_ra,
        target_dec=target_dec,
        ra_tolerance_s=GOTO_RA_TOLERANCE_S,
        dec_tolerance_arcsec=GOTO_DEC_TOLERANCE_ARCSEC,
        timeout_s=GOTO_TIMEOUT_S,
        poll_interval_s=GOTO_POLL_INTERVAL_S,
    )

    final_ra = float(sc.get_ra())
    final_dec = float(sc.get_dec())

    if ra_delta_s != 0:
        assert sc.ra_distance_seconds(final_ra, float(start_ra)) > GOTO_MIN_RA_MOVE_S
    if dec_delta_arcsec != 0:
        assert abs(final_dec - float(start_dec)) > GOTO_MIN_DEC_MOVE_ARCSEC


MOTION_SETTLE_S = 0.6
MOTION_SAMPLE_S = 5.0

RA_SLEW_MIN_MOUNT_RATE = 0.2
DEC_SLEW_MIN_MOUNT_RATE = 1.0
RA_MANUAL_EXTRA_MOTOR_RATE = 0.5
DEC_MANUAL_EXTRA_MOTOR_RATE = 2.0
RA_STABLE_MOUNT_TOLERANCE = 0.25
RA_STABLE_MOTOR_TOLERANCE = 0.4
DEC_STABLE_MOTOR_TOLERANCE = 0.6


@pytest.mark.parametrize(
    ("ra_sign", "dec_sign"),
    (
        pytest.param(-1, 0, id="east"),
        pytest.param(1, 0, id="west"),
        pytest.param(0, 1, id="north"),
        pytest.param(0, -1, id="south"),
        pytest.param(1, 1, id="east-north"),
        pytest.param(1, -1, id="east-south"),
        pytest.param(-1, 1, id="west-north"),
        pytest.param(-1, -1, id="west-south"),
    ),
)
def test_halt_command_returns_to_tracking_from_slew(
    sc: CombinerController,
    ra_sign: int,
    dec_sign: int,
):
    if ra_sign > 0:
        sc.move_west()
    elif ra_sign < 0:
        sc.move_east()

    if dec_sign > 0:
        sc.move_north()
    elif dec_sign < 0:
        sc.move_south()

    time.sleep(MOTION_SETTLE_S)
    moving = sc.get_deltas(MOTION_SAMPLE_S)

    if ra_sign != 0:
        assert ra_sign * moving.ra.rate_per_s.mount > RA_SLEW_MIN_MOUNT_RATE
    else:
        assert abs(moving.ra.rate_per_s.mount) < RA_STABLE_MOUNT_TOLERANCE

    if dec_sign != 0:
        assert dec_sign * moving.dec.rate_per_s.mount > DEC_SLEW_MIN_MOUNT_RATE
    else:
        assert abs(moving.dec.rate_per_s.mount) < sc.TRACKING_MODE_TOLERANCE[1]

    if ra_sign > 0:
        sc.halt_east()
    elif ra_sign < 0:
        sc.halt_west()

    if dec_sign > 0:
        sc.halt_north()
    elif dec_sign < 0:
        sc.halt_south()

    sc.halt_all()
    sc.wait_while_mount_in_tracking(timeout_s=8.0)
    assert sc.check_mount_in_tracking_mode(delta_s=MOTION_SAMPLE_S)


@pytest.mark.parametrize(
    ("ra_delta_s", "dec_delta_arcsec"),
    GOTO_DIRECTION_DELTAS,
)
def test_halt_command_returns_to_tracking_from_goto(
    sc: CombinerController,
    ra_delta_s: float,
    dec_delta_arcsec: float,
):
    start_ra, start_dec = sc.sync_known_position("12:00:00", "+20*00:00")
    target_ra = Ha(float(start_ra) + ra_delta_s)
    target_dec = Dec(float(start_dec) + dec_delta_arcsec)

    sc.set_slew_to_find()
    sc.set_target_ra(target_ra)
    sc.set_target_dec(target_dec)
    sc.slew()

    sc.wait_until_goto_started(
        timeout_s=15.0,
        sample_s=MOTION_SAMPLE_S,
        ra_min_mount_rate=RA_SLEW_MIN_MOUNT_RATE,
        dec_min_mount_rate=DEC_SLEW_MIN_MOUNT_RATE,
    )

    sc.halt_all()
    sc.wait_while_mount_in_tracking(timeout_s=10.0)
    assert sc.check_mount_in_tracking_mode(delta_s=MOTION_SAMPLE_S)


GUIDE_PULSE_MS_VALUES = (2500, 5000)
GUIDE_PULSE_MS_FOR_HALT = GUIDE_PULSE_MS_VALUES[0]

RA_GUIDE_RATE_DELTA_MIN = 0.15
RA_GUIDE_MOUNT_TOLERANCE = 0.25
RA_GUIDE_DIRECTIONS = {"e", "w"}
DEC_GUIDE_MIN_MOTOR_RATE = 10.0
DEC_GUIDE_MOUNT_TOLERANCE = 5.0
DEC_GUIDE_DIRECTIONS = {"n", "s"}


@pytest.mark.parametrize(
    "pulse_ms",
    (
        pytest.param(GUIDE_PULSE_MS_VALUES[0], id="pulse-2500ms"),
        pytest.param(GUIDE_PULSE_MS_VALUES[1], id="pulse-5000ms"),
    ),
)
@pytest.mark.parametrize(
    ("direction", "expected_sign"),
    (
        pytest.param("e", 1, id="guide-east"),
        pytest.param("w", -1, id="guide-west"),
        pytest.param("n", 1, id="guide-north"),
        pytest.param("s", -1, id="guide-south"),
    ),
)
def test_coordinate_system_guide_ra_rates(
    sc: CombinerController,
    pulse_ms: int,
    direction: str,
    expected_sign: int,
):
    baseline = sc.get_deltas(MOTION_SAMPLE_S)
    sc.guide(direction, pulse_ms)
    time.sleep(MOTION_SETTLE_S)
    guided = sc.get_deltas(MOTION_SAMPLE_S)

    if direction in RA_GUIDE_DIRECTIONS:
        ra_rate_delta = guided.ra.rate_per_s.motor - baseline.ra.rate_per_s.motor
        assert expected_sign * ra_rate_delta > RA_GUIDE_RATE_DELTA_MIN
        assert abs(guided.ra.rate_per_s.mount) < RA_GUIDE_MOUNT_TOLERANCE
        assert abs(guided.dec.rate_per_s.mount) < sc.TRACKING_MODE_TOLERANCE[1]
        assert abs(guided.dec.rate_per_s.motor) < DEC_STABLE_MOTOR_TOLERANCE
    elif direction in DEC_GUIDE_DIRECTIONS:
        assert expected_sign * guided.dec.rate_per_s.motor > DEC_GUIDE_MIN_MOTOR_RATE
        assert abs(guided.dec.rate_per_s.mount) < DEC_GUIDE_MOUNT_TOLERANCE
        assert abs(guided.ra.rate_per_s.mount) < RA_STABLE_MOUNT_TOLERANCE
    else:
        pytest.fail(f"Unexpected guide direction: {direction}")


POLAR_COORDINATE_RA_TEXT = "12:12:34"
POLAR_COORDINATE_DEC_TEXT = "+23*15:20"
POLAR_GUIDE_APPLY_TIMEOUT_S = 3.0
POLAR_GUIDE_DELAY_S = 5.0
POLAR_GUIDING_HOLD_S = 2.0
POLAR_IDLE_RESET_TIMEOUT_S = float(PolarCompensator.DROP_GUIDE_PULSES_COUNT_AFTER) + float(Combiner.GUIDE_INTERVAL_S) + 2.0
POLAR_TAKEOVER_TIMEOUT_S = float(PolarCompensator.DROP_GUIDE_PULSES_COUNT_AFTER) + float(Combiner.GUIDE_INTERVAL_S) + 2.0
POLAR_EPS_TOLERANCE = 1e-6
POLAR_EPS_HOLD_TOLERANCE = 0.01
POLAR_RA_SKY_RATE_TOLERANCE = 0.25
POLAR_DEC_SKY_RATE_TOLERANCE = 1.0
POLAR_DEC_GUIDE_RATE_DELTA_MIN = 5.0

POLAR_DIFFERENT_GUIDES = (
    ("e", 1500),
    ("w", 3200),
    ("n", 1500),
    ("s", 3200),
)
POLAR_DIFFERENT_DEC_GUIDES = (
    ("n", 1500),
    ("n", 2100),
    ("n", 2700),
    ("n", 3300),
)


def _polar_snapshot_is_disabled(sc: CombinerController, snapshot: PolarSnapshot) -> bool:
    return (
        snapshot.status == "disabled"
        and abs(snapshot.eps_n) <= POLAR_EPS_TOLERANCE
        and abs(snapshot.eps_e) <= POLAR_EPS_TOLERANCE
        and abs(snapshot.ra_sky_rate - float(STELLAR_SPEED)) <= POLAR_RA_SKY_RATE_TOLERANCE
        and abs(snapshot.dec_sky_rate) <= POLAR_DEC_SKY_RATE_TOLERANCE
    )


@pytest.fixture
def _prepare_polar_compensator(sc: CombinerController) -> CombinerController:
    sc.logger.warning("\n==== PREPARE POLAR COMPENSATOR TEST ====\n")
    sc.sync_known_position(POLAR_COORDINATE_RA_TEXT, POLAR_COORDINATE_DEC_TEXT)
    sc.disable_polar_compensator()
    sc.wait_for_polar_condition(
        lambda snapshot: _polar_snapshot_is_disabled(sc, snapshot),
        timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
        description="polar compensator disabled and guide rates reset",
    )
    return sc


def _send_guide_sequence(sc: CombinerController, guides: tuple[tuple[str, int], ...]) -> None:
    for index, (direction, pulse_ms) in enumerate(guides):
        sc.logger.warning(
            "\n==== GUIDE PULSE ====\n"
            "INDEX: %d/%d\n"
            "DIRECTION: %s\n"
            "PULSE: %d ms\n",
            index + 1,
            len(guides),
            direction,
            pulse_ms,
        )
        sc.guide(direction, pulse_ms)
        if index + 1 < len(guides):
            time.sleep(POLAR_GUIDE_DELAY_S)


def _assert_guide_applies_quickly(sc: CombinerController, direction: str, pulse_ms: int) -> PolarSnapshot:
    before = sc.get_polar_snapshot()
    start = time.monotonic()
    sc.guide(direction, pulse_ms)

    if direction in RA_GUIDE_DIRECTIONS:
        snapshot = sc.wait_for_polar_condition(
            lambda item: (
                item.status == "disabled"
                and abs(item.eps_n) <= POLAR_EPS_TOLERANCE
                and abs(item.eps_e) <= POLAR_EPS_TOLERANCE
                and abs(item.ra_sky_rate - before.ra_sky_rate) > RA_GUIDE_RATE_DELTA_MIN
                and abs(item.dec_sky_rate - before.dec_sky_rate) <= POLAR_DEC_SKY_RATE_TOLERANCE
            ),
            timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
            description=f"{direction} guide applies without DEC sky-rate update",
        )
    else:
        snapshot = sc.wait_for_polar_condition(
            lambda item: (
                item.status == "disabled"
                and abs(item.eps_n) <= POLAR_EPS_TOLERANCE
                and abs(item.eps_e) <= POLAR_EPS_TOLERANCE
                and abs(item.dec_sky_rate - before.dec_sky_rate) > POLAR_DEC_GUIDE_RATE_DELTA_MIN
                and abs(item.ra_sky_rate - before.ra_sky_rate) <= POLAR_RA_SKY_RATE_TOLERANCE
            ),
            timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
            description=f"{direction} guide applies without RA sky-rate update",
        )

    assert time.monotonic() - start < POLAR_GUIDE_APPLY_TIMEOUT_S
    return snapshot


def _enter_guiding_mode(sc: CombinerController) -> PolarSnapshot:
    sc.logger.warning("\n==== DIFFERENT DEC GUIDES KEEP POLAR COMPENSATOR DISABLED ====\n")
    _send_guide_sequence(sc, POLAR_DIFFERENT_DEC_GUIDES)
    disabled_snapshot = sc.wait_for_polar_condition(
        lambda snapshot: (
            snapshot.status == "disabled"
            and abs(snapshot.eps_n) <= POLAR_EPS_TOLERANCE
            and abs(snapshot.eps_e) <= POLAR_EPS_TOLERANCE
        ),
        timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
        description="polar compensator remains disabled after different DEC guides",
    )
    assert abs(disabled_snapshot.ra_sky_rate - float(STELLAR_SPEED)) <= POLAR_RA_SKY_RATE_TOLERANCE

    settled_guides = tuple(
        ("n", 2500 + (index % 2))
        for index in range(sc.polar_compensator().STABLE_GUIDE_PULSES_COUNT + 2)
    )
    sc.logger.warning("\n==== MOSTLY EQUAL DEC GUIDES MOVE POLAR COMPENSATOR TO GUIDING ====\n")
    _send_guide_sequence(sc, settled_guides)
    return sc.wait_for_polar_condition(
        lambda snapshot: (
            snapshot.status == "guiding"
            and abs(snapshot.eps_n) > POLAR_EPS_TOLERANCE
            and abs(snapshot.eps_e) > POLAR_EPS_TOLERANCE
        ),
        timeout_s=POLAR_TAKEOVER_TIMEOUT_S,
        description="polar compensator enters guiding after mostly equal DEC guides",
    )


@pytest.mark.skip(reason="Need to be rewrited")
def test_polar_compensator_stays_disabled_after_different_guides(_prepare_polar_compensator: CombinerController):
    sc = _prepare_polar_compensator

    sc.logger.warning("\n==== DIFFERENT GUIDES APPLY IMMEDIATELY AND KEEP POLAR COMPENSATOR DISABLED ====\n")
    for index, (direction, pulse_ms) in enumerate(POLAR_DIFFERENT_GUIDES):
        _assert_guide_applies_quickly(sc, direction, pulse_ms)
        if index + 1 < len(POLAR_DIFFERENT_GUIDES):
            time.sleep(POLAR_GUIDE_DELAY_S)

    sc.logger.warning("\n==== WAIT FOR DISABLED POLAR COMPENSATOR TO DROP LATCHED GUIDE RATES ====\n")
    sc.wait_for_polar_condition(
        lambda snapshot: _polar_snapshot_is_disabled(sc, snapshot),
        timeout_s=POLAR_IDLE_RESET_TIMEOUT_S,
        description="disabled polar compensator resets latched guide rates",
    )


@pytest.mark.skip(reason="Need to be rewrited")
def test_polar_compensator_enters_guiding_after_settled_guides(_prepare_polar_compensator: CombinerController):
    sc = _prepare_polar_compensator
    guiding_snapshot = _enter_guiding_mode(sc)

    sc.logger.warning("\n==== GUIDING STATE SHOULD HOLD WITHOUT NEW GUIDE PULSES ====\n")
    time.sleep(POLAR_GUIDING_HOLD_S)
    held_snapshot = sc.wait_for_polar_condition(
        lambda snapshot: snapshot.status == "guiding",
        timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
        description="polar compensator stays in guiding without new guide pulses",
    )

    assert held_snapshot.eps_n == pytest.approx(guiding_snapshot.eps_n, abs=POLAR_EPS_HOLD_TOLERANCE)
    assert held_snapshot.eps_e == pytest.approx(guiding_snapshot.eps_e, abs=POLAR_EPS_HOLD_TOLERANCE)


@pytest.mark.skip(reason="Need to be rewrited")
def test_polar_compensator_resets_after_guiding_then_different_guides(_prepare_polar_compensator: CombinerController):
    sc = _prepare_polar_compensator
    guiding_snapshot = _enter_guiding_mode(sc)

    sc.logger.warning("\n==== GUIDING STATE SHOULD STILL HOLD BEFORE RESET SEQUENCE ====\n")
    time.sleep(POLAR_GUIDING_HOLD_S)
    held_snapshot = sc.wait_for_polar_condition(
        lambda snapshot: snapshot.status == "guiding",
        timeout_s=POLAR_GUIDE_APPLY_TIMEOUT_S,
        description="polar compensator stays in guiding before different-guide reset",
    )

    assert held_snapshot.eps_n == pytest.approx(guiding_snapshot.eps_n, abs=POLAR_EPS_HOLD_TOLERANCE)
    assert held_snapshot.eps_e == pytest.approx(guiding_snapshot.eps_e, abs=POLAR_EPS_HOLD_TOLERANCE)

    sc.logger.warning("\n==== DIFFERENT DEC GUIDES SHOULD RESET POLAR COMPENSATOR BACK TO DISABLED ====\n")
    _send_guide_sequence(sc, POLAR_DIFFERENT_DEC_GUIDES)
    disabled_snapshot = sc.wait_for_polar_condition(
        lambda snapshot: (
            snapshot.status == "disabled"
            and abs(snapshot.eps_n) <= POLAR_EPS_TOLERANCE
            and abs(snapshot.eps_e) <= POLAR_EPS_TOLERANCE
        ),
        timeout_s=POLAR_IDLE_RESET_TIMEOUT_S,
        description="different DEC guides reset guiding state back to disabled",
    )

    assert abs(disabled_snapshot.eps_n) <= POLAR_EPS_TOLERANCE
    assert abs(disabled_snapshot.eps_e) <= POLAR_EPS_TOLERANCE
