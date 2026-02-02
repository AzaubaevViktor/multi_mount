from __future__ import annotations

import argparse
import logging

from dummy_server import (
    LX200DummyConstants,
    LX200DummyServerError,
    LX200DummyTcpServer,
)
from lx200.plugins import (
    LX200ObjectPlugin,
    LX200PointingPlugin,
    LX200SitePlugin,
    LX200TimePlugin,
    LX200TrackingPlugin,
)
from lx200.server import LX200CommandHandler, LX200Server
from lx200_combine.splitter import (
    LX200PrimaryAxis,
    LX200Splitter,
)
from skywatcher_lx200.common import SkyWatcherBackendConstants, SkyWatcherSerialConfig
from skywatcher_lx200.mount import SkyWatcherMount
from skywatcher_lx200.object import SkyWatcherObjectBackend
from skywatcher_lx200.pointing import SkyWatcherPointingBackend
from skywatcher_lx200.site import SkyWatcherSiteBackend
from skywatcher_lx200.time import SkyWatcherTimeBackend
from skywatcher_lx200.tracking import SkyWatcherTrackingBackend
from tmc2209.proxy import ProtocolConstants, TMC2209ArduinoConfig, TMC2209ArduinoProxy
from tmc2209_lx200.common import (
    TMC2209AxisConfig,
    TMC2209AxisMapping,
    TMC2209LX200Constants,
    TMC2209MountConfig,
)
from tmc2209_lx200.mount import TMC2209Mount
from tmc2209_lx200.pointing import TMC2209PointingBackend
from tmc2209_lx200.tracking import TMC2209TrackingBackend

from lib.logging_setup import setup_logging


SPLITTER_LOGGER_NAME = "lx200.combine.skywatcher_tmc2209"
DEFAULT_PRIMARY = LX200PrimaryAxis.RA
DEFAULT_TMC_SITE_NAME = "TMC2209"


def _build_skywatcher_handler(
    mount: SkyWatcherMount,
    *,
    site_name: str,
) -> LX200CommandHandler:
    plugins = [
        LX200PointingPlugin(SkyWatcherPointingBackend(mount)),
        LX200TimePlugin(SkyWatcherTimeBackend()),
        LX200SitePlugin(SkyWatcherSiteBackend(site_name=site_name)),
        LX200TrackingPlugin(SkyWatcherTrackingBackend(mount)),
        LX200ObjectPlugin(SkyWatcherObjectBackend()),
    ]
    return LX200Server(plugins)


def _build_tmc_handler(
    mount: TMC2209Mount,
    *,
    site_name: str,
) -> LX200CommandHandler:
    # TODO: extract shared time/site/object backends into a neutral lx200 module.
    plugins = [
        LX200PointingPlugin(TMC2209PointingBackend(mount)),
        LX200TimePlugin(SkyWatcherTimeBackend()),
        LX200SitePlugin(SkyWatcherSiteBackend(site_name=site_name)),
        LX200TrackingPlugin(TMC2209TrackingBackend(mount)),
        LX200ObjectPlugin(SkyWatcherObjectBackend()),
    ]
    return LX200Server(plugins)


CLI_DESCRIPTION = "SkyWatcher RA + TMC2209 DEC LX200 splitter server"
DEFAULT_SKYWATCHER_PORT = "/dev/tty.PL2303G-USBtoUART2120"
DEFAULT_SKYWATCHER_BAUD = 115200
DEFAULT_SKYWATCHER_TIMEOUT_S = 0.5
DEFAULT_TMC_PORT = "/dev/tty.usbserial-2110"
MAIN_WHEEL_DEGREE_PER_MOUNT_ROTATE = 90 / 30.5 
NEMA_ROTATE_PER_MOUNT_ROTATE = 26 / 44
NEMA_STEPS_PER_ROTATION = 200
DEFAULT_TMC_STEPS_PER_DEG = NEMA_STEPS_PER_ROTATION * NEMA_ROTATE_PER_MOUNT_ROTATE * MAIN_WHEEL_DEGREE_PER_MOUNT_ROTATE
DEFAULT_TMC_AUTO_ENABLE = True
DEFAULT_TMC_DEC_FORWARD_IS_NORTH = True


class SkyWatcherTMC2209CliError(Exception):
    pass


class SkyWatcherTMC2209CliConfigError(SkyWatcherTMC2209CliError):
    pass


def _parse_primary(value: str) -> LX200PrimaryAxis:
    try:
        return LX200PrimaryAxis(value)
    except ValueError as exc:
        raise SkyWatcherTMC2209CliConfigError(f"unsupported primary axis: {value!r}") from exc


def _resolve_tmc_serial(args: argparse.Namespace) -> TMC2209ArduinoConfig:
    port = args.tmc_port or DEFAULT_TMC_PORT
    if not port:
        raise SkyWatcherTMC2209CliConfigError(
            "tmc2209 port is required; use --tmc-port"
        )
    baud = args.tmc_baud if args.tmc_baud is not None else ProtocolConstants.DEFAULT_BAUD
    timeout_s = (
        args.tmc_timeout if args.tmc_timeout is not None else ProtocolConstants.DEFAULT_TIMEOUT_S
    )
    idle_timeout_s = (
        args.tmc_idle_timeout
        if args.tmc_idle_timeout is not None
        else ProtocolConstants.DEFAULT_IDLE_TIMEOUT_S
    )
    device_name = args.tmc_device_name or ProtocolConstants.DEFAULT_DEVICE_NAME
    return TMC2209ArduinoConfig(
        port=port,
        baud=baud,
        timeout_s=timeout_s,
        idle_timeout_s=idle_timeout_s,
        device_name=device_name,
    )


def _resolve_tmc_axis_config(args: argparse.Namespace) -> TMC2209AxisConfig:
    steps_per_deg = args.tmc_steps_per_deg
    if steps_per_deg is None:
        steps_per_deg = DEFAULT_TMC_STEPS_PER_DEG
    return TMC2209AxisConfig(
        steps_per_degree=steps_per_deg,
        guide_sps=args.tmc_guide_sps,
        center_sps=args.tmc_center_sps,
        find_sps=args.tmc_find_sps,
        slew_sps=args.tmc_slew_sps,
        goto_sps=args.tmc_goto_sps,
        tolerance_steps=args.tmc_tolerance_steps,
        auto_enable=args.tmc_auto_enable,
    )


def _resolve_tmc_mount_config(args: argparse.Namespace) -> TMC2209MountConfig:
    axis_mapping = TMC2209AxisMapping(dec_forward_is_north=args.tmc_dec_forward_is_north)
    dec_axis_config = _resolve_tmc_axis_config(args)
    return TMC2209MountConfig(
        axis_mapping=axis_mapping,
        dec_axis_config=dec_axis_config,
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=CLI_DESCRIPTION)
    parser.add_argument(
        "--host",
        default=LX200DummyConstants.HOST,
    )
    parser.add_argument(
        "--port",
        default=LX200DummyConstants.PORT,
        type=int,
    )
    parser.add_argument("--skywatcher-port", default=DEFAULT_SKYWATCHER_PORT)
    parser.add_argument("--skywatcher-baud", default=DEFAULT_SKYWATCHER_BAUD, type=int)
    parser.add_argument("--skywatcher-timeout", default=DEFAULT_SKYWATCHER_TIMEOUT_S, type=float)
    parser.add_argument(
        "--primary",
        choices=[axis.value for axis in LX200PrimaryAxis],
        default=DEFAULT_PRIMARY.value,
    )
    parser.add_argument(
        "--skywatcher-site-name",
        default=SkyWatcherBackendConstants.DEFAULT_SITE_NAME,
    )
    parser.add_argument(
        "--tmc-site-name",
        default=DEFAULT_TMC_SITE_NAME,
    )
    parser.add_argument("--tmc-port", default=DEFAULT_TMC_PORT)
    parser.add_argument(
        "--tmc-baud",
        default=None,
        type=int,
    )
    parser.add_argument(
        "--tmc-timeout",
        default=None,
        type=float,
    )
    parser.add_argument(
        "--tmc-idle-timeout",
        default=None,
        type=float,
    )
    parser.add_argument(
        "--tmc-device-name",
        default=ProtocolConstants.DEFAULT_DEVICE_NAME,
    )
    parser.add_argument(
        "--tmc-steps-per-deg",
        default=None,
        type=float,
    )
    parser.add_argument(
        "--tmc-guide-sps",
        default=TMC2209LX200Constants.DEFAULT_GUIDE_SPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-center-sps",
        default=TMC2209LX200Constants.DEFAULT_CENTER_SPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-find-sps",
        default=TMC2209LX200Constants.DEFAULT_FIND_SPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-slew-sps",
        default=TMC2209LX200Constants.DEFAULT_SLEW_SPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-goto-sps",
        default=TMC2209LX200Constants.DEFAULT_GOTO_SPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-tolerance-steps",
        default=TMC2209LX200Constants.DEFAULT_TOLERANCE_STEPS,
        type=int,
    )
    parser.add_argument(
        "--tmc-auto-enable",
        default=DEFAULT_TMC_AUTO_ENABLE,
        action=argparse.BooleanOptionalAction,
    )
    parser.add_argument(
        "--tmc-dec-forward-is-north",
        default=DEFAULT_TMC_DEC_FORWARD_IS_NORTH,
        action=argparse.BooleanOptionalAction,
    )
    return parser.parse_args()


def _resolve_skywatcher_serial(args: argparse.Namespace) -> SkyWatcherSerialConfig:
    if not args.skywatcher_port:
        raise SkyWatcherTMC2209CliConfigError(
            "skywatcher port is required; use --skywatcher-port"
        )
    return SkyWatcherSerialConfig(
        port=args.skywatcher_port,
        baud=args.skywatcher_baud,
        timeout_s=args.skywatcher_timeout,
    )


def _build_splitter(
    args: argparse.Namespace,
) -> tuple[LX200Splitter, SkyWatcherMount, TMC2209Mount]:
    skywatcher_serial = _resolve_skywatcher_serial(args)
    skywatcher_mount = SkyWatcherMount.from_serial(skywatcher_serial)
    if not args.skywatcher_site_name:
        raise SkyWatcherTMC2209CliConfigError("skywatcher site name is required")
    tmc_serial = _resolve_tmc_serial(args)
    tmc_proxy = TMC2209ArduinoProxy.from_serial(tmc_serial)
    tmc_config = _resolve_tmc_mount_config(args)
    tmc_mount = TMC2209Mount(dec_proxy=tmc_proxy, config=tmc_config)
    if not args.tmc_site_name:
        raise SkyWatcherTMC2209CliConfigError("tmc2209 site name is required")
    primary = _parse_primary(args.primary)
    splitter = LX200Splitter(
        _build_skywatcher_handler(skywatcher_mount, site_name=args.skywatcher_site_name),
        _build_tmc_handler(tmc_mount, site_name=args.tmc_site_name),
        primary=primary,
        logger=logging.getLogger(SPLITTER_LOGGER_NAME),
    )
    return splitter, skywatcher_mount, tmc_mount


def run_server(host: str, port: int, *, args: argparse.Namespace) -> None:
    splitter, skywatcher_mount, tmc_mount = _build_splitter(args)
    server = LX200DummyTcpServer(splitter, host=host, port=port)
    try:
        server.serve_forever()
    finally:
        skywatcher_mount.close()
        tmc_mount.close()


def main() -> None:
    setup_logging()
    args = _parse_args()
    try:
        run_server(args.host, args.port, args=args)
    except (SkyWatcherTMC2209CliError, LX200DummyServerError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
