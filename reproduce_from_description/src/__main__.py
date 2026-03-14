from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass
from time import sleep

from logging_setup import configure_logging
from lx200.base_server import LX200SimpleServer
from sky.axis import AxisDEC, AxisRA
from sky.combiner import Combiner
from sky.lx200 import SkyLX200
from sky.polar_compensator import PolarCompensator
from skywatcher.motor import SkyWatcherConfig, SkyWatcherMotor
from tmc2209.motor import TMC2209Config, TMC2209Motor
from web_control.web import MonitorRegistry, MonitorServer


LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class Runtime:
    combiner: Combiner
    lx200: SkyLX200
    lx200_server: LX200SimpleServer
    monitor_server: MonitorServer | None


def build_runtime(args: argparse.Namespace) -> Runtime:
    ra_motor = SkyWatcherMotor(
        SkyWatcherConfig(
            port=args.ra_port,
            search_pattern=args.ra_pattern,
            baudrate=args.ra_baudrate,
            timeout=args.serial_timeout,
        )
    )
    dec_motor = TMC2209Motor(
        TMC2209Config(
            port=args.dec_port,
            search_pattern=args.dec_pattern,
            baudrate=args.dec_baudrate,
            timeout=args.serial_timeout,
        )
    )
    ra_axis = AxisRA(ra_motor)
    dec_axis = AxisDEC(dec_motor)
    combiner = Combiner(ra_axis=ra_axis, dec_axis=dec_axis, polar_compensator=PolarCompensator())
    lx200 = SkyLX200(combiner)
    lx200_server = LX200SimpleServer(args.host, args.lx200_port, lx200)

    monitor_server = None
    if args.monitor_port is not None:
        registry = MonitorRegistry()
        if args.live_monitor:
            registry.register(combiner, "mount")
            registry.register(ra_axis, "ra_axis")
            registry.register(dec_axis, "dec_axis")
        monitor_server = MonitorServer(args.host, args.monitor_port, registry)

    return Runtime(combiner=combiner, lx200=lx200, lx200_server=lx200_server, monitor_server=monitor_server)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Reconstructed hybrid mount runtime")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--lx200-port", type=int, default=11880)
    parser.add_argument("--monitor-port", type=int, default=18080)
    parser.add_argument("--serial-timeout", type=float, default=1.0)
    parser.add_argument("--ra-port")
    parser.add_argument("--dec-port")
    parser.add_argument("--ra-pattern", default="SkyWatcher|SynScan|ttyUSB.*")
    parser.add_argument("--dec-pattern", default="TMC2209|ttyUSB.*|ttyACM.*")
    parser.add_argument("--ra-baudrate", type=int, default=9600)
    parser.add_argument("--dec-baudrate", type=int, default=115200)
    parser.add_argument("--live-monitor", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_logging()
    runtime = build_runtime(args)
    LOGGER.info("Runtime built: LX200 on %s:%s", args.host, args.lx200_port)

    if runtime.monitor_server is not None:
        runtime.monitor_server.start()
        LOGGER.info("Monitor server listening on %s:%s", *runtime.monitor_server.server_address)

    if args.dry_run:
        LOGGER.info("Dry run requested, exiting before hardware connect")
        return

    try:
        runtime.lx200_server.serve_forever()
    except KeyboardInterrupt:  # pragma: no cover - runtime path
        LOGGER.info("Stopping runtime")
    finally:
        runtime.lx200_server.shutdown()
        if runtime.monitor_server is not None:
            runtime.monitor_server.stop()
        sleep(0.1)


if __name__ == "__main__":
    main()
