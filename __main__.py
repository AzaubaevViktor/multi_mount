from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys

from serial_prims import SerialLineDevice
from skywatcher import SkyWatcherMC
from arduino_dec import LX200SerialClient
from dummy_dec import DummyDEC
from mount import FrankenMount, SiteTime
from lx200_interface import LX200TCPServer
from coords import parse_ra_hms, parse_dec_dms, fmt_ra, fmt_dec


def setup_logging(level: str) -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s.%(msecs)03d %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


async def console_cli(mount: FrankenMount) -> None:
    print("Test console started. Type 'help' for commands.")
    while True:
        try:
            line = await asyncio.to_thread(input, "console> ")
        except (EOFError, KeyboardInterrupt):
            print("Exiting console.")
            os._exit(0)
        cmd = line.strip()
        if not cmd:
            continue
        if cmd in ("help", "h"):
            print("pos, stop, setra HH:MM:SS, setdec +DD*MM[:SS], goto, sidereal on|off, exit")
            continue
        if cmd == "pos":
            ra, dec = mount.get_ra_dec()
            print(f"RA={fmt_ra(ra)[:-1]} DEC={fmt_dec(dec)[:-1]}")
            continue
        if cmd == "stop":
            await mount.abort()
            print("Abort sent.")
            continue
        if cmd.startswith("setra "):
            try:
                ra = parse_ra_hms(cmd[6:])
                mount.set_target_ra(ra)
                print(f"Target RA set to {fmt_ra(ra)[:-1]}")
            except Exception as e:
                print(f"Bad RA: {e}")
            continue
        if cmd.startswith("setdec "):
            try:
                dec = parse_dec_dms(cmd[7:])
                mount.set_target_dec(dec)
                print(f"Target DEC set to {fmt_dec(dec)[:-1]}")
            except Exception as e:
                print(f"Bad DEC: {e}")
            continue
        if cmd == "goto":
            print("Starting GOTO...")
            res = await mount.goto_target()
            print(f"GOTO result: {res}")
            continue
        if cmd.startswith("sidereal "):
            a = cmd.split()
            if len(a) >= 2 and a[1] in ("on", "off"):
                await mount.enable_tracking(a[1] == "on")
                print(f"Sidereal tracking {'enabled' if a[1]=='on' else 'disabled'}.")
            else:
                print("Usage: sidereal on|off")
            continue
        if cmd in ("exit", "quit"):
            print("Exiting process.")
            os._exit(0)
        print("Unknown command. Type 'help'.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="127.0.0.1:10001", help="host:port for LX200 TCP server")
    ap.add_argument("--log-level", default="INFO")

    ap.add_argument("--ra-port", required=True)
    ap.add_argument("--ra-baud", type=int, default=9600)
    ap.add_argument("--ra-timeout", type=float, default=0.5)
    ap.add_argument("--ra-channel", default="1", choices=["1", "2"]) 
    ap.add_argument("--ra-ccw", action="store_true")
    ap.add_argument("--ra-sign", type=int, default=+1, choices=[-1, +1])

    ap.add_argument("--dec-port", default=None, help="serial port for DEC Arduino (optional)")
    ap.add_argument("--dec-baud", type=int, default=115200)
    ap.add_argument("--dec-timeout", type=float, default=0.5)

    ap.add_argument("--site-lat", type=float, default=0.0)
    ap.add_argument("--site-lon", type=float, default=0.0)
    ap.add_argument("--utc-offset", type=float, default=0.0)
    ap.add_argument("--dec-accel", type=float, default=5.0)
    ap.add_argument("--dec-vmax", type=float, default=4.0)
    ap.add_argument("--test-mode", action="store_true", help="Run interactive test console for mount control")

    args = ap.parse_args()
    setup_logging(args.log_level)

    host, port_s = args.listen.split(":")
    port = int(port_s)

    ra_dev = SerialLineDevice(args.ra_port, args.ra_baud, args.ra_timeout, name="serial.ra")

    ra = SkyWatcherMC(ra_dev)

    dec_dev = None
    if args.dec_port:
        dec_dev = SerialLineDevice(args.dec_port, args.dec_baud, args.dec_timeout, name="serial.dec")
        dec = LX200SerialClient(dec_dev)
    else:
        dec = DummyDEC(site=None)
    site = SiteTime(lat_deg=args.site_lat, lon_deg_east=args.site_lon, utc_offset_hours=args.utc_offset)

    mount = FrankenMount(
        ra=ra,
        ra_ch=args.ra_channel,
        dec=dec,
        site=site,
        ra_ccw=args.ra_ccw,
        ra_sign=args.ra_sign,
        dec_accel_deg_s2=args.dec_accel,
        dec_vmax_deg_s=args.dec_vmax,
    )

    server = LX200TCPServer(mount)

    async def runner():
        await mount.start()
        if args.test_mode:
            asyncio.create_task(console_cli(mount))
        srv = await asyncio.start_server(server.handle_client, host, port)
        addrs = ", ".join(str(sock.getsockname()) for sock in srv.sockets or [])
        logging.getLogger("main").info("LX200 TCP listening on %s", addrs)
        async with srv:
            await srv.serve_forever()

    try:
        asyncio.run(runner())
    finally:
        try:
            ra_dev.close()
        except Exception:
            pass
        try:
            if dec_dev is not None:
                dec_dev.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

