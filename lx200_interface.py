from __future__ import annotations
import asyncio
import datetime as dt
import logging
from typing import Optional

from coords import fmt_ra, fmt_dec, fmt_lx200_lat, fmt_lx200_lon
from mount import FrankenMount


class LX200TCPServer:
    def __init__(self, mount: FrankenMount):
        self.mount = mount
        self.log = logging.getLogger("lx200.tcp")
        self.slew_rate = "C"
        self.rates = {"C": 4.0, "M": 1.0, "S": 0.2, "G": 0.02}

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        self.log.info("Client connected: %s", peer)
        buf = bytearray()
        try:
            while True:
                b = await reader.read(1)
                if not b:
                    return
                buf += b
                if buf.endswith(b"#"):
                    cmd = buf.decode("ascii", errors="replace")
                    buf.clear()
                    resp = await self.dispatch(cmd)
                    if resp is not None:
                        self.log.debug("CMD %r -> %r", cmd, resp)
                        writer.write(resp.encode("ascii"))
                        await writer.drain()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.log.warning("Client error: %s", e, exc_info=True)
        finally:
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass
            self.log.info("Client disconnected: %s", peer)

    async def dispatch(self, cmd: str) -> Optional[str]:
        cmd = cmd.strip()
        if not (cmd.startswith(":") and cmd.endswith("#")):
            return None
        body = cmd[1:-1]
        if body == "GR":
            ra, _ = self.mount.get_ra_dec()
            return fmt_ra(ra)
        if body == "GD":
            _, dec = self.mount.get_ra_dec()
            return fmt_dec(dec)
        if body == "GC":
            utc = self.mount.site.now_utc()
            offset = dt.timedelta(hours=self.mount.site.utc_offset_hours)
            local = utc.astimezone(dt.timezone(offset))
            return f"{local.month:02d}/{local.day:02d}/{local.year%100:02d}#"
        if body == "GL":
            utc = self.mount.site.now_utc()
            offset = dt.timedelta(hours=self.mount.site.utc_offset_hours)
            local = utc.astimezone(dt.timezone(offset))
            return f"{local.hour:02d}:{local.minute:02d}:{local.second:02d}#"
        if body == "Gt":
            return fmt_lx200_lat(self.mount.site.lat_deg)
        if body == "Gg":
            return fmt_lx200_lon(self.mount.site.lon_deg_east)
        if body.startswith("Sr "):
            from coords import parse_ra_hms
            try:
                ra = parse_ra_hms(body[3:])
                return self.mount.set_target_ra(ra)
            except Exception:
                return "0#"
        if body.startswith("Sd "):
            from coords import parse_dec_dms
            try:
                dec = parse_dec_dms(body[3:])
                return self.mount.set_target_dec(dec)
            except Exception:
                return "0#"
        if body == "MS":
            return await self.mount.goto_target()
        if body == "CM":
            return await self.mount.sync_to_target()
        if body == "Q":
            return await self.mount.abort()
        if body in ("RG", "RC", "RM", "RS"):
            self.slew_rate = body[1]
            return "1#"
        if body in ("Mn", "Ms", "Me", "Mw"):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in ("Mn", "Ms"):
                direction = "N" if body == "Mn" else "S"
                return await self.mount.move("dec", direction, True, rate)
            else:
                direction = "E" if body == "Me" else "W"
                return await self.mount.move("ra", direction, True, rate)
        if body in ("Qn", "Qs", "Qe", "Qw"):
            rate = self.rates.get(self.slew_rate, 0.2)
            if body in ("Qn", "Qs"):
                direction = "N" if body == "Qn" else "S"
                return await self.mount.move("dec", direction, False, rate)
            else:
                direction = "E" if body == "Qe" else "W"
                return await self.mount.move("ra", direction, False, rate)
        if body.startswith("XAC"):
            try:
                accel = float(body[3:])
                self.mount.dec_accel = accel
                await asyncio.to_thread(self.mount.dec.set_accel, accel)
                return "1#"
            except Exception:
                return "0#"
        if body.startswith("XVM"):
            try:
                vmax = float(body[3:])
                self.mount.dec_vmax = vmax
                await asyncio.to_thread(self.mount.dec.set_max_rate, vmax)
                return "1#"
            except Exception:
                return "0#"
        return "#"
