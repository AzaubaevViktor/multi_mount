from __future__ import annotations
import asyncio
import datetime as dt
import logging
from typing import Optional

from coords import fmt_ra, fmt_dec, fmt_lx200_lat, fmt_lx200_lon
from mount import FrankenMount
from lx200_prims import LX200Server, LX200Backend


class LX200TCPServer:
    def __init__(self, mount: FrankenMount):
        self.mount = mount
        self.log = logging.getLogger("lx200.tcp")
        self.slew_rate = "C"
        self.rates = {"C": 4.0, "M": 1.0, "S": 0.2, "G": 0.02}
        backend = LX200Backend(mount)
        self.dispatcher = LX200Server(backend)

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
        return await self.dispatcher.dispatch(cmd)
