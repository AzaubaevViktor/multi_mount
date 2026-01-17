from __future__ import annotations
import logging
from typing import Optional

from serial_prims import SerialLineDevice
from coords import parse_dec_dms, parse_ra_hms, fmt_ra, fmt_dec


class LX200SerialClient:
    def __init__(self, dev: SerialLineDevice):
        self.dev = dev
        self.log = logging.getLogger("dec.lx200")

    def cmd(self, cmd: str, expect_hash: bool = True) -> str:
        if not cmd.startswith(":"):
            cmd = ":" + cmd
        if not cmd.endswith("#"):
            cmd = cmd + "#"
        raw = self.dev.transact(cmd.encode("ascii"), terminator=b"#" if expect_hash else b"\n")
        if not raw.endswith(b"#"):
            raise RuntimeError(f"bad lx200 response {raw!r}")
        return raw[:-1].decode("ascii", errors="replace")

    def get_dec(self) -> float:
        resp = self.cmd(":GD#")
        return parse_dec_dms(resp)

    def get_ra(self) -> Optional[float]:
        try:
            resp = self.cmd(":GR#")
            return parse_ra_hms(resp)
        except Exception:
            return None

    def set_target_ra(self, ra_h: float) -> bool:
        return self.cmd(f":Sr {fmt_ra(ra_h)[:-1]}#") in ("1", "0", "")

    def set_target_dec(self, dec_deg: float) -> bool:
        return self.cmd(f":Sd {fmt_dec(dec_deg)[:-1]}#") in ("1", "0", "")

    def goto(self) -> str:
        return self.cmd(":MS#")

    def abort(self) -> None:
        try:
            _ = self.cmd(":Q#")
        except Exception:
            pass

    def move_ns(self, north: bool, start: bool) -> None:
        if start:
            _ = self.cmd(":Mn#" if north else ":Ms#")
        else:
            _ = self.cmd(":Qn#" if north else ":Qs#")

    def move_we(self, east: bool, start: bool) -> None:
        if start:
            _ = self.cmd(":Me#" if east else ":Mw#")
        else:
            _ = self.cmd(":Qe#" if east else ":Qw#")

    def set_accel(self, accel_deg_s2: float) -> None:
        try:
            _ = self.cmd(f":XAC{accel_deg_s2:+08.3f}#")
        except Exception as e:
            self.log.debug("DEC accel extension not supported: %s", e)

    def set_max_rate(self, rate_deg_s: float) -> None:
        try:
            _ = self.cmd(f":XVM{rate_deg_s:07.3f}#")
        except Exception as e:
            self.log.debug("DEC vmax extension not supported: %s", e)
