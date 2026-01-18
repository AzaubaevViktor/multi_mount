from __future__ import annotations

from serial_prims import SerialLineDevice
from lx200_prims import LX200Client


class LX200SerialClient(LX200Client):
    def __init__(self, dev: SerialLineDevice):
        super().__init__(dev, logger_name="dec.lx200")
