import logging
from serial.wrapper import SerialLine


class SkyWatcher:
    _LEADING = b":"
    _TRAILING = b"\r"

    def __init__(self, serial: SerialLine) -> None:
        self.logger = logging.getLogger("skywatcher")
        self.serial = serial

    def connect(self):
        self.serial.connect()