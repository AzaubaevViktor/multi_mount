import logging
import os
import re

import serial


class SerialLineError(Exception):
    pass


class SerialLineSearchError(SerialLineError):
    pass


class SerialLineSearchInvalidPattern(SerialLineSearchError):
    pass


class SerialLineSearchDirectoryError(SerialLineSearchError):
    pass


class SerialLineSearchNotFound(SerialLineSearchError):
    pass


class SerialLine:
    def __init__(self, port: str, baud: int, timeout_s: float, name: str, terminator: str = "\n", encoding: str ='ascii') -> None:
        self.logger = logging.getLogger(f"serial.{name}")
        self.port = port
        self.baud = baud
        self.timeout_s = timeout_s
        self.encoding = encoding
        self.terminator = terminator.encode(self.encoding)

        self.serial: serial.Serial

    @classmethod
    def search(cls, pattern: str, directory: str = "/dev") -> str:
        if not pattern:
            raise SerialLineSearchError("pattern is required")
        if not directory:
            raise SerialLineSearchError("directory is required")

        try:
            regex = re.compile(pattern)
        except re.error as exc:
            raise SerialLineSearchInvalidPattern(
                f"invalid search pattern: {pattern!r}"
            ) from exc

        try:
            with os.scandir(directory) as entries:
                for entry in entries:
                    if regex.search(entry.name):
                        return entry.path
        except OSError as exc:
            raise SerialLineSearchDirectoryError(
                f"cannot read directory: {directory!r}"
            ) from exc

        raise SerialLineSearchNotFound(
            f"no match for pattern {pattern!r} in {directory!r}"
        )
    
    def reset(self):
        self.serial.dtr = True

    def connect(self):
        self.serial = serial.Serial(port=self.port, baudrate=self.baud, timeout=self.timeout_s)
        self.logger.info("Connected to %s:%s (timeout=%d)", self.port, self.baud, self.timeout_s)

    def query(self, payload: str) -> str:
        self.logger.debug("Send `%s`", payload)
        self.serial.reset_input_buffer()
        self.serial.write(payload.encode(self.encoding))
        self.serial.flush()

        return self.read_line()
    
    def read_line(self, timeout: int | None = None):
        _timeout = self.serial.timeout
        self.serial.timeout = timeout
        try:
            line = self.serial.read_until(self.terminator, 1024)
        finally:
            self.serial.timeout = _timeout

        responce = line.decode(self.encoding).strip()
        self.logger.debug("Receive `%s`", responce)

        return responce
    
    def read_all_data(self, timeout: int | None = None) -> list[str] | None:
        _timeout = self.serial.timeout
        self.serial.timeout = timeout
        try:
            if (data := self.serial.read_all()) is None:
                return None
        finally:
            self.serial.timeout = _timeout
        
        lines = [line.decode(self.encoding) for line in data.split(self.terminator)]

        self.logger.debug("Receive all data from input:\n%s", lines)

    def close(self):
        self.serial.close()

