from __future__ import annotations

import logging
import socketserver

from .base import LX200Base, LX200Handler


LOGGER = logging.getLogger(__name__)


class _LX200RequestHandler(socketserver.BaseRequestHandler):
    def handle(self) -> None:
        buffer = bytearray()
        while True:
            chunk = self.request.recv(1024)
            if not chunk:
                return

            for byte in chunk:
                if byte == 0x06:
                    self.request.sendall(self.server.lx200.handle_alignment().encode("ascii"))
                    continue

                if not buffer and byte != ord(":"):
                    continue

                buffer.append(byte)
                if byte != ord("#"):
                    continue

                frame = bytes(buffer)
                buffer.clear()
                try:
                    reply = self.server.handler.handle(frame[1:-1].decode("ascii"))
                except Exception:  # pragma: no cover - defensive runtime path
                    LOGGER.exception("Failed to process LX200 command")
                    reply = b""

                if reply:
                    self.request.sendall(reply)


class _ThreadingLX200Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler: LX200Handler, lx200: LX200Base) -> None:
        super().__init__(server_address, _LX200RequestHandler)
        self.handler = handler
        self.lx200 = lx200


class LX200SimpleServer:
    def __init__(self, host: str, port: int, lx200: LX200Base) -> None:
        self.lx200 = lx200
        self.handler = LX200Handler(lx200)
        self._server = _ThreadingLX200Server((host, port), handler=self.handler, lx200=lx200)

    @property
    def server_address(self) -> tuple[str, int]:
        return self._server.server_address

    def serve_forever(self) -> None:
        self.lx200.connect()
        self._server.serve_forever()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self.lx200.stop()
