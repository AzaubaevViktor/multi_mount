import logging
import socket
import threading
from typing import Any

from lx200.base import LX200Base
from lx200.protocol import AlignmentMode, Protocol


class LX200SimpleServer:
    def __init__(
            self, 
            lx200: LX200Base,
            host: str = 'localhost', 
            port: int = 7624, 
            buffer_size: int = 1,
            encoding: str = 'ascii') -> None:
        self.log = logging.getLogger("server")
        self.lx200 = lx200
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.encoding = encoding
        self._terminator_byte = Protocol.TERMINATOR.encode(self.encoding)

        self._connection_id = -1
        
    def serve_forever(self) -> None:
        self.lx200.connect()

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            self._socket = srv
            self.log.info("Dummy server listening on %s:%s", self.host, self.port)
            while True:
                conn, addr = srv.accept()
                self.log.info("Client connected: %s", addr)
                thread = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                thread.start()
    
    def _handle_client(self, conn: socket.socket) -> None:
        # TODO: Don't allow multiple connections
        # TODO: Detect disconnection and run self.handle_disconnect
        with conn:
            self._connection_id += 1

            log = logging.getLogger(f"{self.log.name}.{self._connection_id}")
            log.info("Connected: %r", conn)

            buf = bytearray()

            while True:
                data = conn.recv(self.buffer_size)
                if not data:
                    return

                idx = data.find(Protocol.ALIGNMENT_QUERY_BYTE)

                if idx >= 0:
                    while idx >= 0:
                        if idx:
                            buf.extend(data[:idx])
                        
                        response = self.handle_alignment(buf)

                        self.log.info("Client asks about alignment mode, responce with %s", response)
                        conn.sendall((response.value).encode(self.encoding))
                        data = data[idx + 1 :]
                        idx = data.find(Protocol.ALIGNMENT_QUERY_BYTE)
                    if data:
                        buf.extend(data)
                else:
                    buf.extend(data)

                while True:
                    idx = buf.find(self._terminator_byte)
                    if idx < 0:
                        break
                    raw = bytes(buf[: idx + 1])
                    del buf[: idx + 1]
                    
                    self.log.debug("Receive %s", raw)

                    message = raw.decode(self.encoding)

                    if not message.startswith(Protocol.COMMAND_PREFIX):
                        self.log.warning("Wrong command (prefix): %s", message)
                        break

                    cmd = message.removeprefix(Protocol.COMMAND_PREFIX).removesuffix(Protocol.TERMINATOR)

                    response = self.handle(cmd)

                    if response is None:
                        str_response = None
                    elif isinstance(response, bool):
                        str_response = str(int(response))
                        self.log.debug("Send %s", str_response)
                    else:
                        str_response = str(response) + Protocol.TERMINATOR

                    self.log.debug("Convert %r -> %s", response, str_response)


                    if str_response is None:
                        self.log.debug("Nothing to return")
                    else:
                        self.log.debug("Send %s", str_response)

                        conn.sendall(str_response.encode(self.encoding))
                    

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return self.lx200.handle_alignment(data)

    def handle(self, data: str) -> Any:
        return self.lx200.handle(data)
