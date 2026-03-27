import logging
import socket
import threading
from typing import Any

from lx200.base import LX200Handler
from lx200.protocol import AlignmentMode, Protocol


class LX200SimpleServer:
    def __init__(
        self,
        lx200: LX200Handler,
        host: str = "localhost",
        port: int = 7624,
        buffer_size: int = 1,
        encoding: str = "ascii",
    ) -> None:
        self.log = logging.getLogger("server")
        self.lx200 = lx200
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.encoding = encoding
        self._terminator_byte = Protocol.TERMINATOR.encode(self.encoding)

        self._connection_id = -1
        self._socket: socket.socket | None = None
        self._serve_thread: threading.Thread | None = None
        self._running = False
        self._state_lock = threading.RLock()
        self.last_error: Exception | None = None
        
    def serve_forever(self) -> None:
        with self._state_lock:
            if self._running:
                self.log.info("LX200 server is already running on %s:%s", self.host, self.port)
                return
            self._running = True
            self.last_error = None

        if not self.lx200.is_connected():
            self.lx200.connect()

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((self.host, self.port))
                srv.listen(1)
                with self._state_lock:
                    self._socket = srv
                self.log.info("LX200 server listening on %s:%s", self.host, self.port)
                while True:
                    try:
                        conn, addr = srv.accept()
                    except OSError:
                        with self._state_lock:
                            if not self._running:
                                break
                        raise
                    self.log.info("Client connected: %s", addr)
                    thread = threading.Thread(target=self._handle_client, args=(conn,), daemon=True)
                    thread.start()
        except Exception as exc:
            with self._state_lock:
                self.last_error = exc
            raise
        finally:
            with self._state_lock:
                self._running = False
                self._socket = None

    def start_background(self) -> bool:
        with self._state_lock:
            if self._serve_thread is not None and self._serve_thread.is_alive():
                return False
            self.last_error = None
            self._serve_thread = threading.Thread(target=self._serve_in_background, name="LX200_SERVER", daemon=True)
            self._serve_thread.start()
            return True

    def _serve_in_background(self) -> None:
        try:
            self.serve_forever()
        except Exception:
            self.log.exception("LX200 server stopped with error")

    def stop(self, join_timeout_s: float = 1.0) -> None:
        with self._state_lock:
            self._running = False
            server_socket = self._socket
            serve_thread = self._serve_thread

        if server_socket is not None and hasattr(server_socket, "close"):
            try:
                server_socket.close()
            except OSError:
                pass

        if serve_thread is not None and serve_thread.is_alive() and serve_thread is not threading.current_thread():
            serve_thread.join(join_timeout_s)

    def is_running(self) -> bool:
        with self._state_lock:
            return self._running
    
    def _handle_client(self, conn: socket.socket) -> None:
        # Current limitation: multiple clients can talk to the same LX200 handler concurrently.
        # EOF only stops this client thread; there is no connection-level disconnect hook yet.
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
                        conn.sendall(response.value.encode(self.encoding))
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
                    else:
                        str_response = str(response) + Protocol.TERMINATOR

                    self.log.debug("Convert %r -> %r", response, str_response)

                    if str_response is not None:
                        self.log.debug("Send %s", str_response)

                        conn.sendall(str_response.encode(self.encoding))
                    

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return self.lx200.handle_alignment(data)

    def handle(self, data: str) -> Any:
        return self.lx200.handle(data)
