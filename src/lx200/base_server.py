import logging
import socket
import threading

from lx200.base import LX200Answer, LX200BadCommandError, LX200Handler, LX200SlewResult
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
        # One mount, one conversation. The handler keeps per-conversation state
        # (target RA/DEC from `Sr`/`Sd`, the active manual-move directions behind
        # `Qe`/`Qw`), so two clients on one handler silently overwrite each other's
        # target and halt each other's axes. A second client is refused, loudly,
        # instead of being let in to corrupt the first one's session.
        self._client_slot = threading.Lock()
        self.refused_clients = 0
        
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
        with conn:
            if not self._client_slot.acquire(blocking=False):
                with self._state_lock:
                    self.refused_clients += 1
                self.log.warning("Refusing %r: an LX200 client is already connected to this mount", conn)
                return

            try:
                with self._state_lock:
                    self._connection_id += 1
                    connection_id = self._connection_id

                log = logging.getLogger(f"{self.log.name}.{connection_id}")
                log.info("Connected: %r", conn)
                self.lx200.begin_client_session(repr(conn))

                self._serve_connection(conn, log)
            finally:
                self._client_slot.release()
                self.log.info("Client %s disconnected", connection_id)

    def _serve_connection(self, conn: socket.socket, log: logging.Logger) -> None:
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

                    alignment_mode = self.handle_alignment(bytes(buf))

                    self.log.info("Client asks about alignment mode, responce with %s", alignment_mode)
                    conn.sendall(alignment_mode.value.encode(self.encoding))
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

                log.debug("Receive %s", raw)

                message = raw.decode(self.encoding)

                if not message.startswith(Protocol.COMMAND_PREFIX):
                    # `continue`, not `break`: whatever follows a garbled frame in the
                    # buffer is a perfectly good command whose client is waiting for an
                    # answer, and breaking here strands it until more bytes arrive.
                    self.log.warning("Wrong command (prefix): %s", message)
                    continue

                cmd = message.removeprefix(Protocol.COMMAND_PREFIX).removesuffix(Protocol.TERMINATOR)

                response = self._handle_or_report(cmd, log)
                str_response = self._to_wire(response)

                log.debug("Convert %r -> %r", response, str_response)

                if str_response is not None:
                    log.debug("Send %s", str_response)
                    conn.sendall(str_response.encode(self.encoding))

    def _handle_or_report(self, cmd: str, log: logging.Logger) -> LX200Answer:
        """Run one command. A client never loses its connection over a bad command.

        `Mg` with a short payload used to raise IndexError, an unknown command a
        RuntimeError, a malformed `Sr` an HaFormatError -- all of them straight out
        of the socket loop, killing the connection thread mid-session (PLAN.md #30).
        """
        try:
            return self.handle(cmd)
        except LX200BadCommandError as error:
            log.warning("Rejecting %r: %s", cmd, error)
            return error.answer
        except Exception:
            log.exception("Command %r failed", cmd)
            return None

    @staticmethod
    def _to_wire(response: LX200Answer) -> str | None:
        if response is None:
            return None
        if isinstance(response, LX200SlewResult):
            # `MS` brings its own framing: a bare `0` when accepted, `<code><reason>#`
            # when not. It is not a 0/1 acknowledgement and must not be encoded as one.
            return response.to_wire()
        if isinstance(response, bool):
            return str(int(response))
        return str(response) + Protocol.TERMINATOR

    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return self.lx200.handle_alignment(data)

    def handle(self, data: str) -> LX200Answer:
        return self.lx200.handle(data)
