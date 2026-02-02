import logging
from logging_setup import setup_logging
from lx200.base_server import LX200BaseServer
from lx200.protocol import AlignmentMode


setup_logging()

log = logging.getLogger("lx200")

class LX200TestServer(LX200BaseServer):
    def handle_alignment(self, data: bytes) -> AlignmentMode:
        return AlignmentMode.POLAR
    
    def handle(self, data: str) -> str:
        log.info('Receive %s', data)
        return "0"
    

if __name__ == "__main__":
    server = LX200TestServer()
    server.serve_forever()
