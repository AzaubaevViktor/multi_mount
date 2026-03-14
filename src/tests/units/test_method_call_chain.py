import logging

from utils.method_call_chain import log_method_call_chain


class _Sample:
    def __init__(self) -> None:
        self.logger = logging.getLogger("tests.method_call_chain")

    @log_method_call_chain(depth=None)
    def decorated(self) -> str:
        return "ok"


def test_log_method_call_chain_supports_unbounded_depth(caplog) -> None:
    sample = _Sample()

    with caplog.at_level(logging.DEBUG, logger="tests.method_call_chain"):
        assert sample.decorated() == "ok"

    assert any("decorated call stack:" in message for message in caplog.messages)
