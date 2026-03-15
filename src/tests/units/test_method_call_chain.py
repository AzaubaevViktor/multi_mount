import logging

from utils.method_call_chain import log_method_call_chain


class _Sample:
    def __init__(self) -> None:
        self.logger = logging.getLogger("tests.method_call_chain")

    @log_method_call_chain(depth=None)
    def decorated(self) -> str:
        return "ok"


class _WithoutLogger:
    @log_method_call_chain(depth=None)
    def decorated(self) -> str:
        return "ok"


@log_method_call_chain(depth=None)
def _decorated_function(sample) -> str:
    return f"ok:{sample}"


def test_log_method_call_chain_supports_unbounded_depth(caplog) -> None:
    sample = _Sample()

    with caplog.at_level(logging.DEBUG, logger="tests.method_call_chain"):
        assert sample.decorated() == "ok"

    assert any("decorated call stack:" in message for message in caplog.messages)


def test_log_method_call_chain_falls_back_to_root_logger_when_method_has_no_logger(caplog) -> None:
    sample = _WithoutLogger()

    with caplog.at_level(logging.DEBUG):
        assert sample.decorated() == "ok"

    assert any("decorated call stack:" in message for message in caplog.messages)


def test_log_method_call_chain_treats_plain_function_as_plain_function(caplog) -> None:
    sample = _Sample()

    with caplog.at_level(logging.DEBUG):
        assert _decorated_function(sample) == f"ok:{sample}"

    assert any("_decorated_function call stack:" in message for message in caplog.messages)
