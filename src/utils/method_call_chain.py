from collections.abc import Callable
from functools import wraps
import sys
from typing import ParamSpec, TypeVar


_P = ParamSpec("_P")
_R = TypeVar("_R")


def log_method_call_chain(depth: int | None = 4) -> Callable[[Callable[_P, _R]], Callable[_P, _R]]:
    def decorator(func: Callable[_P, _R]) -> Callable[_P, _R]:
        @wraps(func)
        def wrapper(*args: _P.args, **kwargs: _P.kwargs) -> _R:
            stack_parts: list[str] = []
            caller_frame = sys._getframe(1)
            frame_index = 0
            while depth is None or frame_index < depth:
                stack_parts.append(
                    f"{caller_frame.f_code.co_name} ({caller_frame.f_code.co_filename}:{caller_frame.f_lineno})"
                )
                if caller_frame.f_back is None:
                    break
                caller_frame = caller_frame.f_back
                frame_index += 1

            args[0].logger.debug(
                "%s call stack: %s",
                func.__name__,
                " <- ".join(stack_parts),
            )
            return func(*args, **kwargs)

        return wrapper

    return decorator
