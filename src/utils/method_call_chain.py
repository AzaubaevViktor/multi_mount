import logging
from collections.abc import Callable
from functools import wraps
import sys
from types import FrameType
from typing import ParamSpec, Protocol, TypeVar, cast


_P = ParamSpec("_P")
_R = TypeVar("_R")


class _SupportsDebug(Protocol):
    def debug(self, msg: object, *args: object, **kwargs: object) -> object: ...


def format_stack_frame(frame: FrameType) -> str:
    return f"{frame.f_code.co_name} ({frame.f_code.co_filename}:{frame.f_lineno})"


def log_method_call_chain(depth: int | None = 4) -> Callable[[Callable[_P, _R]], Callable[_P, _R]]:
    def decorator(func: Callable[_P, _R]) -> Callable[_P, _R]:
        method_owner_qualname: str | None = None
        if "." in func.__qualname__ and "<locals>" not in func.__qualname__:
            method_owner_qualname = func.__qualname__.rsplit(".", maxsplit=1)[0]

        @wraps(func)
        def wrapper(*args: _P.args, **kwargs: _P.kwargs) -> _R:
            stack_parts: list[str] = []
            caller_frame = sys._getframe(1)
            frame_index = 0
            while depth is None or frame_index < depth:
                stack_parts.append(format_stack_frame(caller_frame))
                if caller_frame.f_back is None:
                    break
                caller_frame = caller_frame.f_back
                frame_index += 1

            logger: _SupportsDebug = logging.getLogger()

            if method_owner_qualname is not None and args:
                first_arg = args[0]
                first_arg_type = first_arg if isinstance(first_arg, type) else type(first_arg)

                is_class_method = any(
                    cls.__module__ == func.__module__ and cls.__qualname__ == method_owner_qualname
                    for cls in first_arg_type.__mro__
                )
                if is_class_method:
                    logger_candidate = getattr(first_arg, "logger", None)
                    if callable(getattr(logger_candidate, "debug", None)):
                        logger = cast(_SupportsDebug, logger_candidate)

            logger.debug(
                "%s call stack: %s",
                func.__name__,
                " <- ".join(stack_parts),
            )
            return func(*args, **kwargs)

        return wrapper

    return decorator
