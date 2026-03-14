from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class MethodCall:
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]


@dataclass(slots=True)
class MethodCallChain:
    calls: list[MethodCall] = field(default_factory=list)

    def record(self, name: str, *args: Any, **kwargs: Any) -> None:
        self.calls.append(MethodCall(name=name, args=args, kwargs=kwargs))

    def names(self) -> list[str]:
        return [call.name for call in self.calls]

    def clear(self) -> None:
        self.calls.clear()
