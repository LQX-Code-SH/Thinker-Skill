"""状态存取抽象。

组合技能之间经 ``state_store`` 传递语义状态（holding / map_ready 等），
key 受白名单约束。
"""
from __future__ import annotations

from typing import Any, Protocol


class StateAPI(Protocol):
    def get(self, key: str, default: Any = None) -> Any: ...
    def set(self, key: str, value: Any) -> None: ...
