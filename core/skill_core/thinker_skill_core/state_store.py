"""语义状态存取（key 白名单）。

组合技能之间经此传递 holding / map_ready 等语义状态；未注册的 key 直接拒绝，
防止任意技能污染全局状态。
"""
from __future__ import annotations

import threading
from typing import Any, Dict, Iterable, Optional

# 默认白名单（与详细设计 4.5 一致）
DEFAULT_STATE_KEYS = {"holding", "holding_object", "map_ready", "last_detection"}


class StateKeyError(Exception):
    """访问未注册的 state key。"""


class StateStore:
    def __init__(self, allowed_keys: Optional[Iterable[str]] = None):
        self._lock = threading.Lock()
        self._data: Dict[str, Any] = {}
        self._allowed = set(allowed_keys) if allowed_keys is not None else None

    def _check(self, key: str) -> None:
        if self._allowed is not None and key not in self._allowed:
            raise StateKeyError(f"state key not registered: {key!r}")

    def get(self, key: str, default: Any = None) -> Any:
        self._check(key)
        with self._lock:
            return self._data.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._check(key)
        with self._lock:
            self._data[key] = value

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._data)
