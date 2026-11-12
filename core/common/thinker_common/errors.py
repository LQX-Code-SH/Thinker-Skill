"""错误码 → 文案渲染。

技能只产出 ``code``；runtime 出口用本模块按 ``configs/errors.yaml`` 渲染 ``msg``。
"""
from __future__ import annotations

from typing import Dict, Optional, Tuple

_DEFAULT: Dict[str, Tuple[str, bool]] = {
    "OK": ("", False),
    "SYS_INTERNAL_ERROR": ("内部错误，请联系维护人员", True),
    "SYS_TIMEOUT": ("任务超时", True),
    "PRECOND_BODY_BUSY": ("身体正被其他任务占用，请稍后", True),
}


class ErrorCatalog:
    def __init__(self, path: Optional[str] = None):
        self._table: Dict[str, Tuple[str, bool]] = dict(_DEFAULT)
        if path:
            self._load(path)

    def _load(self, path: str) -> None:
        try:
            import yaml
        except Exception:
            return
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
        for code, entry in data.items():
            if isinstance(entry, dict):
                self._table[code] = (entry.get("msg", "") or "", bool(entry.get("retryable", False)))

    def render(self, code: str, **params) -> Tuple[str, bool]:
        """返回 ``(msg, retryable)``；占位符缺失时保留原文。"""
        msg, retryable = self._table.get(code, ("", False))
        if params and msg:
            try:
                msg = msg.format(**params)
            except (KeyError, IndexError, ValueError):
                pass
        return msg, retryable

    def __contains__(self, code: str) -> bool:
        return code in self._table
