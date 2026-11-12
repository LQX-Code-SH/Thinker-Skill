"""配置分层加载：``main.yaml`` 的 includes 递归合并（后者覆盖前者）。"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict


def _load_yaml(path: Path) -> Dict[str, Any]:
    import yaml
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


class ConfigLoader:
    def __init__(self, base_dir: str):
        self._base = Path(base_dir)

    def load(self, name: str = "main.yaml") -> Dict[str, Any]:
        data = _load_yaml(self._base / name)
        includes = data.pop("includes", []) or []
        merged: Dict[str, Any] = {}
        for inc in includes:
            merged = _deep_merge(merged, _load_yaml(self._base / inc))
        return _deep_merge(merged, data)
