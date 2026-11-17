"""物体类别归一化与抓取参数合并。

``object_name`` 由上层（LLM / 编排器）传入，可能是英文、中文或带修饰的长句，
因此归一化放在技能内，配置里不堆别名。未识别名称一律落 ``generic``，不报错。
"""
from __future__ import annotations

from typing import Any, Dict, Mapping

CATEGORIES = ("bottle", "apple", "banana", "generic")

# 关键词 → 类别；按顺序匹配，越具体的放前面
_KEYWORDS = (
    ("banana", ("banana", "香蕉")),
    ("apple", ("apple", "苹果", "水果", "fruit")),
    ("bottle", ("bottle", "水瓶", "瓶子", "矿泉水", "饮料", "can", "罐")),
)


def normalize_object(name: Any) -> str:
    """``object_name`` → ``bottle | apple | banana | generic``。"""
    text = str(name or "").strip().lower()
    if not text:
        return "generic"
    for category, keys in _KEYWORDS:
        if any(key in text for key in keys):
            return category
    return "generic"


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def resolve_params(defaults: Mapping[str, Any], cfg: Mapping[str, Any],
                   category: str, params: Mapping[str, Any] | None = None
                   ) -> Dict[str, Any]:
    """四层合并：``defaults`` → 技能配置 → 类别覆写 → 调用覆写。

    - ``cfg`` 是 ``configs/robot.yaml`` 的 ``skills.fetch_object`` 段；
      其中 ``objects.<类别>`` 只作为该类别覆写，不并入通用参数；
    - ``params["overrides"]`` 供运行期调试局部覆盖（一次性快照，不改配置）。
    """
    cfg = dict(cfg or {})
    objects = dict(cfg.pop("objects", {}) or {})
    merged = _deep_merge(defaults, cfg)
    merged = _deep_merge(merged, objects.get(category) or {})
    overrides = (params or {}).get("overrides") or {}
    if isinstance(overrides, Mapping):
        merged = _deep_merge(merged, overrides)
    merged["category"] = category
    return merged
