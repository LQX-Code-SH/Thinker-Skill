"""有效空间过滤 + prompt 归一化（对齐参考实现 body.yaml）。"""
from __future__ import annotations

VALID_RANGE = {"x": (0.2, 0.8), "y": (-0.4, 0.4), "z": (0.4, 1.3)}

_ALIASES = {"toy": "plush toy"}


def normalize_prompt(name: str) -> str:
    name = (name or "bottle").strip().lower()
    return _ALIASES.get(name, name)


def in_valid_range(point) -> bool:
    x, y, z = float(point[0]), float(point[1]), float(point[2])
    return (
        VALID_RANGE["x"][0] < x < VALID_RANGE["x"][1]
        and VALID_RANGE["y"][0] < y < VALID_RANGE["y"][1]
        and VALID_RANGE["z"][0] < z < VALID_RANGE["z"][1]
    )
