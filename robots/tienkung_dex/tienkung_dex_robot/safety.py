"""机型安全层：关节限幅 + 急停收敛。

SDK 本身不做限幅（``joints.yaml`` 的 limits 只被示例使用），适配层必须夹紧。
"""
from __future__ import annotations

from typing import Mapping, Optional, Tuple


def clamp(value: float, limits: Optional[Tuple[float, float]]) -> float:
    """按 (lower, upper) 夹紧；limits 为 None 时原样返回。"""
    if limits is None:
        return float(value)
    lo, hi = limits
    if lo > hi:
        lo, hi = hi, lo
    return max(float(lo), min(float(hi), float(value)))


def clamp_many(joints: Mapping[str, float],
               limits: Mapping[str, Tuple[float, float]]) -> dict:
    """按关节名批量夹紧（未知关节不夹）。"""
    return {name: clamp(value, limits.get(name))
            for name, value in joints.items()}


class EstopActive(RuntimeError):
    """急停已触发（SDK 的 ``EstopActiveError`` 在此收敛）。"""


ESTOP_ERROR_NAME = "EstopActiveError"


def call_with_estop(fn, *args, **kwargs):
    """调用 SDK；把 ``EstopActiveError`` 收敛为 ``EstopActive``。"""
    try:
        return fn(*args, **kwargs)
    except Exception as exc:      # noqa: BLE001 - 只按类名精确匹配
        if type(exc).__name__ == ESTOP_ERROR_NAME:
            raise EstopActive(str(exc)) from exc
        raise
