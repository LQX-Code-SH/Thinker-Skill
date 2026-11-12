"""look_down_table 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。

docstring 即 LLM 可见的工具描述，改动签名必须同步改 docstring。
"""
from typing import Any, Dict


def look_down_table(pitch_deg: float = 45.0) -> Dict[str, Any]:
    """把头部 pitch 关节转到指定角度（低头看桌面），默认 45 度。

    Args:
        pitch_deg: 目标俯仰角（度），范围 -15~58，默认 45。
    """
    raise NotImplementedError("由 host 注入执行")
