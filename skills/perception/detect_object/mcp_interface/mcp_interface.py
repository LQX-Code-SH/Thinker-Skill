"""detect_object 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。

docstring 即 LLM 可见的工具描述，改动签名必须同步改 docstring。
"""
from typing import Any, Dict


def detect_object(object_name: str = "bottle",
                  head_pitch_deg: float = 50.0) -> Dict[str, Any]:
    """识别指定物体，返回其在机器人基座坐标系下的 3D 位置（单位：米）。

    Args:
        object_name: 物体名称（如 bottle / cup），默认 bottle。
        head_pitch_deg: 低头角度（度），默认 50；看桌面约 50。
    """
    raise NotImplementedError("由 host 注入执行")
