"""detect_object 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。

docstring 即 LLM 可见的工具描述，改动签名必须同步改 docstring。
"""
from typing import Any, Dict


def detect_object(object_name: str = "bottle") -> Dict[str, Any]:
    """识别指定物体，返回其在机器人基座坐标系下的 3D 位置（单位：米）。

    只读取当前相机画面，不控制头部；调用前请先摆好头部姿态（如先调用
    look_down_table）。

    Args:
        object_name: 物体名称（如 bottle / cup / woven basket），默认 bottle。
    """
    raise NotImplementedError("由 host 注入执行")
