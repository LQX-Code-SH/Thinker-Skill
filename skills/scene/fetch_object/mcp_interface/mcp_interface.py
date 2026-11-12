"""fetch_object 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。"""
from typing import Any, Dict


def fetch_object(object_name: str = "bottle") -> Dict[str, Any]:
    """抓取指定物体并抬起到持物姿态。

    Args:
        object_name: 物体名称（如 bottle / cup），默认 bottle。
    """
    raise NotImplementedError("由 host 注入执行")
