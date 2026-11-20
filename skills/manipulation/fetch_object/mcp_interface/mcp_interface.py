"""fetch_object 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。

docstring 即 LLM 可见的工具描述；改动签名必须同步改 docstring。
"""
from typing import Any, Dict


def fetch_object(object_name: str = "bottle", arm: str = "auto") -> Dict[str, Any]:
    """抓取桌面上的指定物体，送到机器人持物位。

    一次调用完成：低头看桌 → 识别定位 → 选臂 → 接近 → 抓取 → 提拉 → 视觉确认 →
    收到持物位。成功后可继续调用放置/递物类技能。

    适用于简单规则物体（水瓶/易拉罐、苹果、香蕉、杯子、纸盒）。不支持需要
    判断朝向的物体（如马克杯把手定向）或薄片边缘抓取。

    调用前无需手动摆头部姿态，技能内部会先低头；执行期间机器人上肢被占用，
    失败时返回可读原因，`retryable` 为真表示可以重试一次。

    Args:
        object_name: 目标物体名称（如 bottle / can / apple / banana / 水瓶 / 苹果 / 香蕉），
            默认 bottle。未识别的名称按通用规则处理。
        arm: 用哪只手抓，auto 按目标在身体左侧还是右侧自动选择；也可强制
            left / right。默认 auto。
    """
    raise NotImplementedError("由 host 注入执行")
