"""ROS Service frontend（ControlCommand 调试通道）。

服务名：``/thinker/skill/<skill>/command``
"""
from __future__ import annotations

import json
from typing import Any, Dict, List

SERVICE_PREFIX = "/thinker/skill"


def _decode(text: Any) -> Dict[str, Any]:
    try:
        data = json.loads(text) if text else {}
    except (ValueError, TypeError):
        return {}
    return data if isinstance(data, dict) else {}


def handle_request(host, skill_name: str, request: Any) -> Dict[str, Any]:
    """纯逻辑处理：不依赖 ROS 消息类型，便于单测。"""
    params = _decode(getattr(request, "params_json", ""))
    result = host.execute(skill_name, params)
    return {
        "state": result.state,
        "code": result.code,
        "msg": result.msg,
        "retryable": bool(getattr(result, "retryable", False)),
        "data_json": json.dumps(result.data or {}, ensure_ascii=False),
    }


class CommandServiceFrontend:
    def __init__(self, node, host, skill_name: str):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from thinker_interfaces.srv import ControlCommand  # 延迟导入（需 ROS 环境）
        self._host = host
        self._skill = skill_name
        # 可重入组：技能执行期间不阻塞 /robot_state 等订阅回调
        self._srv = node.create_service(
            ControlCommand,
            f"{SERVICE_PREFIX}/{skill_name}/command",
            self._on_request,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_request(self, request, response):
        out = handle_request(self._host, self._skill, request)
        response.state = out["state"]
        response.code = out["code"]
        response.msg = out["msg"]
        response.retryable = out["retryable"]
        response.data_json = out["data_json"]
        return response


def create_services(node, host, skill_names: List[str]) -> List[CommandServiceFrontend]:
    return [CommandServiceFrontend(node, host, name) for name in skill_names]
