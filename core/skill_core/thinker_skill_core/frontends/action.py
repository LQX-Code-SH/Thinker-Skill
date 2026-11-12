"""ROS Action frontend（Skill.action）。

每个技能一个 Action Server：``/thinker/skill/<skill>/action``；
goal 携带技能名（通用型），feedback 回传 phase/progress。
"""
from __future__ import annotations

import json
import threading
from typing import Any, Callable, Dict, Optional

ACTION_PREFIX = "/thinker/skill"


def _decode(text: Any) -> Dict[str, Any]:
    try:
        data = json.loads(text) if text else {}
    except (ValueError, TypeError):
        return {}
    return data if isinstance(data, dict) else {}


def handle_goal(host, skill: str, params_json: Any, *,
                on_feedback: Optional[Callable] = None,
                cancel_event: Optional[threading.Event] = None) -> Dict[str, Any]:
    """纯逻辑：执行一次 action goal（不依赖 ROS 类型，便于单测）。"""
    params = _decode(params_json)
    result = host.execute(skill, params, cancel_event=cancel_event,
                          on_feedback=on_feedback)
    return {
        "state": result.state,
        "code": result.code,
        "msg": result.msg,
        "retryable": bool(getattr(result, "retryable", False)),
        "data_json": json.dumps(result.data or {}, ensure_ascii=False),
    }


class SkillActionFrontend:
    def __init__(self, node, host, skill_name: str):
        from rclpy.action import ActionServer  # 延迟导入
        from rclpy.callback_groups import ReentrantCallbackGroup
        from thinker_interfaces.action import Skill

        self._host = host
        self._skill = skill_name
        self._cancel = threading.Event()
        self._Skill = Skill
        self._server = ActionServer(
            node, Skill, f"{ACTION_PREFIX}/{skill_name}/action",
            execute_callback=self._execute,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_cancel(self, goal_handle):
        from rclpy.action import CancelResponse
        self._cancel.set()
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        import asyncio
        loop = asyncio.get_event_loop()
        self._cancel.clear()
        Skill = self._Skill

        def _run():
            def on_fb(f):
                fb = Skill.Feedback()
                fb.phase = f.phase
                fb.progress = float(f.progress)
                fb.msg = f.msg
                goal_handle.publish_feedback(fb)
            return handle_goal(
                self._host, goal_handle.request.skill,
                goal_handle.request.params_json,
                on_feedback=on_fb, cancel_event=self._cancel)

        out = await loop.run_in_executor(None, _run)

        result = Skill.Result()
        result.state = out["state"]
        result.code = out["code"]
        result.msg = out["msg"]
        result.retryable = out["retryable"]
        result.data_json = out["data_json"]

        if out["state"] == "cancelled":
            goal_handle.canceled()
        elif out["state"] == "succeed":
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def create_actions(node, host, skill_names):
    return [SkillActionFrontend(node, host, name) for name in skill_names]
