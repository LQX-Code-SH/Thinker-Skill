"""低头看桌面：把头部 pitch 关节转到指定角度（默认 50°）。

角度范围按 SDK ``joints.yaml`` 的 head_pitch 限位 [-15°, 58°] 夹紧。
"""
from __future__ import annotations

import math

from thinker_skill_sdk import Step, StepResult

DEFAULT_PITCH_DEG = 50.0
PITCH_MIN_RAD = math.radians(-15.0)   # 抬头极限
PITCH_MAX_RAD = math.radians(58.0)    # 低头极限


class LookDownTable:
    name = "look_down_table"
    requires = ["body.head"]
    postcondition = {}
    timeout = 10.0

    def build_steps(self, ctx, params):
        pitch_deg = float(params.get("pitch_deg", DEFAULT_PITCH_DEG))
        target = max(PITCH_MIN_RAD, min(PITCH_MAX_RAD, math.radians(pitch_deg)))

        tol = 0.02  # rad，约 1.1°

        def lower(c):
            # 幂等下发目标；未到位返回 running，下一 tick 再查
            c.body.head.look_at(yaw=0.0, pitch=target)
            _, cur_pitch = c.body.head.get_pose()
            if abs(cur_pitch - target) <= tol:
                return StepResult(
                    status="done",
                    data={
                        "pitch_deg": round(math.degrees(target), 2),
                        "actual_pitch_deg": round(math.degrees(cur_pitch), 2),
                    },
                )
            return StepResult(status="running")

        return [Step(name="低头看桌面", kind="motion", weight=1, run=lower, tick_budget=0.1)]
