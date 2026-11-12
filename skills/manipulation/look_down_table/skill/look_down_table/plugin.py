"""低头看桌面：把头部 pitch 关节转到指定角度（默认 45°）。

角度范围按 SDK ``joints.yaml`` 的 head_pitch 限位 [-15°, 58°] 夹紧；
真机实测低头稳态上限约 47.5°（目标 50/58 均停在 0.829 rad），
故支持 ``max_pitch_deg`` 软上限与 ``tol_deg`` 到位容差（均来自技能私有配置）。
"""
from __future__ import annotations

import math

from thinker_skill_sdk import Step, StepResult

DEFAULT_PITCH_DEG = 45.0
DEFAULT_TOL_DEG = 3.0
PITCH_MIN_RAD = math.radians(-15.0)   # 抬头极限
PITCH_MAX_RAD = math.radians(58.0)    # SDK 表低头极限（真机不可达）


class LookDownTable:
    name = "look_down_table"
    requires = ["body.head"]
    postcondition = {}
    timeout = 10.0

    def build_steps(self, ctx, params):
        cfg = dict(ctx.cfg or {})
        requested_deg = float(params.get("pitch_deg", DEFAULT_PITCH_DEG))

        # 真机软上限：机械/负载导致的稳态可达角小于 SDK 目标限位
        soft_max_rad = min(
            PITCH_MAX_RAD, math.radians(float(cfg.get("max_pitch_deg", 58.0))))
        target = max(PITCH_MIN_RAD, min(soft_max_rad, math.radians(requested_deg)))

        # 到位容差 + 连续稳定判据：单 tick 命中可能在运动中提前判完成
        tol_deg = float(params.get("tol_deg", cfg.get("tol_deg", DEFAULT_TOL_DEG)))
        tol = math.radians(max(0.1, tol_deg))
        settle_ticks = max(1, int(cfg.get("settle_ticks", 3)))
        stable = {"n": 0}

        def lower(c):
            # 幂等下发目标；未到位返回 running，下一 tick 再查
            c.body.head.look_at(yaw=0.0, pitch=target)
            _, cur_pitch = c.body.head.get_pose()
            err = abs(cur_pitch - target)
            if err <= tol:
                stable["n"] += 1
                if stable["n"] >= settle_ticks:
                    return StepResult(
                        status="done",
                        data={
                            "pitch_deg": round(math.degrees(target), 2),
                            "requested_pitch_deg": round(requested_deg, 2),
                            "actual_pitch_deg": round(math.degrees(cur_pitch), 2),
                        },
                    )
            else:
                stable["n"] = 0
            return StepResult(status="running")

        return [Step(name="低头看桌面", kind="motion", weight=1, run=lower, tick_budget=0.1)]
