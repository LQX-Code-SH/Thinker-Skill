"""fetch_object：场景技能，编排 detect_object + 手臂抓取。

设计期以 ``exec_depend`` 声明依赖；运行期经 ``ctx.call_skill`` 调用其他技能，
**禁止 import 其他技能源码**（D-006）。
"""
from __future__ import annotations

from thinker_skill_sdk import ErrorCode, Step, StepResult

DEFAULT_APPROACH = {
    "right_shoulder_pitch": 0.2,
    "right_elbow_pitch": -0.6,
}


class FetchObject:
    name = "fetch_object"
    requires = ["body.arm", "body.gripper"]
    postcondition = {"holding": True}
    timeout = 120.0

    def build_steps(self, ctx, params):
        object_name = params.get("object_name", "bottle")
        approach = dict(params.get("approach_joints", DEFAULT_APPROACH))
        st = {"pose": None}

        def detect(c):
            if c.call_skill is None:
                return StepResult(
                    status="failed", code=ErrorCode.SYS_CONFIG_ERROR,
                    data={"reason": "当前 host 不支持技能调用"})
            res = c.call_skill("detect_object", {"object_name": object_name})
            if res.state != "succeed":
                return StepResult(status="failed", code=res.code,
                                  retryable=res.retryable, data=res.data)
            st["pose"] = res.data.get("obj_pose")
            return StepResult(status="done", data={"obj_pose": st["pose"]})

        def approach_arm(c):
            c.body.arm.move_to_joints(approach)
            return StepResult(status="done")

        def grasp(c):
            c.body.gripper.grasp(force=0.6)
            if not c.body.gripper.is_holding():
                return StepResult(status="failed",
                                  code=ErrorCode.MOTION_GRASP_FAILED,
                                  retryable=True)
            return StepResult(status="done")

        def lift(c):
            c.body.arm.move_to_joints({k: 0.0 for k in approach})
            return StepResult(status="done")

        return [
            Step(name="识别", kind="perception", weight=2,
                 tick_budget=5.0, run=detect),
            Step(name="接近", kind="motion", weight=2, run=approach_arm),
            Step(name="抓取", kind="motion", weight=3, retries=1, run=grasp),
            Step(name="抬起", kind="motion", weight=1,
                 on_failure="skip", run=lift),
        ]
