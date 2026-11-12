"""mock 抓取技能：不碰真机，演示 Step 编排与 BodyAPI 调用。"""
from __future__ import annotations

from thinker_skill_sdk import Step, StepResult


class FetchObjectMock:
    name = "fetch_object_mock"
    requires = ["body.arm", "body.gripper"]
    postcondition = {"holding": True}
    timeout = 30.0

    def build_steps(self, ctx, params):
        def locate(c):
            return StepResult(status="done")

        def approach(c):
            c.body.arm.move_to_joints({"right_shoulder_pitch": 0.2})
            return StepResult(status="done")

        def grasp(c):
            c.body.gripper.grasp(force=0.6)
            return StepResult(status="done")

        def back(c):
            c.body.arm.move_to_joints({"right_shoulder_pitch": 0.0})
            return StepResult(status="done")

        return [
            Step(name="感知定位", kind="perception", weight=2, run=locate),
            Step(name="接近", kind="motion", weight=1, run=approach),
            Step(name="抓取", kind="motion", weight=2, retries=1, run=grasp),
            Step(name="回收", kind="motion", weight=1, on_failure="skip", run=back),
        ]
