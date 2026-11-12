"""skill_sdk 契约冒烟测试（无需 ROS / 真机）。"""
from thinker_skill_sdk import (
    ErrorCode,
    Pose,
    Step,
    StepCtx,
    StepResult,
    __version__,
)


def test_version():
    assert __version__ == "0.1.0"


def test_step_defaults():
    step = Step(name="approach", run=lambda ctx: StepResult(status="done"))
    assert step.kind == "motion"
    assert step.tick_budget == 0.1
    assert step.on_failure == "abort"
    assert step.retries == 0


def test_step_result_failed():
    r = StepResult(
        status="failed",
        code=ErrorCode.MOTION_GRASP_FAILED,
        retryable=True,
    )
    assert r.status == "failed"
    assert r.retryable is True
    assert r.data == {}


def test_pose_tuple():
    p = Pose(x=1.0, y=2.0, z=3.0)
    assert p.as_tuple() == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)


def test_error_code_prefix():
    assert ErrorCode.MOTION_GRASP_FAILED.startswith("MOTION_")
    assert ErrorCode.PRECOND_BODY_BUSY.startswith("PRECOND_")
    assert ErrorCode.OK == "OK"


def test_step_ctx_dataclass_fields():
    fields = set(StepCtx.__dataclass_fields__)
    assert {"skill", "params", "cfg", "body", "sensor", "state",
            "cancelled", "check_cancel", "now", "append_steps"} <= fields
