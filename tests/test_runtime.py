"""runtime + state_store 测试。"""
import pytest

from thinker_skill_core import SkillRuntime, StateKeyError, StateStore
from thinker_skill_sdk import ErrorCode, Step, StepResult

from fakes import make_ctx


class _Skill:
    def __init__(self, name="fetch_object", requires=("body.arm",),
                 steps=None, postcondition=None, timeout=1.0):
        self.name = name
        self.requires = list(requires)
        self._steps = steps or [Step(name="a", run=lambda c: StepResult(status="done"))]
        self.postcondition = postcondition or {}
        self.timeout = timeout

    def build_steps(self, ctx, params):
        return list(self._steps)


def test_success_and_postcondition():
    state = StateStore({"holding"})
    rt = SkillRuntime(state=state)
    r = rt.execute(_Skill(postcondition={"holding": True}), {}, make_ctx())
    assert r.ok
    assert state.get("holding") is True


def test_capability_missing():
    rt = SkillRuntime(capabilities={"body.head"})
    r = rt.execute(_Skill(requires=["body.arm"]), {}, make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.PRECOND_CAPABILITY_MISSING


def test_body_busy():
    rt = SkillRuntime()
    rt._arbiter.acquire("other", ["body.arm"])
    r = rt.execute(_Skill(), {}, make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.PRECOND_BODY_BUSY


def test_readonly_skill_no_lease():
    rt = SkillRuntime()
    r = rt.execute(_Skill(requires=["sensor.rgbd"]), {}, make_ctx())
    assert r.ok


def test_failed_propagates():
    def fail(c):
        return StepResult(status="failed", code=ErrorCode.MOTION_GRASP_FAILED)

    rt = SkillRuntime()
    r = rt.execute(_Skill(steps=[Step(name="x", run=fail)]), {}, make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.MOTION_GRASP_FAILED


def test_postcondition_not_written_on_failure():
    state = StateStore({"holding"})

    def fail(c):
        return StepResult(status="failed", code="X")

    rt = SkillRuntime(state=state)
    rt.execute(_Skill(steps=[Step(name="x", run=fail)],
                      postcondition={"holding": True}), {}, make_ctx())
    assert state.get("holding") is None


def test_build_steps_exception():
    class Bad:
        name = "bad"
        requires = []
        timeout = 1.0
        postcondition = {}

        def build_steps(self, ctx, params):
            raise RuntimeError("boom")

    rt = SkillRuntime()
    r = rt.execute(Bad(), {}, make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.SYS_INTERNAL_ERROR


def test_state_store_whitelist():
    s = StateStore({"holding"})
    s.set("holding", True)
    assert s.get("holding") is True
    with pytest.raises(StateKeyError):
        s.set("bogus", 1)
