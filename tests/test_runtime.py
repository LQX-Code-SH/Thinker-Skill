"""runtime + state_store 测试。"""
import threading
import time

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


def test_watchdog_fires_releases_lease_and_maps_timeout():
    """Step.run 阻塞导致 Pipeline 无法按 tick 检查 deadline 时，看门狗兜底。"""
    estops = []
    rt = SkillRuntime(estop=lambda: estops.append(1))

    def blocking(c):
        time.sleep(0.5)
        return StepResult(status="running")

    skill = _Skill(steps=[Step(name="block", run=blocking)], timeout=0.1)
    out = {}
    worker = threading.Thread(
        target=lambda: out.setdefault("r", rt.execute(skill, {}, make_ctx())))
    worker.start()
    time.sleep(0.25)          # 0.1 × 1.2 = 0.12s，此时应已软急停并释放
    lease = rt._arbiter.acquire("other", ["body.arm"])
    assert lease is not None, "看门狗未释放身体租约"
    rt._arbiter.release(lease)
    worker.join(3.0)

    assert estops, "看门狗未触发软急停"
    r = out["r"]
    assert r.state == "failed"
    assert r.code == ErrorCode.SYS_TIMEOUT
    assert rt._arbiter.holder("upper") is None


def test_watchdog_not_fired_for_fast_skill():
    estops = []
    rt = SkillRuntime(estop=lambda: estops.append(1))
    r = rt.execute(_Skill(timeout=1.0), {}, make_ctx())
    assert r.ok
    assert estops == []


def test_msg_rendered_from_catalog():
    import os

    from thinker_common import ErrorCatalog

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    catalog = ErrorCatalog(os.path.join(root, "configs", "errors.yaml"))

    def fail(c):
        return StepResult(status="failed", code="PERCEPTION_OBJECT_NOT_FOUND",
                          data={"object_name": "瓶子"})

    rt = SkillRuntime(catalog=catalog)
    r = rt.execute(_Skill(steps=[Step(name="x", run=fail)]), {}, make_ctx())
    assert "瓶子" in r.msg
