"""fetch_object 场景技能测试（FakeBody + 假 detect_object）。"""
from thinker_skill_core import Host, Registry, SkillRuntime, SkillSpec
from thinker_skill_sdk import ErrorCode, Step, StepResult

from fakes import make_ctx

POSE = [0.5, 0.0, 0.72]


class _Detect:
    name = "detect_object"
    requires = []
    postcondition = {}
    timeout = 30.0

    def build_steps(self, ctx, params):
        return [Step(name="detect", run=lambda c: StepResult(
            status="done", data={"obj_pose": POSE}))]


def _host(with_detect=True):
    from fetch_object.plugin import FetchObject

    reg = Registry()
    if with_detect:
        reg.register(SkillSpec(name="detect_object"), _Detect())
    reg.register(SkillSpec(name="fetch_object"), FetchObject())
    ctx = make_ctx()
    return Host(registry=reg, runtime=SkillRuntime(),
                body=ctx.body, sensor=ctx.sensor), ctx


def test_fetch_ok():
    host, ctx = _host()
    r = host.execute("fetch_object", {})
    assert r.ok
    assert ctx.body.gripper.holding is True


def test_fetch_records_pose():
    host, _ = _host()
    r = host.execute("fetch_object", {})
    assert r.data["obj_pose"] == POSE


def test_missing_detect_skill_fails():
    host, _ = _host(with_detect=False)
    r = host.execute("fetch_object", {})
    assert r.state == "failed"
    assert r.code == ErrorCode.INPUT_UNSUPPORTED_COMMAND


def test_feedback_phases():
    host, _ = _host()
    phases = []
    host.execute("fetch_object", {}, on_feedback=lambda f: phases.append(f.phase))
    assert phases[0] == "识别"
    assert "抓取" in phases
