"""host 测试。"""
import threading

from thinker_skill_core import Host, Registry, SkillRuntime, SkillSpec
from thinker_skill_sdk import ErrorCode, Step, StepResult

from fakes import make_ctx


class _Skill:
    name = "s"
    requires = []
    postcondition = {}
    timeout = 1.0

    def build_steps(self, ctx, params):
        return [Step(name="a", run=lambda c: StepResult(status="done"))]


def _host(runtime=None):
    reg = Registry()
    reg.register(SkillSpec(name="s"), _Skill())
    ctx = make_ctx()
    return Host(registry=reg, runtime=runtime or SkillRuntime(),
                body=ctx.body, sensor=ctx.sensor)


def test_unknown_skill():
    r = _host().execute("nope", {})
    assert r.state == "failed"
    assert r.code == ErrorCode.INPUT_UNSUPPORTED_COMMAND


def test_execute_ok():
    assert _host().execute("s", {}).ok


def test_cancel_event():
    class Slow:
        name = "slow"
        requires = []
        postcondition = {}
        timeout = 5.0

        def build_steps(self, ctx, params):
            return [Step(name="a", run=lambda c: StepResult(status="running"),
                         timeout=0.2)]

    reg = Registry()
    reg.register(SkillSpec(name="slow"), Slow())
    ctx = make_ctx()
    h = Host(registry=reg, runtime=SkillRuntime(tick=0.001),
             body=ctx.body, sensor=ctx.sensor)
    ev = threading.Event()
    ev.set()
    assert h.execute("slow", {}, cancel_event=ev).state == "cancelled"


class _Leaf:
    name = "leaf"
    requires = []
    postcondition = {}
    timeout = 1.0

    def build_steps(self, ctx, params):
        return [Step(name="leaf", run=lambda c: StepResult(
            status="done", data={"value": params.get("x", 0)}))]


class _Scene:
    name = "scene"
    requires = []
    postcondition = {}
    timeout = 5.0

    def build_steps(self, ctx, params):
        def run(c):
            res = c.call_skill("leaf", {"x": 42})
            if res.state != "succeed":
                return StepResult(status="failed", code=res.code)
            return StepResult(status="done", data={"got": res.data["value"]})

        return [Step(name="scene", run=run)]


def test_call_skill():
    reg = Registry()
    reg.register(SkillSpec(name="leaf"), _Leaf())
    reg.register(SkillSpec(name="scene"), _Scene())
    ctx = make_ctx()
    host = Host(registry=reg, runtime=SkillRuntime(),
                body=ctx.body, sensor=ctx.sensor)
    r = host.execute("scene", {})
    assert r.ok
    assert r.data["got"] == 42
