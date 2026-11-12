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
