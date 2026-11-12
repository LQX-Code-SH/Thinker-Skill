"""frontends/service 测试（纯逻辑，不依赖 ROS）。"""
import json
import threading
from types import SimpleNamespace

from thinker_skill_core import Host, Registry, SkillRuntime, SkillSpec
from thinker_skill_core.frontends.action import handle_goal
from thinker_skill_core.frontends.service import _decode, handle_request
from thinker_skill_sdk import Step, StepResult

from fakes import make_ctx


class _Skill:
    name = "s"
    requires = []
    postcondition = {}
    timeout = 1.0

    def build_steps(self, ctx, params):
        return [Step(name="a", run=lambda c: StepResult(status="done"))]


def _host():
    reg = Registry()
    reg.register(SkillSpec(name="s"), _Skill())
    ctx = make_ctx()
    return Host(registry=reg, runtime=SkillRuntime(), body=ctx.body, sensor=ctx.sensor)


def test_decode():
    assert _decode('{"a": 1}') == {"a": 1}
    assert _decode("") == {}
    assert _decode("not json") == {}
    assert _decode("[1, 2]") == {}


def test_handle_request_ok():
    out = handle_request(_host(), "s", SimpleNamespace(params_json='{"x": 1}'))
    assert out["state"] == "succeed"
    assert out["code"] == "OK"
    assert json.loads(out["data_json"])["progress"] == 1.0


def test_handle_request_unknown_skill():
    out = handle_request(_host(), "nope", SimpleNamespace(params_json="{}"))
    assert out["state"] == "failed"
    assert out["code"] == "INPUT_UNSUPPORTED_COMMAND"


def _slow_host():
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
    return Host(registry=reg, runtime=SkillRuntime(tick=0.001),
                body=ctx.body, sensor=ctx.sensor)


def test_handle_goal_ok():
    out = handle_goal(_host(), "s", '{"x": 1}')
    assert out["state"] == "succeed"
    assert out["code"] == "OK"


def test_handle_goal_feedback():
    phases = []
    handle_goal(_host(), "s", "{}", on_feedback=lambda f: phases.append(f.phase))
    # 每步开始与结束各反馈一次
    assert phases and all(p == "a" for p in phases)


def test_handle_goal_cancel():
    ev = threading.Event()
    ev.set()
    out = handle_goal(_slow_host(), "slow", "{}", cancel_event=ev)
    assert out["state"] == "cancelled"


def test_register_tools_mock():
    from thinker_skill_core.frontends.mcp import register_tools

    class FakeServer:
        def __init__(self):
            self.fns = []

        def tool(self):
            def deco(fn):
                self.fns.append(fn)
                return fn
            return deco

    server = FakeServer()
    count = register_tools(server, [lambda: None, lambda: None])
    assert count == 2
    assert len(server.fns) == 2


def test_tool_result_envelope():
    from thinker_skill_core.frontends.mcp import tool_result

    class R:
        state = "succeed"
        code = "OK"
        msg = ""
        retryable = False
        data = {"a": 1}

    out = tool_result(R())
    assert out == {"state": "succeed", "code": "OK", "msg": "",
                   "retryable": False, "data": {"a": 1}}


