"""frontends/service 测试（纯逻辑，不依赖 ROS）。"""
import json
from types import SimpleNamespace

from thinker_skill_core import Host, Registry, SkillRuntime, SkillSpec
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
