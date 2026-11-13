"""MCP frontend 测试：工具加载 / 绑定 / 注册（不依赖 fastmcp）。"""
import inspect
import math
import os

from thinker_skill_core import Host, Registry, SkillRuntime, SkillSpec
from thinker_skill_core.frontends.mcp import bind_tool, load_tools, register_tools
from thinker_skill_sdk import Step, StepResult

from fakes import make_ctx

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKILL_DIR = os.path.join(ROOT, "skills", "manipulation", "look_down_table")


class _Skill:
    name = "look_down_table"
    requires = []
    postcondition = {}
    timeout = 1.0

    def build_steps(self, ctx, params):
        pitch = float(params.get("pitch_deg", 50))

        def run(c):
            c.body.head.look_at(yaw=0.0, pitch=math.radians(pitch))
            return StepResult(status="done", data={"pitch": pitch})

        return [Step(name="a", run=run)]


def _host():
    reg = Registry()
    reg.register(SkillSpec(name="look_down_table"), _Skill())
    ctx = make_ctx()
    return Host(registry=reg, runtime=SkillRuntime(),
                body=ctx.body, sensor=ctx.sensor)


def test_load_tools_from_skill_dir():
    assert [t.__name__ for t in load_tools(SKILL_DIR)] == ["look_down_table"]


def test_tool_signature_and_doc():
    tool = load_tools(SKILL_DIR)[0]
    assert "低头" in tool.__doc__
    assert "pitch_deg" in inspect.signature(tool).parameters


def test_bind_and_call():
    bound = bind_tool(_host(), "look_down_table", load_tools(SKILL_DIR)[0])
    assert "pitch_deg" in inspect.signature(bound).parameters
    out = bound(pitch_deg=30.0)
    assert out["state"] == "succeed"
    assert out["data"]["pitch"] == 30.0


class _FakeServer:
    def __init__(self):
        self.fns = []

    def tool(self):
        def deco(fn):
            self.fns.append(fn)
            return fn
        return deco


def test_register_tools():
    server = _FakeServer()
    assert register_tools(server, [lambda: None]) == 1
    assert len(server.fns) == 1


def test_missing_skill_dir_returns_empty(tmp_path):
    assert load_tools(tmp_path) == []


def test_can_bind_detects_busy_port():
    import socket

    from thinker_skill_core.frontends.mcp import can_bind

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    sock.listen(1)
    port = sock.getsockname()[1]
    try:
        assert can_bind("localhost", port) is False
    finally:
        sock.close()
    assert can_bind("localhost", port) is True


def test_build_server_without_fastmcp():
    import importlib.util

    import pytest

    if importlib.util.find_spec("fastmcp") is not None:
        pytest.skip("fastmcp 已安装")

    from thinker_skill_core.frontends.mcp import build_server

    with pytest.raises(ModuleNotFoundError):
        build_server(_host(), [("look_down_table", SKILL_DIR)])
