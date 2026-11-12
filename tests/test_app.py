"""app 装配测试（bootstrap_core 不依赖 ROS）。"""
import textwrap

from thinker_skill_core.app import bootstrap_core, load_capabilities

from fakes import make_ctx

SKILL_YAML = """
name: demo
version: 1.0.0
entry: demo_plugin:DemoSkill
requires: [body.arm]
postcondition: {holding: true}
timeout: 5
"""

PLUGIN = '''
from thinker_skill_sdk import Step, StepResult


class DemoSkill:
    name = "demo"
    requires = ["body.arm"]
    postcondition = {"holding": True}
    timeout = 5.0

    def build_steps(self, ctx, params):
        return [Step(name="a", run=lambda c: StepResult(status="done"))]
'''


def _setup(tmp_path, monkeypatch):
    cfg = tmp_path / "configs"
    cfg.mkdir()
    (cfg / "main.yaml").write_text("tick_rate: 10\n", encoding="utf-8")
    (cfg / "errors.yaml").write_text("OK:\n  msg: ''\n  retryable: false\n",
                                     encoding="utf-8")
    skill = tmp_path / "skills" / "demo"
    skill.mkdir(parents=True)
    (skill / "skill.yaml").write_text(textwrap.dedent(SKILL_YAML), encoding="utf-8")
    (skill / "demo_plugin.py").write_text(textwrap.dedent(PLUGIN), encoding="utf-8")
    monkeypatch.syspath_prepend(str(skill))
    return str(cfg), [str(tmp_path / "skills")]


def test_bootstrap_core(tmp_path, monkeypatch):
    cfg, skills = _setup(tmp_path, monkeypatch)
    ctx = make_ctx()
    host, registry, skipped = bootstrap_core(
        config_dir=cfg, skills_dirs=skills, body=ctx.body, sensor=ctx.sensor)
    assert skipped == []
    assert registry.names() == ["demo"]
    assert host.execute("demo", {}).ok
    assert host._state.get("holding") is True   # 与 runtime 共享同一 state


def test_load_capabilities(tmp_path):
    d = tmp_path / "robot"
    d.mkdir()
    (d / "capabilities.yaml").write_text(textwrap.dedent("""
        robot_model: x
        capabilities:
          body.arm: {}
          sensor.rgbd: {}
    """), encoding="utf-8")
    assert load_capabilities(str(d)) == {"body.arm", "sensor.rgbd"}


def test_load_capabilities_merges_hand_variant(tmp_path):
    d = tmp_path / "robot"
    d.mkdir()
    (d / "capabilities.yaml").write_text(textwrap.dedent("""
        robot_model: x
        capabilities:
          body.arm: {}
        hand_variants:
          brainco:
            capabilities:
              body.gripper: {}
          fixed:
            capabilities: {}
    """), encoding="utf-8")
    assert load_capabilities(str(d), "brainco") == {"body.arm", "body.gripper"}
    assert load_capabilities(str(d), "fixed") == {"body.arm"}


def test_resolve_robot_dir_prefers_existing(tmp_path):
    from thinker_skill_core.app import _resolve_robot_dir

    d = tmp_path / "robot"
    d.mkdir()
    (d / "capabilities.yaml").write_text("capabilities: {}\n", encoding="utf-8")
    assert _resolve_robot_dir(str(d), "tienkung_dex") == str(d)


def test_resolve_robot_dir_falls_back_unchanged(tmp_path):
    from thinker_skill_core.app import _resolve_robot_dir

    missing = str(tmp_path / "nope")
    # ament 找不到 no_such_model_xyz_robot 时原样返回
    assert _resolve_robot_dir(missing, "no_such_model_xyz") == missing
