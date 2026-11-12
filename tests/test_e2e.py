"""端到端：技能发现 → 装配 → 执行 → 状态写回（无 ROS / 无真机）。"""
import textwrap

from thinker_skill_core import SkillRuntime, build_registry, discover

from fakes import make_ctx

SKILL_YAML = """
name: mock_skill
version: 1.0.0
category: mock
entry: mock_plugin:MockSkill
requires: [body.arm, body.gripper]
postcondition: {holding: true}
timeout: 5
"""

PLUGIN_PY = '''
from thinker_skill_sdk import Step, StepResult


class MockSkill:
    name = "mock_skill"
    requires = ["body.arm", "body.gripper"]
    postcondition = {"holding": True}
    timeout = 5.0

    def build_steps(self, ctx, params):
        def locate(c):
            return StepResult(status="done")

        def grasp(c):
            c.body.gripper.grasp(force=0.6)
            return StepResult(status="done")

        return [
            Step(name="locate", run=locate, kind="perception"),
            Step(name="grasp", run=grasp, kind="motion", weight=2),
        ]
'''


def _make_skill_dir(tmp_path, monkeypatch):
    d = tmp_path / "skills" / "mock"
    d.mkdir(parents=True)
    (d / "skill.yaml").write_text(textwrap.dedent(SKILL_YAML), encoding="utf-8")
    (d / "mock_plugin.py").write_text(textwrap.dedent(PLUGIN_PY), encoding="utf-8")
    monkeypatch.syspath_prepend(str(d))
    return str(tmp_path / "skills")


def test_end_to_end(tmp_path, monkeypatch):
    root = _make_skill_dir(tmp_path, monkeypatch)

    specs = discover([root])
    assert [s.name for s in specs] == ["mock_skill"]

    reg, skipped = build_registry(specs, capabilities={"body.arm", "body.gripper"})
    assert skipped == []
    skill = reg.get("mock_skill")
    assert skill is not None

    phases = []
    ctx = make_ctx()
    rt = SkillRuntime()
    result = rt.execute(skill, {}, ctx, on_feedback=lambda f: phases.append(f.phase))

    assert result.ok
    assert ctx.body.gripper.is_holding() is True          # 动作真的作用到 body
    assert rt._state.get("holding") is True               # postcondition 已写回
    assert phases[0] == "locate" and phases[-1] == "grasp"


def test_end_to_end_capability_mismatch(tmp_path, monkeypatch):
    root = _make_skill_dir(tmp_path, monkeypatch)
    specs = discover([root])
    reg, skipped = build_registry(specs, capabilities={"body.arm"})   # 缺 gripper
    assert reg.names() == []
    assert "missing" in skipped[0][1]
