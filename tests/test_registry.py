"""registry 测试。"""
import textwrap

import pytest

from thinker_skill_core.registry import (
    Registry,
    SkillSpec,
    _version_lt,
    build_registry,
    discover,
    load_spec,
)


def _write(path, content):
    path.write_text(textwrap.dedent(content), encoding="utf-8")
    return path


def test_load_spec(tmp_path):
    p = _write(tmp_path / "skill.yaml", """
        name: fetch_object
        version: 1.0.0
        requires: [body.arm, body.gripper]
        timeout: 120
        entry: foo:Bar
    """)
    spec = load_spec(str(p))
    assert spec.name == "fetch_object"
    assert spec.requires == ["body.arm", "body.gripper"]
    assert spec.timeout == 120


def test_load_spec_missing_name(tmp_path):
    p = _write(tmp_path / "skill.yaml", "version: 1.0\n")
    with pytest.raises(ValueError):
        load_spec(str(p))


def test_discover(tmp_path):
    d = tmp_path / "skills" / "fetch"
    d.mkdir(parents=True)
    _write(d / "skill.yaml", "name: fetch\n")
    specs = discover([str(tmp_path / "skills")])
    assert [s.name for s in specs] == ["fetch"]


def test_build_registry_min_core(tmp_path):
    d = tmp_path / "s" / "a"
    d.mkdir(parents=True)
    _write(d / "skill.yaml", "name: a\nmin_core: 2.0.0\n")
    reg, skipped = build_registry(discover([str(tmp_path / "s")]), min_core="1.0.0")
    assert reg.names() == []
    assert skipped and "min_core" in skipped[0][1]


def test_build_registry_capability(tmp_path):
    d = tmp_path / "s" / "b"
    d.mkdir(parents=True)
    _write(d / "skill.yaml", "name: b\nrequires: [body.arm, body.base]\n")
    reg, skipped = build_registry(discover([str(tmp_path / "s")]),
                                  capabilities={"body.arm"})
    assert reg.names() == []
    assert "missing" in skipped[0][1]


def test_version_lt():
    assert _version_lt("1.0.0", "2.0.0")
    assert not _version_lt("2.0.0", "1.0.0")


def test_registry_register_get():
    reg = Registry()
    spec = SkillSpec(name="x")
    reg.register(spec, object())
    assert reg.has("x")
    assert reg.spec("x") is spec
