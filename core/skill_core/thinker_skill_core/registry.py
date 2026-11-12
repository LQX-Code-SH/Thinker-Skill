"""技能发现与装配：扫描 skill.yaml → SkillSpec → 实例。"""
from __future__ import annotations

import importlib
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class SkillSpec:
    name: str
    version: str = "0.0.0"
    category: str = ""
    min_core: str = "0.0.0"
    entry: str = ""
    commands: List[str] = field(default_factory=list)
    requires: List[str] = field(default_factory=list)
    precondition: List[str] = field(default_factory=list)
    postcondition: Dict[str, Any] = field(default_factory=dict)
    timeout: Optional[float] = None
    interfaces: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    config: Optional[str] = None
    mcp: Dict[str, Any] = field(default_factory=dict)
    path: str = ""


def load_spec(yaml_path: str) -> SkillSpec:
    with open(yaml_path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    if "name" not in data:
        raise ValueError(f"skill.yaml missing 'name': {yaml_path}")
    return SkillSpec(
        name=data["name"],
        version=str(data.get("version", "0.0.0")),
        category=data.get("category", ""),
        min_core=str(data.get("min_core", "0.0.0")),
        entry=data.get("entry", ""),
        commands=list(data.get("commands", []) or []),
        requires=list(data.get("requires", []) or []),
        precondition=list(data.get("precondition", []) or []),
        postcondition=dict(data.get("postcondition", {}) or {}),
        timeout=data.get("timeout"),
        interfaces=list(data.get("interfaces", []) or []),
        errors=list(data.get("errors", []) or []),
        config=data.get("config"),
        mcp=dict(data.get("mcp", {}) or {}),
        path=yaml_path,
    )


def _current_roots(base: Path) -> List[Path]:
    """部署结构下的 ``*/current``（一或两层），存在则只扫它。"""
    roots: List[Path] = []
    for pattern in ("*/current", "*/*/current"):
        roots.extend(p for p in base.glob(pattern) if p.is_dir())
    return roots


def discover(search_dirs) -> List[SkillSpec]:
    specs: List[SkillSpec] = []
    seen = set()
    for root in search_dirs:
        base = Path(root)
        if not base.exists():
            continue
        # 有 current 时只扫 current：避免多版本共存时重复发现
        roots = _current_roots(base) or [base]
        for r in roots:
            for dirpath, _, files in os.walk(str(r), followlinks=True):
                if "skill.yaml" not in files:
                    continue
                path = os.path.realpath(os.path.join(dirpath, "skill.yaml"))
                if path in seen:
                    continue
                seen.add(path)
                try:
                    specs.append(load_spec(path))
                except Exception:
                    continue
    return specs


def instantiate(spec: SkillSpec) -> Any:
    module_name, _, class_name = spec.entry.partition(":")
    if not module_name or not class_name:
        raise ValueError(f"invalid entry: {spec.entry!r}")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)()


class Registry:
    def __init__(self):
        self._skills: Dict[str, Any] = {}
        self._specs: Dict[str, SkillSpec] = {}

    def register(self, spec: SkillSpec, instance: Any) -> None:
        self._skills[spec.name] = instance
        self._specs[spec.name] = spec

    def get(self, name: str) -> Any:
        return self._skills.get(name)

    def spec(self, name: str) -> Optional[SkillSpec]:
        return self._specs.get(name)

    def names(self) -> List[str]:
        return sorted(self._skills)

    def has(self, name: str) -> bool:
        return name in self._skills


def _version_lt(a: str, b: str) -> bool:
    def parse(v):
        return tuple(int(x) for x in str(v).replace("-", ".").split(".") if x.isdigit())
    try:
        return parse(a) < parse(b)
    except Exception:
        return False


def _check(spec: SkillSpec, min_core: Optional[str], capabilities) -> Optional[str]:
    if min_core and _version_lt(min_core, spec.min_core):
        return f"min_core {spec.min_core} > core {min_core}"
    if capabilities is not None:
        missing = [c for c in spec.requires if c not in capabilities]
        if missing:
            return f"missing capabilities: {missing}"
    return None


def build_registry(specs, *, min_core: Optional[str] = None,
                   capabilities=None):
    """校验 + 实例化，返回 ``(registry, skipped)``；不合格跳过并告警。"""
    reg = Registry()
    skipped = []
    for spec in specs:
        reason = _check(spec, min_core, capabilities)
        if reason:
            skipped.append((spec.name, reason))
            continue
        try:
            reg.register(spec, instantiate(spec))
        except Exception as exc:
            skipped.append((spec.name, f"instantiate: {exc}"))
    return reg, skipped
