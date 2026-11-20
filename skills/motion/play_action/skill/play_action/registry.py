"""动作清单（``actions.yaml``）解析与索引。

清单是技能私有的**声明式索引**：把"动作名 / 别名 / 标签"映射到实际文件，并
声明适用机型、末端类型、默认参数与所需能力。它只参与"选择与校验"，不改变
执行路径；没有清单时 :mod:`play_action.plugin` 完全回退到按路径搜索，行为与
旧版本一致。

``version: 1`` 结构::

    version: 1
    actions:
      goready:
        file: goready.json          # 相对 actions_dir / <skill>/actions / cwd
        clip: goready               # 可选：默认片段
        display_name: 就绪
        aliases: [ready]            # 别名（也可直接用 display_name 查找）
        tags: [greeting]
        models: [tienkung_dex]      # 空表示不限机型
        hand_variants: [inspire, brainco]   # 空表示不限末端
        hand_mode: auto             # 默认手部口径：auto | 6d | 13d
        defaults: {speed: 1.0, transition_s: 2.0}
        requires: [body.arm.left, body.arm.right, body.waist, body.head]
        on_mismatch: strict         # strict（默认，失败）| warn（放行并告警）
        verified: {env: real, date: 2026-11-18}
        safety: {max_speed: 1.5}

简写形式：``goready: goready.json``（只给文件名，其余取默认）。

本模块纯 Python（只依赖 PyYAML），不 import ROS / 机型包，可离机单测。
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Mapping, Optional, Tuple

try:  # host 环境已依赖 PyYAML；缺失时给出可读错误而不是 ImportError
    import yaml
except ImportError:  # pragma: no cover - 仅缺依赖的环境
    yaml = None

REGISTRY_FILENAME = "actions.yaml"
SCHEMA_VERSION = 1
_HAND_MODES = ("", "auto", "6d", "13d")
_MISMATCH_POLICIES = ("strict", "warn")


class ActionRegistryError(ValueError):
    """清单结构非法 / 版本不支持 / 别名冲突 / 文件缺失。"""


def _as_tuple(value: Any) -> Tuple[str, ...]:
    if value is None:
        return ()
    if isinstance(value, str):
        return (value,) if value.strip() else ()
    if isinstance(value, Iterable):
        return tuple(str(v).strip() for v in value if str(v).strip())
    return (str(value),)


def _as_mapping(value: Any, what: str) -> Dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ActionRegistryError(f"{what} 必须是对象")
    return dict(value)


def _as_float(value: Any, what: str) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ActionRegistryError(f"{what} 不是数值: {value!r}") from exc


@dataclass(frozen=True)
class ActionEntry:
    """一条动作注册项。"""

    key: str
    file: str = ""
    clip: str = ""
    display_name: str = ""
    tags: Tuple[str, ...] = ()
    aliases: Tuple[str, ...] = ()
    models: Tuple[str, ...] = ()
    hand_variants: Tuple[str, ...] = ()
    hand_mode: str = ""
    speed: Optional[float] = None
    transition_s: Optional[float] = None
    requires: Tuple[str, ...] = ()
    on_mismatch: str = "strict"
    verified: Mapping[str, Any] = field(default_factory=dict)
    safety: Mapping[str, Any] = field(default_factory=dict)


class ActionRegistry:
    """动作清单索引：按 key / display_name / alias 解析。"""

    def __init__(self, entries: Mapping[str, ActionEntry], path: str = ""):
        self.path = path
        self._entries: Dict[str, ActionEntry] = dict(entries)
        self._index: Dict[str, ActionEntry] = {}
        for key, entry in self._entries.items():
            for name in (key, entry.display_name, *entry.aliases):
                text = str(name or "").strip()
                if not text:
                    continue
                existing = self._index.get(text)
                if existing is not None and existing.key != entry.key:
                    raise ActionRegistryError(
                        f"动作清单别名冲突: {text!r} 同时指向 "
                        f"{existing.key!r} 与 {entry.key!r}")
                self._index[text] = entry

    def __bool__(self) -> bool:
        return bool(self._entries)

    def __len__(self) -> int:
        return len(self._entries)

    @property
    def entries(self) -> Dict[str, ActionEntry]:
        return dict(self._entries)

    def resolve(self, ref: str) -> Optional[ActionEntry]:
        """按动作名 / 别名 / display_name 查找；未注册返回 ``None``。"""
        return self._index.get(str(ref or "").strip())


def load_registry(path: str) -> ActionRegistry:
    """读取并校验清单；失败抛 :class:`ActionRegistryError`。"""
    if yaml is None:  # pragma: no cover - 仅缺依赖的环境
        raise ActionRegistryError("缺少 PyYAML，无法解析动作清单")
    try:
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
    except OSError as exc:
        raise ActionRegistryError(f"动作清单无法读取: {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ActionRegistryError(f"动作清单不是合法 YAML: {path}: {exc}") from exc

    if not isinstance(doc, Mapping):
        raise ActionRegistryError("动作清单根节点必须是对象")

    version = doc.get("version", SCHEMA_VERSION)
    try:
        version_num = int(version)
    except (TypeError, ValueError) as exc:
        raise ActionRegistryError(f"动作清单 version 非法: {version!r}") from exc
    if version_num != SCHEMA_VERSION:
        raise ActionRegistryError(
            f"不支持的动作清单版本 version={version_num}"
            f"（仅支持 {SCHEMA_VERSION}）")

    raw_actions = doc.get("actions") or {}
    if not isinstance(raw_actions, Mapping):
        raise ActionRegistryError("动作清单 actions 必须是对象")

    entries: Dict[str, ActionEntry] = {}
    for key, raw in raw_actions.items():
        name = str(key).strip()
        if not name:
            continue
        if isinstance(raw, str):              # 简写：goready: goready.json
            raw = {"file": raw}
        if not isinstance(raw, Mapping):
            raise ActionRegistryError(
                f"动作 {name!r} 的定义必须是对象或文件名字符串")

        defaults = _as_mapping(raw.get("defaults"), f"{name}.defaults")
        hand_mode = str(raw.get("hand_mode") or "").strip().lower()
        if hand_mode not in _HAND_MODES:
            raise ActionRegistryError(
                f"动作 {name!r} 的 hand_mode 非法: {hand_mode!r}"
                f"（可选 {[m for m in _HAND_MODES if m]}）")
        policy = str(raw.get("on_mismatch") or "strict").strip().lower()
        if policy not in _MISMATCH_POLICIES:
            raise ActionRegistryError(
                f"动作 {name!r} 的 on_mismatch 非法: {policy!r}"
                f"（可选 {list(_MISMATCH_POLICIES)}）")

        entry = ActionEntry(
            key=name,
            file=str(raw.get("file") or ""),
            clip=str(raw.get("clip") or ""),
            display_name=str(raw.get("display_name") or name),
            tags=_as_tuple(raw.get("tags")),
            aliases=_as_tuple(raw.get("aliases")),
            models=_as_tuple(raw.get("models")),
            hand_variants=_as_tuple(raw.get("hand_variants")),
            hand_mode=hand_mode,
            speed=_as_float(raw.get("speed", defaults.get("speed")),
                            f"{name}.speed"),
            transition_s=_as_float(
                raw.get("transition_s", defaults.get("transition_s")),
                f"{name}.transition_s"),
            requires=_as_tuple(raw.get("requires")),
            on_mismatch=policy,
            verified=_as_mapping(raw.get("verified"), f"{name}.verified"),
            safety=_as_mapping(raw.get("safety"), f"{name}.safety"),
        )
        if not entry.file:
            raise ActionRegistryError(f"动作 {name!r} 缺少 file")
        entries[name] = entry

    return ActionRegistry(entries, path=path)


def resolve_registry_path(skill_dir: Optional[str] = None,
                          actions_dir: Optional[str] = None,
                          env_var: str = "THINKER_ACTIONS_DIR") -> Optional[str]:
    """按动作搜索顺序找 ``actions.yaml``；找不到返回 ``None``（合法状态）。"""
    dirs = []
    if actions_dir:
        dirs.append(os.path.expanduser(str(actions_dir)))
    if skill_dir:
        dirs.append(os.path.join(skill_dir, "actions"))
        dirs.append(skill_dir)
    env_dir = os.environ.get(env_var)
    if env_dir:
        dirs.append(os.path.expanduser(env_dir))
    for base in dirs:
        candidate = os.path.join(base, REGISTRY_FILENAME)
        if os.path.isfile(candidate):
            return candidate
    return None
