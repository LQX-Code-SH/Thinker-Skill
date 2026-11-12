"""host 启动装配：配置 → 机型适配器 → 技能 → frontends。"""
from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from thinker_common import ConfigLoader, ErrorCatalog

from .body_arbiter import BodyArbiter
from .host import Host
from .registry import Registry, build_registry, discover
from .runtime import SkillRuntime
from .state_store import DEFAULT_STATE_KEYS, StateStore


def _load_yaml(path: Path) -> Dict[str, Any]:
    import yaml
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def load_capabilities(robot_dir: str,
                      hand_variant: Optional[str] = None) -> Optional[set]:
    """合并「基础能力 + 手部变体能力」，返回能力名集合。"""
    path = Path(robot_dir) / "capabilities.yaml"
    if not path.exists():
        return None
    data = _load_yaml(path)
    caps = dict(data.get("capabilities") or {})
    if hand_variant:
        variant = (data.get("hand_variants") or {}).get(hand_variant) or {}
        caps.update(variant.get("capabilities") or {})
    return set(caps)


def load_robot_adapter(node, config: Dict[str, Any]):
    model = config.get("model", "tienkung_dex")
    module = importlib.import_module(f"thinker_robot_{model}")
    return module.create_adapter(
        node,
        backend=config.get("backend", "real"),
        hand_variant=config.get("hand_variant", "brainco"),
        hand_side=config.get("hand_side", "right"),
        urdf_path=config.get("urdf_path"),
        camera_k=(config.get("camera") or {}).get("k"),
        pelvis_height=float(config.get("pelvis_height", 0.0) or 0.0),
    )


def bootstrap_core(*, config_dir: str, skills_dirs: List[str], body: Any,
                   sensor: Any, min_core: str = "0.1.0",
                   capabilities=None) -> Tuple[Host, Registry, List]:
    """不依赖 ROS 的装配核心（可单测）。"""
    config = ConfigLoader(config_dir).load()
    errors_path = Path(config_dir) / "errors.yaml"
    catalog = ErrorCatalog(str(errors_path)) if errors_path.exists() else ErrorCatalog()

    specs = discover(skills_dirs)
    registry, skipped = build_registry(specs, min_core=min_core,
                                       capabilities=capabilities)

    tick_rate = float(config.get("tick_rate", 10))
    state = StateStore(DEFAULT_STATE_KEYS)
    runtime = SkillRuntime(arbiter=BodyArbiter(), state=state,
                           tick=1.0 / tick_rate, catalog=catalog)
    host = Host(registry=registry, runtime=runtime, body=body, sensor=sensor,
                state=state, config=config, tick_rate=tick_rate)
    return host, registry, skipped


def bootstrap_ros(node, *, config_dir: str, skills_dirs: List[str],
                  robot_dir: str, min_core: str = "0.1.0"):
    """完整装配：机型适配器 + 技能 + Service/Action frontends。"""
    from .frontends.action import create_actions
    from .frontends.service import create_services

    config = ConfigLoader(config_dir).load()
    adapter = load_robot_adapter(node, config)
    adapter.start()

    host, registry, skipped = bootstrap_core(
        config_dir=config_dir, skills_dirs=skills_dirs,
        body=adapter.body, sensor=adapter.sensor,
        min_core=min_core,
        capabilities=load_capabilities(robot_dir, config.get("hand_variant")))

    names = registry.names()
    services = create_services(node, host, names)
    actions = create_actions(node, host, names)
    return host, adapter, services, actions, skipped
