"""host 启动装配：配置 → 机型适配器 → 技能 → frontends。"""
from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from thinker_common import ConfigLoader, ErrorCatalog

from .body_arbiter import BodyArbiter
from .host import Host
from .registry import Registry, build_registry, discover, skill_root
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


def _resolve_robot_dir(explicit: Optional[str], model: str) -> Optional[str]:
    """解析机型目录：显式路径存在则用，否则按 ament_index 找 ``<model>_robot``。

    部署到机器人后通常没有 ``robots/`` 源码目录，此时应落到
    ``<install>/share/<model>_robot``（``capabilities.yaml`` 就在那里）。
    """
    if explicit:
        p = Path(explicit)
        if p.exists():
            return str(p)
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(f"{model}_robot")
    except Exception:
        return explicit


def load_robot_adapter(node, config: Dict[str, Any]):
    model = config.get("model", "tienkung_dex")
    module = importlib.import_module(f"{model}_robot")
    ik_cfg = config.get("ik") or {}
    ee_cfg = config.get("end_effector") or {}
    return module.create_adapter(
        node,
        backend=config.get("backend", "real"),
        hand_variant=config.get("hand_variant", "brainco"),
        hand_side=config.get("hand_side", "right"),
        urdf_path=config.get("urdf_path"),
        camera_k=(config.get("camera") or {}).get("k"),
        pelvis_height=float(config.get("pelvis_height", 0.0) or 0.0),
        ik_solver=ik_cfg.get("solver", "daqp"),
        ik_max_iter=int(ik_cfg.get("max_iter", 100)),
        ik_tol=float(ik_cfg.get("tol", 1e-3)),
        ik_damping=float(ik_cfg.get("damping", 0.08)),
        tcp_offset=ee_cfg.get("tcp_offset"),
        reach_margin=float(ik_cfg.get("reach_margin", 0.03)),
    )


def bootstrap_core(*, config_dir: str, skills_dirs: List[str], body: Any,
                   sensor: Any, min_core: str = "0.1.0",
                   capabilities=None, estop=None) -> Tuple[Host, Registry, List]:
    """不依赖 ROS 的装配核心（可单测）。"""
    config = ConfigLoader(config_dir).load()
    errors_path = Path(config_dir) / "errors.yaml"
    catalog = ErrorCatalog(str(errors_path)) if errors_path.exists() else ErrorCatalog()

    specs = discover(skills_dirs)
    registry, skipped = build_registry(specs, min_core=min_core,
                                       capabilities=capabilities)

    tick_rate = float(config.get("tick_rate", 10))
    state = StateStore(DEFAULT_STATE_KEYS)
    runtime = SkillRuntime(
        arbiter=BodyArbiter(), state=state, tick=1.0 / tick_rate,
        catalog=catalog, estop=estop,
        default_timeout=float(config.get("default_timeout", 300)))
    host = Host(registry=registry, runtime=runtime, body=body, sensor=sensor,
                state=state, config=config, tick_rate=tick_rate)
    return host, registry, skipped


def skill_root_dir(skill_yaml: str) -> Path:
    """兼容包装：见 :func:`thinker_skill_core.registry.skill_root`。"""
    return Path(skill_root(skill_yaml))


def bootstrap_ros(node, *, config_dir: str, skills_dirs: List[str],
                  robot_dir: Optional[str] = None, min_core: str = "0.1.0"):
    """完整装配：机型适配器 + 技能 + Service/Action frontends。"""
    from .frontends.action import create_actions
    from .frontends.service import create_services

    config = ConfigLoader(config_dir).load()
    robot_dir = _resolve_robot_dir(robot_dir, config.get("model", "tienkung_dex"))
    adapter = load_robot_adapter(node, config)
    adapter.start()

    host, registry, skipped = bootstrap_core(
        config_dir=config_dir, skills_dirs=skills_dirs,
        body=adapter.body, sensor=adapter.sensor,
        min_core=min_core,
        capabilities=load_capabilities(robot_dir, config.get("hand_variant")),
        estop=getattr(adapter, "emergency_stop", None))

    names = registry.names()
    services = create_services(node, host, names)
    actions = create_actions(node, host, names)

    # MCP 聚合器（可选：fastmcp 未安装时跳过，不影响 ROS 通道）
    mcp_server = None
    try:
        from .frontends.mcp import build_server
        pairs = []
        for name in names:
            spec = registry.spec(name)
            if spec is not None and spec.path:
                pairs.append((name, str(skill_root_dir(spec.path))))
        mcp_server = build_server(
            host, pairs,
            name=(config.get("mcp") or {}).get("name", "thinker_skills"))
    except Exception as exc:       # noqa: BLE001 - MCP 失败不应拖垮 ROS 通道
        skipped.append(f"mcp: {exc}")

    return host, adapter, services, actions, mcp_server, skipped
