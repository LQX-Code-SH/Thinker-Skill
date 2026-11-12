"""host 可执行入口：``ros2 run thinker_skill_core thinker_host``。

部署路径由环境变量决定（有默认值，便于开发机直接跑）：
- ``THINKER_CONFIG_DIR``  默认 ``configs``
- ``THINKER_SKILLS_DIR``  默认 ``skills``，支持多路径（``:`` 分隔）
- ``THINKER_ROBOT_DIR``   默认 ``robots/tienkung_dex``
"""
from __future__ import annotations

import os

import rclpy
from rclpy.node import Node

from .app import bootstrap_ros


def _env_paths():
    config_dir = os.environ.get("THINKER_CONFIG_DIR", "configs")
    robot_dir = os.environ.get("THINKER_ROBOT_DIR", "robots/tienkung_dex")
    skills_env = os.environ.get("THINKER_SKILLS_DIR", "skills")
    skills_dirs = [p for p in skills_env.split(os.pathsep) if p]
    return config_dir, robot_dir, skills_dirs


def _add_skill_site_packages(skills_dirs):
    """把技能独立 install 树的 site-packages 加入 sys.path。"""
    import pathlib
    import sys

    added = []
    for root in skills_dirs:
        base = pathlib.Path(root)
        if not base.exists():
            continue
        for current in sorted(base.glob("*/current")):
            install = current / "install"
            if not install.exists():
                continue
            for spp in install.glob("*/lib/python3.*/site-packages"):
                p = str(spp.resolve())
                if p not in sys.path:
                    sys.path.insert(0, p)
                    added.append(p)
    return added


def main(argv=None):
    config_dir, robot_dir, skills_dirs = _env_paths()
    added = _add_skill_site_packages(skills_dirs)

    rclpy.init(args=argv)
    node = Node("thinker_host")
    host, adapter, services, actions, skipped = bootstrap_ros(
        node,
        config_dir=config_dir,
        skills_dirs=skills_dirs,
        robot_dir=robot_dir,
    )
    node.get_logger().info(
        f"thinker_host ready: services={len(services)} actions={len(actions)} "
        f"skills={sorted(host._registry.names())} skipped={skipped} "
        f"syspath_added={len(added)}"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
