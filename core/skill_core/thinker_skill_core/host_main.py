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


def main(argv=None):
    config_dir, robot_dir, skills_dirs = _env_paths()

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
        f"skills={sorted(host._registry.names())} skipped={skipped}"
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
