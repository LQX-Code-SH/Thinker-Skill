"""host 可执行入口：``ros2 run thinker_skill_core thinker_host``。

部署路径由环境变量决定（有默认值，便于开发机直接跑）：
- ``THINKER_CONFIG_DIR``  默认 ``configs``
- ``THINKER_SKILLS_DIR``  默认 ``skills``，支持多路径（``:`` 分隔）
- ``THINKER_ROBOT_DIR``   默认 ``robots/tienkung_dex``
"""
from __future__ import annotations

import os

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .app import bootstrap_ros


def _env_paths():
    config_dir = os.environ.get("THINKER_CONFIG_DIR", "configs")
    # 未设置时交给 bootstrap_ros 按 ament_index 解析 <model>_robot 的 share
    robot_dir = os.environ.get("THINKER_ROBOT_DIR") or None
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
    host, adapter, services, actions, mcp_server, skipped = bootstrap_ros(
        node,
        config_dir=config_dir,
        skills_dirs=skills_dirs,
        robot_dir=robot_dir,
    )
    mcp_url = None
    if mcp_server is not None:
        from thinker_common import ConfigLoader

        mcp_cfg = (ConfigLoader(config_dir).load().get("mcp") or {})
        if mcp_cfg.get("enabled", True):
            from .frontends.mcp import can_bind, start_http

            mcp_host = mcp_cfg.get("host", "localhost")
            mcp_port = int(mcp_cfg.get("port", 8000))
            if can_bind(mcp_host, mcp_port):
                start_http(mcp_server, host=mcp_host, port=mcp_port)
                mcp_url = f"http://{mcp_host}:{mcp_port}/mcp"
            else:
                node.get_logger().error(
                    f"mcp port {mcp_host}:{mcp_port} 已被占用，MCP 通道未启动")

    node.get_logger().info(
        f"thinker_host ready: services={len(services)} actions={len(actions)} "
        f"mcp={mcp_url or 'off'} "
        f"skills={sorted(host._registry.names())} skipped={skipped} "
        f"syspath_added={len(added)}"
    )
    # 多线程：技能执行（可重入组）不得阻塞 /robot_state 等订阅回调，
    # 否则 get_pose / is_moving 在整个执行期读到冻结的旧状态。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        adapter.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
