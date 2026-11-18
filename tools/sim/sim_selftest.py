#!/usr/bin/env python3
"""在 robot_sim 容器内跑一次技能 smoke test。"""
from __future__ import annotations

import argparse
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from thinker_skill_core.app import bootstrap_ros


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skills", nargs="*", default=["look_down_table"])
    args = parser.parse_args(argv)

    rclpy.init()
    node = Node("thinker_sim_selftest")
    host, adapter, services, actions, mcp, skipped = bootstrap_ros(
        node,
        config_dir="/thinker/configs/sim",
        skills_dirs=["/thinker/skills"],
        robot_dir="/thinker/robots/tienkung_dex",
    )
    print("skills:", host._registry.names(), "skipped:", skipped)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        ok = True
        for name in args.skills:
            result = host.execute(name, {})
            print(name, result.state, result.code, result.data)
            ok = ok and result.ok
        return 0 if ok else 2
    finally:
        executor.shutdown()
        adapter.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
