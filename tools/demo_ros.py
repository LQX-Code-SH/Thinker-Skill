#!/usr/bin/env python3
"""ROS 集成演示 / 自测。

起一个 Node，装配 mock 技能 + FakeBody，暴露 Service/Action；
``--selftest`` 模式下自己调一次并打印结果（无需真机）。

用法：
    source /opt/ros/humble/setup.bash
    source .colcon/install/setup.bash
    python3 tools/demo_ros.py --selftest
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("core/skill_sdk", "core/skill_core", "core/common", "robots/tienkung_dex", "tests"):
    sys.path.insert(0, os.path.join(ROOT, p))

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402

from thinker_skill_core.app import bootstrap_core  # noqa: E402
from thinker_skill_core.frontends.service import create_services  # noqa: E402
from fakes import make_ctx  # noqa: E402


def main(selftest=False):
    rclpy.init()
    node = Node("thinker_demo")
    ctx = make_ctx()
    host, registry, _ = bootstrap_core(
        config_dir=os.path.join(ROOT, "configs"),
        skills_dirs=[os.path.join(ROOT, "skills")],
        body=ctx.body, sensor=ctx.sensor)
    create_services(node, host, registry.names())
    print("skills:", registry.names())

    if not selftest:
        rclpy.spin(node)
        return 0

    from thinker_interfaces.srv import ControlCommand

    name = registry.names()[0]
    client = node.create_client(ControlCommand, f"/thinker/skill/{name}/command")
    if not client.wait_for_service(timeout_sec=5.0):
        print("service not ready")
        return 1

    req = ControlCommand.Request()
    req.command = "fetch_bottle"
    req.params_json = "{}"
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    res = future.result()
    print("state:", res.state, "| code:", res.code, "| msg:", res.msg)
    print("data:", res.data_json)
    ok = res.state == "succeed"
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(selftest="--selftest" in sys.argv))
