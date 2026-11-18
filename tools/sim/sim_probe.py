#!/usr/bin/env python3
"""robot_sim 话题探针：确认仿真已对齐全套真机接口。

真机/仿真共用 SDK real backend，因此这里只检查接口名是否齐全；
图像额外订阅一次确认有数据（BEST_EFFORT，与真机 Orbbec 一致）。

用法（Jazzy 容器内）：
    python3 tools/sim/sim_probe.py --check
"""
from __future__ import annotations

import argparse
import sys
import time


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="检查话题")
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args(argv)
    if not args.check:
        parser.print_help()
        return 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node("thinker_sim_probe")
    image_count = {"color": 0, "depth": 0}

    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(
        Image, "/ob_camera_head/color/image_raw",
        lambda _msg: image_count.__setitem__("color",
                                            image_count["color"] + 1), qos)
    node.create_subscription(
        Image, "/ob_camera_head/depth/image_raw",
        lambda _msg: image_count.__setitem__("depth",
                                            image_count["depth"] + 1), qos)

    deadline = time.monotonic() + max(0.5, args.seconds)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    available = {name for name, _ in node.get_topic_names_and_types()}
    required = [
        "/robot_state",
        "/arm/cmd", "/head/cmd", "/waist/cmd",
        "/ob_camera_head/color/image_raw",
        "/ob_camera_head/depth/image_raw",
        "/left_hand/angle_set", "/right_hand/angle_set",
    ]
    ok = True
    for topic in required:
        hit = topic in available
        print(f"{topic}: {'OK' if hit else 'MISSING'}")
        ok = ok and hit
    print("color frames:", image_count["color"],
          "depth frames:", image_count["depth"])
    if image_count["color"] == 0 or image_count["depth"] == 0:
        ok = False

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
