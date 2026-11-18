#!/usr/bin/env python3
"""打印仿真基座位姿：/manip/base_pose -> x/y/z + roll/pitch/yaw。

消息是 std_msgs/Float64MultiArray：[x, y, z, qw, qx, qy, qz]（米 / 四元数）。

用法（容器内，桥在跑）：
    python3 tools/sim/sim_pose.py            # 一直打印
    python3 tools/sim/sim_pose.py --count 1  # 只打印一帧
"""
from __future__ import annotations

import argparse
import math
import sys


def quat_to_rpy(w: float, x: float, y: float, z: float):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="打印 /manip/base_pose")
    parser.add_argument("--topic", default="/manip/base_pose")
    parser.add_argument("--count", type=int, default=0,
                        help="打印几帧后退出；0 = 一直打印")
    args = parser.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray

    rclpy.init()
    node = Node("sim_pose_echo")
    state = {"n": 0}

    def _on_msg(msg) -> None:
        data = list(msg.data)
        if len(data) < 7:
            return
        x, y, z, w, qx, qy, qz = data[:7]
        roll, pitch, yaw = quat_to_rpy(w, qx, qy, qz)
        print(f"x={x:+.3f} y={y:+.3f} z={z:+.3f} "
              f"yaw={yaw:+.2f}deg roll={roll:+.2f} pitch={pitch:+.2f}",
              flush=True)
        state["n"] += 1

    node.create_subscription(Float64MultiArray, args.topic, _on_msg, 10)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.count and state["n"] >= args.count:
                break
        return 0 if state["n"] else 2
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
