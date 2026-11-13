#!/usr/bin/env python3
"""按站姿关节角核算 pelvis 离地高度（URDF 正运动学 + 足底碰撞圆柱）。

用法：
    python3 tools/measure_pelvis_height.py                     # 默认读 joints.yaml 的 home_standing
    python3 tools/measure_pelvis_height.py --pose home2
    python3 tools/measure_pelvis_height.py --live              # 读真机 /robot_state 当前角度
    python3 tools/measure_pelvis_height.py --json

前置：source ROS 与工作空间 install（导入 tienkung_dex_robot 需要 thinker_skill_sdk）。
"""
from __future__ import annotations

import argparse
import json
import sys
import time

from tienkung_dex_robot.leg_kinematics import (
    LEG_JOINTS,
    compute_pelvis_height,
    default_paths,
    load_pose,
)


def _read_live(timeout: float = 5.0) -> dict:
    import rclpy
    from rclpy.node import Node
    from ros2_bridge_msgs.msg import RobotState

    rclpy.init()
    node = Node("measure_pelvis_height")
    raw = {}

    def _cb(msg):
        if raw:
            return
        for item in getattr(msg, "leg").status:
            raw[int(item.name)] = float(item.pos)

    node.create_subscription(RobotState, "/robot_state", _cb, 10)
    deadline = time.time() + timeout
    while not raw and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if not raw:
        raise SystemExit("未收到 /robot_state")
    return raw


def main(argv=None):
    urdf_default, joints_default = default_paths()
    parser = argparse.ArgumentParser(description="核算 pelvis 离地高度")
    parser.add_argument("--pose", default="home_standing", help="joints.yaml 中的姿态名")
    parser.add_argument("--joints", default=joints_default, help="joints.yaml 路径")
    parser.add_argument("--urdf", default=urdf_default, help="URDF 路径")
    parser.add_argument("--live", action="store_true", help="读取真机 /robot_state 当前角度")
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    parser.add_argument("-v", "--verbose", action="store_true", help="打印每个关节角")
    args = parser.parse_args(argv)

    if args.live:
        angles = _read_live()
        source = "live:/robot_state"
    else:
        angles = load_pose(args.joints, args.pose)
        source = f"{args.joints}:{args.pose}"

    if not angles:
        raise SystemExit(f"姿态为空: {source}")

    result = compute_pelvis_height(args.urdf, angles)
    result["source"] = source
    result["angles"] = {str(k): v for k, v in sorted(angles.items())}

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0

    print(f"source: {source}")
    if args.verbose:
        for jid in sorted(angles):
            name = next((n for n, i in LEG_JOINTS.items() if i == jid), "")
            print(f"  {jid:3d} {name:24s} {angles[jid]:+.4f} rad")
    for side, foot in result["feet"].items():
        print(f"{side:5s} ankle_z={foot['ankle_z']:+.6f}  "
              f"sole_z={foot['sole_z']:+.6f}")
    height = result["pelvis_height"]
    print(f"\npelvis_height = {height:.6f} m = {height * 1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
