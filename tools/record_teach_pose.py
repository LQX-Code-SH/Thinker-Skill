#!/usr/bin/env python3
"""记录示教位姿：读真机当前关节角，算腕部与末端 TCP 在 base 系的位姿。

用法（source ROS 与技能安装空间后）：

    # 记录"预备抓取位姿"（腕 roll 末端口径）——打印 YAML 片段供粘贴
    python3 tools/record_teach_pose.py --part pregrasp --arm right --yaml
    # 记录"抓取位姿"（末端 TCP 抓取中心口径）——存 JSON
    python3 tools/record_teach_pose.py --part grasp --arm right --out grasp.json

关节角取自 ``/robot_state``（约 200Hz），正运动学用机型包 ``ArmIK.fk_pose``，
坐标系 base = pelvis + pelvis_height，与 ``body.get_tcp_pose()`` 完全同口径。
``--yaml`` 输出的字段可直接粘贴进 ``configs/robot.yaml`` 的
``skills.fetch_object.taught_poses``（脚本不改写配置文件）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in ("core/skill_sdk", "core/common", "robots/tienkung_dex"):
    sys.path.insert(0, os.path.join(ROOT, p))

ARM_IDS = {
    "right": (21, 22, 23, 24, 25, 26, 27),
    "left": (11, 12, 13, 14, 15, 16, 17),
}
WAIST_IDS = (31, 32, 33)  # pitch, roll, yaw


def _load_cfg(path):
    import yaml
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _read_state(arm_side, timeout, with_hand=False):
    import rclpy
    from ros2_bridge_msgs.msg import RobotState
    from inspire_hand_msgs.msg import GetAngleAct

    rclpy.init()
    node = rclpy.create_node("record_teach_pose")
    got = {"msg": None, "hand": None}

    def cb(msg):
        got["msg"] = msg

    node.create_subscription(RobotState, "/robot_state", cb, 10)
    if with_hand:
        node.create_subscription(
            GetAngleAct, f"/{arm_side}_hand/angle_actual",
            lambda m: got.__setitem__("hand", m), 10)
    deadline = time.time() + timeout
    while (got["msg"] is None or (with_hand and got["hand"] is None)) \
            and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    msg = got["msg"]
    if msg is None:
        raise RuntimeError(f"/robot_state 在 {timeout}s 内无消息")

    arm = {s.name: float(s.pos) for s in msg.arm.status}
    waist = {s.name: float(s.pos) for s in msg.waist.status}
    try:
        q = [arm[i] for i in ARM_IDS[arm_side]]
        wp = [waist[i] for i in WAIST_IDS]
    except KeyError as exc:
        raise RuntimeError(f"/robot_state 缺少关节 {exc}") from exc
    hand = None
    if with_hand and got["hand"] is not None:
        hand = {
            "joint_names": list(got["hand"].joint_names),
            "joint_values": [int(v) for v in got["hand"].joint_values],
        }
    return q, wp, hand


def build_record(cfg, arm_side):
    """读一帧关节角 → 返回完整位姿记录（纯计算，便于单测复用）。"""
    import numpy as np
    from tienkung_dex_robot.ik import ArmIK, matrix_to_quat
    from tienkung_dex_robot.kinematics import default_urdf_path

    pelvis_height = float(cfg.get("pelvis_height", 1.0))
    tcp_offset = (cfg.get("end_effector") or {}).get("tcp_offset") or [0, 0, 0]
    ik_cfg = cfg.get("ik") or {}
    q, wp, hand = _read_state(arm_side, timeout=5.0, with_hand=True)

    ik = ArmIK(
        default_urdf_path(), side=arm_side,
        base_offset=(0.0, 0.0, pelvis_height),
        tcp_offset=tcp_offset, solver=ik_cfg.get("solver", "dls"),
        tol=float(ik_cfg.get("tol", 1e-3)),
        rot_tol=float(ik_cfg.get("rot_tol", 0.02)),
        max_iter=int(ik_cfg.get("max_iter", 100)),
        damping=float(ik_cfg.get("damping", 0.08)),
    )
    p_wrist, R = ik.fk_pose(q, tuple(wp))
    quat = [float(v) for v in matrix_to_quat(R)]
    p_tcp = p_wrist + R @ np.asarray(tcp_offset, dtype=float)

    return {
        "side": arm_side,
        "frame": "base (pelvis + pelvis_height)",
        "stamp": round(time.time(), 3),
        "pelvis_height": pelvis_height,
        "tcp_offset": [float(v) for v in tcp_offset],
        "joint_ids": list(ARM_IDS[arm_side]),
        "joints_rad": [round(float(v), 6) for v in q],
        "waist_pry_rad": [round(float(v), 6) for v in wp],
        "wrist_position": [round(float(v), 6) for v in p_wrist],
        "tcp_position": [round(float(v), 6) for v in p_tcp],
        "quat_xyzw": [round(v, 6) for v in quat],
        # 抓取前手型（inspire 手 13 关节实际角度，原始值）
        "pregrasp_hand": hand,
    }


def _yaml_entry(rec, part):
    """把记录转成可粘贴进 taught_poses.<part> 的 YAML 字典。"""
    entry = {
        "joints_rad": rec["joints_rad"],
        "waist_pry_rad": rec["waist_pry_rad"],
        "quat_xyzw": rec["quat_xyzw"],
    }
    if part == "grasp":
        entry["tcp_position"] = rec["tcp_position"]
        entry["wrist_position"] = rec["wrist_position"]
    else:  # pregrasp：腕 roll 末端口径 + 抓取前手型
        entry["wrist_position"] = rec["wrist_position"]
        entry["tcp_position"] = rec["tcp_position"]
        if rec.get("pregrasp_hand"):
            entry["pregrasp_hand"] = rec["pregrasp_hand"]
    return entry


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--part", choices=("pregrasp", "grasp"),
                    default="pregrasp", help="记录哪一段位姿（默认 pregrasp）")
    ap.add_argument("--arm", choices=("right", "left"), default="right")
    ap.add_argument("--config", default=os.path.join(ROOT, "configs", "robot.yaml"))
    ap.add_argument("--out", help="JSON 输出文件（默认打印到 stdout）")
    ap.add_argument("--yaml", action="store_true",
                    help="打印可粘贴进 robot.yaml 的 YAML 片段")
    args = ap.parse_args(argv)

    rec = build_record(_load_cfg(args.config), args.arm)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(rec, fh, indent=2, ensure_ascii=False)
        print("saved:", args.out)

    if args.yaml:
        import yaml
        print(f"# 粘贴到 skills.fetch_object.taught_poses.{args.part}:")
        print(yaml.safe_dump({args.part: _yaml_entry(rec, args.part)},
                             allow_unicode=True, sort_keys=False))
    if not args.out and not args.yaml:
        print(json.dumps(rec, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
