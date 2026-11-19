"""把 robot_sim 的 ROS 话题包装成 SDK robot 的鸭子类型。

TienkungDexBody / TienkungDexSensor 只依赖 robot.arm/waist/head/hand_*/cameras
这几个属性；这里用同名方法提供同样的语义，从而复用真机的关节表、限位、IK
与手部映射。

桥侧话题（robot_sim v3 + HEAD 补丁）：
    /arm|waist|head/cmd_ctrl   bodyctrl_msgs/CmdMotorCtrl
    /arm|waist|head/status     bodyctrl_msgs/MotorStatusMsg
    /left|right_hand/angle_set|angle_actual   inspire_hand_msgs
    /ob_camera_head/{color,depth}/image_raw + camera_info + pose
"""
from __future__ import annotations

import threading
from typing import Any, Mapping, Optional, Sequence

from ..body import TienkungDexHead
from ..joints import HEAD_JOINTS

from . import plant


def _body_msgs():
    from bodyctrl_msgs.msg import CmdMotorCtrl, MotorCtrl, MotorStatusMsg
    return CmdMotorCtrl, MotorCtrl, MotorStatusMsg


def _inspire_msgs():
    from inspire_hand_msgs.msg import GetAngleAct, SetAngle
    return SetAngle, GetAngleAct


class SimJointGroup:
    """一个 /{group}/cmd_ctrl + /{group}/status 关节组。

    以 ``hz`` 心跳重发目标（手臂默认 100 Hz，与 body 层伺服同频；腰/头 20 Hz
    足够）。桥有 0.25 s 看门狗，停发会掉软阻抗。
    """

    def __init__(self, node, topic: str, ids: Sequence[int],
                 kp: float, kd: float, hz: float = 20.0):
        CmdMotorCtrl, MotorCtrl, MotorStatusMsg = _body_msgs()
        self._node = node
        self._CmdMotorCtrl = CmdMotorCtrl
        self._MotorCtrl = MotorCtrl
        self._ids = tuple(int(i) for i in ids)
        self._targets: dict[int, float] = {}
        self._states: dict[int, plant.JointState] = {}
        self._lock = threading.Lock()
        self._kp = float(kp)
        self._kd = float(kd)
        self._pub = node.create_publisher(CmdMotorCtrl, f"{topic}/cmd_ctrl", 10)
        self._sub = node.create_subscription(
            MotorStatusMsg, f"{topic}/status", self._on_status, 20)
        # streaming：只有技能执行期间才 20 Hz 重发目标。技能结束后停发，
        # 桥侧看门狗 0.25 s 后会撤掉软阻抗（这正是"动作播完就停"的语义）。
        self._streaming = True
        self._timer = node.create_timer(1.0 / max(float(hz), 1.0), self._flush)

    def _on_status(self, msg) -> None:
        with self._lock:
            for s in msg.status:
                self._states[int(s.name)] = plant.JointState(
                    pos=float(s.pos), vel=float(s.speed),
                    effort=float(s.current))

    def set_streaming(self, on: bool) -> None:
        """开关心跳重发：False = 不再发保持指令（技能结束后由 Host 调用）。"""
        self._streaming = bool(on)

    def _flush(self) -> None:
        if not self._streaming:
            return
        with self._lock:
            targets = dict(self._targets)
        if not targets:
            return
        msg = self._CmdMotorCtrl()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.cmds = []
        for jid, pos in targets.items():
            c = self._MotorCtrl()
            c.name = int(jid)
            c.kp = self._kp
            c.kd = self._kd
            c.pos = float(pos)
            c.spd = 0.0
            c.tor = 0.0
            if hasattr(c, "cur"):
                c.cur = 0.0
            msg.cmds.append(c)
        self._pub.publish(msg)

    def move_to(self, joints: Mapping[int, float],
                cur: Optional[float] = None) -> None:
        with self._lock:
            self._targets.update({int(k): float(v) for k, v in joints.items()})
        self._flush()

    def get_states(self) -> dict[int, plant.JointState]:
        with self._lock:
            return dict(self._states)

    def get_pose(self):
        """腰/头这类组按关节顺序返回当前位置。"""
        states = self.get_states()
        return tuple(states[j].pos if j in states else 0.0
                     for j in self._ids)


class SimHand:
    """inspire 手：angle_set 下发、angle_actual 回读（0.1°）。"""

    def __init__(self, node, topic: str, hand_id: int):
        SetAngle, GetAngleAct = _inspire_msgs()
        self._node = node
        self._SetAngle = SetAngle
        self._hand_id = int(hand_id)
        self._values: list[int] = [0] * 13
        self._lock = threading.Lock()
        self._pub = node.create_publisher(SetAngle, f"{topic}/angle_set", 10)
        self._sub = node.create_subscription(
            GetAngleAct, f"{topic}/angle_actual", self._on_status, 10)

    def _on_status(self, msg) -> None:
        with self._lock:
            self._values = [int(v) for v in msg.joint_values][:13]

    def set_joint_values(self, values: Sequence[int]) -> None:
        vals = [int(v) for v in values][:13]
        if len(vals) != 13:
            raise ValueError(f"inspire 手需要 13 维，实际 {len(vals)}")
        with self._lock:
            self._values = list(vals)
        msg = self._SetAngle()
        msg.hand_id = self._hand_id
        msg.joint_values = vals
        self._pub.publish(msg)

    def get_joint_values(self) -> list[int]:
        with self._lock:
            return list(self._values)

    def set_positions(self, values: Sequence[int]) -> None:
        self.set_joint_values(values)

    def get_status(self):
        return type("HandStatus", (), {"positions": self.get_joint_values()})()


class SimCameraStream:
    """robot.cameras[name].latest() 的最小实现。"""

    def __init__(self, node, prefix: str):
        import numpy as np
        from rclpy.qos import (QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy)
        from sensor_msgs.msg import CameraInfo, Image

        self._np = np
        self._lock = threading.Lock()
        self._frame = None
        self._k = None
        qos = QoSProfile(depth=2, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        prefix = str(prefix).rstrip("/")
        node.create_subscription(Image, f"{prefix}/color/image_raw",
                                 self._on_color, qos)
        node.create_subscription(Image, f"{prefix}/depth/image_raw",
                                 self._on_depth, qos)
        node.create_subscription(CameraInfo, f"{prefix}/color/camera_info",
                                 self._on_info, 2)

    def _on_color(self, msg) -> None:
        np = self._np
        h, w = int(msg.height), int(msg.width)
        ch = 3 if msg.encoding in ("rgb8", "bgr8") else 1
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size != h * w * ch:
            return
        image = arr.reshape(h, w, ch) if ch == 3 else arr.reshape(h, w)
        image = (plant.rgb_to_bgr(image) if msg.encoding == "rgb8"
                 else np.ascontiguousarray(image))
        with self._lock:
            frame = self._frame or {}
            frame.update({"color": image, "stamp": msg.header.stamp,
                          "frame_id": msg.header.frame_id or ""})
            self._frame = frame

    def _on_depth(self, msg) -> None:
        np = self._np
        h, w = int(msg.height), int(msg.width)
        dtype = np.float32 if "32F" in msg.encoding.upper() else np.uint16
        arr = np.frombuffer(msg.data, dtype=dtype)
        if arr.size != h * w:
            return
        depth = plant.depth_to_mm(arr.reshape(h, w), msg.encoding)
        with self._lock:
            frame = self._frame or {}
            frame.update({"depth": depth})
            self._frame = frame

    def _on_info(self, msg) -> None:
        with self._lock:
            self._k = [float(v) for v in msg.k]

    @property
    def k(self):
        with self._lock:
            return self._k

    def latest(self):
        with self._lock:
            f = self._frame
            if not f or "color" not in f or "depth" not in f:
                return None
            data = dict(f)
        return type("CameraFrame", (), data)()


class SimRobot:
    """扮演 SDK robot：arm / waist / head / hand_left|right / cameras。"""

    def __init__(self, node, mapping: Mapping[str, Any]):
        topics = mapping.get("topics") or {}
        gains = mapping.get("gains") or {}
        arm = gains.get("arm") or {}
        waist = gains.get("waist") or {}
        head = gains.get("head") or {}
        # 手臂下发频率与 body 层伺服（100 Hz）对齐，否则中间目标会被心跳丢弃
        arm_hz = float(arm.get("control_hz", 100.0))

        self.arm = SimJointGroup(node, topics.get("arm", "/arm"),
                                 plant.ARM_IDS,
                                 arm.get("kp", 100.0), arm.get("kd", 7.4),
                                 hz=arm_hz)
        self.waist = SimJointGroup(node, topics.get("waist", "/waist"),
                                   plant.WAIST_IDS,
                                   waist.get("kp", 120.0), waist.get("kd", 8.0))
        self.head = SimJointGroup(node, topics.get("head", "/head"),
                                  plant.HEAD_IDS,
                                  head.get("kp", 40.0), head.get("kd", 2.0))

        hand = (mapping.get("hand") or {}).get("inspire") or {}
        hand_ids = hand.get("hand_id") or {}
        hand_topics = topics.get("hand") or {}
        self.hand_left = SimHand(node, hand_topics.get("left", "/left_hand"),
                                 hand_ids.get("left", 1))
        self.hand_right = SimHand(node, hand_topics.get("right", "/right_hand"),
                                  hand_ids.get("right", 2))

        camera_prefix = topics.get("camera", "/ob_camera_head")
        self.cameras = {"ob_camera_head": SimCameraStream(node, camera_prefix)}
        self.walk = None

    def start(self) -> None:
        """节点由 host 持有并已在 spin，这里无事可做。"""

    def shutdown(self) -> None:
        pass

    def set_streaming(self, on: bool) -> None:
        """统一开关心跳：arm / waist / head 三类命令流。"""
        for group in (self.arm, self.waist, self.head):
            group.set_streaming(on)


class SimHead(TienkungDexHead):
    """头部：把真机 pitch 语义映射到 MJCF 的符号/零点。"""

    def __init__(self, robot, cfg: Mapping[str, Any]):
        super().__init__(robot)
        self._cfg = dict(cfg or {})

    def _sign(self, key: str, default: float) -> float:
        return float(self._cfg.get(key, default))

    def look_at(self, *, yaw: float, pitch: float) -> None:
        self._require()
        yaw_sign = self._sign("yaw_sign", 1.0)
        pitch_sign = self._sign("pitch_sign", 1.0)
        offset = self._sign("pitch_offset", 0.0)
        ids = {
            HEAD_JOINTS["head_yaw"]: plant.clamp(
                yaw * yaw_sign, self._cfg.get("yaw_limits", [-1.0472, 1.0472])),
            HEAD_JOINTS["head_pitch"]: plant.clamp(
                pitch * pitch_sign + offset,
                self._cfg.get("pitch_limits", [-0.9599, 0.2618])),
        }
        self._robot.head.move_to(ids)

    def get_pose(self):
        self._require()
        states = self._robot.head.get_states()
        jyaw = HEAD_JOINTS["head_yaw"]
        jpitch = HEAD_JOINTS["head_pitch"]
        sim_yaw = states[jyaw].pos if jyaw in states else 0.0
        sim_pitch = states[jpitch].pos if jpitch in states else 0.0
        yaw_sign = self._sign("yaw_sign", 1.0)
        pitch_sign = self._sign("pitch_sign", 1.0)
        offset = self._sign("pitch_offset", 0.0)
        return (sim_yaw / yaw_sign, (sim_pitch - offset) / pitch_sign)
