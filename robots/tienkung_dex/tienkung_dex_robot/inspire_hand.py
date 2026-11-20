"""因时（inspire）RH56 灵巧手适配：13 通道，走 angle_set / angle_actual。

口径对齐参考实现 ``Thinker-IL-LAB/ubt_IL/tienkung_dex``：
- 线缆上是 13 个 **0.1° 角码**（``SetAngle.joint_values[13]``），每通道一个关节；
- 策略侧 ``grasp_positions`` 是 6 维归一化 ``0=张开 / 1=闭合``，由
  :func:`pico6_to_inspire13` 展开成 13 角码（与采集端 bridge 完全一致）；
- 通道行程/零位/方向见 :data:`INSPIRE_CHANNELS`。

SDK ``tienkung_dex`` 只实现 brainco 手，本模块在项目适配层补齐 inspire，
对外提供与 brainco 手相同的 gripper 接口，由 ``TienkungDexGripper`` 按
``hand_variant`` 分派。
"""
from __future__ import annotations

import math
from typing import Any, Mapping, Optional, Sequence

# 因时 RH56：13 通道，单位 0.1°。
# (仿真关节短名, 码下限, 码上限, 0 rad 对应码, 方向)
# 屈曲通道码值越大越伸直（方向 -1）；侧摆零位在 0（方向 +1）。
INSPIRE_CHANNELS = (
    ("pinky_mcp",   900, 1650, 1650, -1),   # 0  小拇指指根
    ("ring_mcp",    900, 1650, 1650, -1),   # 1  无名指指根
    ("middle_mcp",  900, 1650, 1650, -1),   # 2  中指指根
    ("middle_yaw", -150,  150,    0, +1),   # 3  中指侧摆
    ("index_mcp",   900, 1650, 1650, -1),   # 4  食指指根
    ("index_yaw",  -150,  150,    0, +1),   # 5  食指侧摆
    ("pinky_pip",  1150, 1900, 1900, -1),   # 6  小拇指指中
    ("ring_pip",   1150, 1900, 1900, -1),   # 7  无名指指中
    ("middle_pip", 1150, 1900, 1900, -1),   # 8  中指指中
    ("index_pip",  1150, 1900, 1900, -1),   # 9  食指指中
    ("thumb_yaw",   650, 1750, 1750, -1),   # 10 大拇指侧摆
    ("thumb_mcp",  1250, 1600, 1600, -1),   # 11 大拇指一关节
    ("thumb_pip",  1500, 2040, 2040, -1),   # 12 大拇指二关节（mimic）
)
N_CHANNELS = len(INSPIRE_CHANNELS)
CODE_DEG = 0.1

# URDF 短关节名 → inspire 通道号（用于 13 维关节角回放）。
# 与 INSPIRE_CHANNELS 一一对应；``*_dip_joint`` 在 URDF 里是 mimic，不单独下发。
INSPIRE_JOINT_TO_CHANNEL = {
    "pinky_mcp": 0,
    "ring_mcp": 1,
    "middle_mcp": 2,
    "middle_yaw": 3,
    "index_mcp": 4,
    "index_yaw": 5,
    "pinky_pip": 6,
    "ring_pip": 7,
    "middle_pip": 8,
    "index_pip": 9,
    "thumb_yaw": 10,
    "thumb_mcp": 11,
    "thumb_pip": 12,
}
INSPIRE_MIMIC_JOINTS = ("thumb_dip", "index_dip", "middle_dip",
                        "ring_dip", "pinky_dip")

# 6 维归一化输入顺序：thumb_pitch, thumb_roll, index, middle, ring, little
# 展开表：(输出通道, 输入通道 or None, 是否 ×1.2, 固定码 or None)
PICO6_TO_INSPIRE13 = (
    (0, 5, True, None),
    (1, 4, True, None),
    (2, 3, True, None),
    (3, None, False, 0),
    (4, 2, True, None),
    (5, None, False, 0),
    (6, 5, False, None),
    (7, 4, False, None),
    (8, 3, False, None),
    (9, 2, False, None),
    (10, 1, False, None),
    (11, 0, False, None),
    (12, 0, False, None),
)


def pico6_to_inspire13(values: Sequence[float]) -> list:
    """6 维归一化（0=张开, 1=闭合）→ 13 个 0.1° 角码。

    与采集端 ``pico_to_inspire`` / bridge ``pico6_to_inspire13`` 一致：
    取整用 ``int`` 截断；四指 MCP 先 ``v + v*0.2``（复现浮点边界）；
    侧摆通道 3/5 固定中立 0。
    """
    values = list(values)
    if len(values) != 6:
        raise ValueError(f"pico6 需要 6 维输入，收到 {len(values)}")
    clamped = [min(max(float(v), 0.0), 1.0) for v in values]

    def mapped(v, low, high):
        return int(low + (1 - min(max(v, 0.0), 1.0)) * (high - low))

    codes = []
    for ch, src, boost, fixed in PICO6_TO_INSPIRE13:
        _, low, high, _, _ = INSPIRE_CHANNELS[ch]
        if fixed is not None:
            codes.append(fixed)
            continue
        v = clamped[int(src)]
        codes.append(mapped(v + v * 0.2 if boost else v, low, high))
    return codes


# 偏航/外展通道（侧摆）：合手时保持基准手型的值，不随闭合度变化。
# 3 = middle_yaw、5 = index_yaw、10 = thumb_yaw（拇指侧摆）。
ABDUCTION_CHANNELS = (3, 5, 10)


def close_codes_keep_abduction(grip_values: Sequence[float],
                               base_codes: Sequence[int]) -> list:
    """闭合手型：屈曲到 ``grip_values``，偏航/外展轴保持 ``base_codes``。

    对齐参考 ``hand_closed_pose(variant, base_pose=ready_hand_pose)``：
    握拳只驱动屈曲轴，侧摆（尤其拇指侧摆通道 10）留在抓取前的值，
    避免合手瞬间把拇指甩回全外展。``base_codes`` 为抓取前示教手型
    的 13 角码（顺序 = INSPIRE_CHANNELS）。
    """
    codes = pico6_to_inspire13(grip_values)
    for ch in ABDUCTION_CHANNELS:
        if ch < len(base_codes):
            codes[ch] = int(base_codes[ch])
    return codes


def code_to_rad(channel: int, code: int) -> float:
    """因时通道角码 → 仿真关节角（rad）。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    clipped = min(max(int(code), low), high)
    return (clipped - zero) * direction * CODE_DEG * math.pi / 180.0


def rad_to_code(channel: int, rad: float) -> int:
    """仿真关节角（rad）→ 因时通道角码，超行程夹到上下限。"""
    _, low, high, zero, direction = INSPIRE_CHANNELS[int(channel)]
    code = zero + float(rad) * 180.0 / math.pi / (direction * CODE_DEG)
    return int(round(min(max(code, low), high)))


def joint_short_name(name: str) -> str:
    """``left_index_yaw_joint`` → ``index_yaw``（也接受 ``index_yaw``）。"""
    text = str(name).strip()
    for side in ("left_", "right_"):
        if text.startswith(side):
            text = text[len(side):]
            break
    if text.endswith("_joint"):
        text = text[:-len("_joint")]
    return text


def codes_from_joint_rads(joints: Mapping[str, float],
                          base_codes: Optional[Sequence[int]] = None) -> list:
    """URDF 关节角（rad）→ 13 路角码。

    ``joints`` 为 ``{关节名: rad}``，键可带 ``left_/right_`` 前缀与 ``_joint``
    后缀；未给出的通道沿用 ``base_codes``（读不到当前值时用零位/张开），
    因此不会因为只回放部分通道而把其它手指突然抽回。
    """
    codes = [rad_to_code(i, 0.0) for i in range(N_CHANNELS)]
    for i, value in enumerate(list(base_codes or [])[:N_CHANNELS]):
        _, low, high, _, _ = INSPIRE_CHANNELS[i]
        code = int(value)
        # 未初始化时 status 可能是全 0；越界值不采用，保留零位，避免把手抽紧
        if low <= code <= high:
            codes[i] = code
    for name, rad in (joints or {}).items():
        channel = INSPIRE_JOINT_TO_CHANNEL.get(joint_short_name(name))
        if channel is None:
            continue
        codes[channel] = rad_to_code(channel, float(rad))
    return codes


def joint_rads_from_codes(codes: Sequence[int], side: str = "") -> dict:
    """13 路角码 → ``{``<side>_<short>_joint``: rad}``，供上层做起始姿态衔接。"""
    prefix = f"{side}_" if side else ""
    out = {}
    for short, channel in INSPIRE_JOINT_TO_CHANNEL.items():
        if channel < len(codes):
            out[f"{prefix}{short}_joint"] = code_to_rad(channel, codes[channel])
    return out


class InspireHand:
    """inspire 手适配（ROS 话题），接口对齐 SDK DexterousHand。"""

    def __init__(self, node: Any, side: str, *, hand_id: int = 1, qos: int = 10):
        self._node = node
        self._side = side
        self._hand_id = int(hand_id)
        self._qos = int(qos)
        self._pub = None
        self._force_pub = None
        self._timer = None
        self._target_codes: list = []    # 锁存的 13 角码（需持续重发，见下）
        self._values: list = []
        self._names: list = []
        self._force_threshold: list = []  # 13 通道力控阈值（force_actual，单位 g，预留）
        self._touch: dict = {}            # 5 指法向/切向力、接近觉、掌心

    def start(self) -> None:
        from inspire_hand_msgs.msg import (
            GetAngleAct, GetForceAct, SetAngle, SetForce, TouchData)

        self._pub = self._node.create_publisher(
            SetAngle, f"/{self._side}_hand/angle_set", self._qos)
        self._force_pub = self._node.create_publisher(
            SetForce, f"/{self._side}_hand/force_set", self._qos)
        self._node.create_subscription(
            GetAngleAct, f"/{self._side}_hand/angle_actual",
            self._on_actual, self._qos)
        self._node.create_subscription(
            GetForceAct, f"/{self._side}_hand/force_actual",
            self._on_force, self._qos)
        self._node.create_subscription(
            TouchData, f"/{self._side}_hand/touch_data",
            self._on_touch, self._qos)
        # 真机 angle_set 是"持续目标"语义：单次发布会被其他常驻节点的保持
        # 指令覆盖；20Hz 重发锁存目标，确保手型到位并保持。
        try:
            self._timer = self._node.create_timer(0.05, self._publish_target)
        except Exception:  # noqa: BLE001 - fake/无 ROS 环境退化为单次发布
            self._timer = None

    def stop(self) -> None:
        if self._timer is not None:
            try:
                self._timer.cancel()
            except Exception:  # noqa: BLE001
                pass
            self._timer = None
        self._pub = None
        self._force_pub = None

    def set_streaming(self, on: bool) -> None:
        """开关心跳重发：False = 不再发保持手型（技能结束后由 Host 调用）。"""
        self._streaming = bool(on)

    def _publish_target(self) -> None:
        """定时器回调：持续重发锁存的手型角码。"""
        if not getattr(self, "_streaming", True):
            return
        if self._pub is None or not self._target_codes:
            return
        from inspire_hand_msgs.msg import SetAngle
        msg = SetAngle()
        msg.hand_id = self._hand_id
        for i, v in enumerate(self._target_codes):
            msg.joint_values[i] = int(v)
        self._pub.publish(msg)

    def _on_actual(self, msg) -> None:
        self._values = [int(v) for v in getattr(msg, "joint_values", ())]
        self._names = list(getattr(msg, "joint_names", ()) or ())

    def _on_force(self, msg) -> None:
        # 文档标注「预留，暂无实际用途」：力控阈值设定值（单位 g），
        # 不是实测接触力。真正的接触检测看 touch_data。
        self._force_threshold = [int(v) for v in getattr(
            msg, "joint_values", ())]

    def _on_touch(self, msg) -> None:
        self._touch = {
            "finger_forces": [int(v) for v in getattr(msg, "finger_forces", ())],
            "finger_tangentials": [int(v) for v in getattr(
                msg, "finger_tangentials", ())],
            "finger_angles": [int(v) for v in getattr(msg, "finger_angles", ())],
            "finger_proximity": [int(v) for v in getattr(
                msg, "finger_proximity", ())],
            "palm_data": [int(v) for v in getattr(msg, "plam_data", ())],
        }

    @property
    def is_active(self) -> bool:
        return bool(self._values)

    def set_positions(self, codes: Sequence[int]) -> None:
        """锁存 13 个原始角码并立即发一帧（后续由定时器持续重发）。

        顺序 = INSPIRE_CHANNELS / ``angle_actual.joint_names``。
        """
        if self._pub is None:
            raise RuntimeError(f"inspire {self._side} hand 未 start")
        values = [int(round(float(v))) for v in codes][:N_CHANNELS]
        values += [0] * (N_CHANNELS - len(values))
        self._target_codes = values
        self._publish_target()   # 立即一帧，20Hz 定时器随后持续重发

    def set_force(self, forces: Sequence[int]) -> None:
        """下发 13 通道力控阈值（0~1000，单位 g）到 ``force_set``。

        注意：这是**力控阈值**而非位置；文件大小单位 g，值越大越"用力"。
        """
        from inspire_hand_msgs.msg import SetForce

        if self._force_pub is None:
            raise RuntimeError(f"inspire {self._side} hand 未 start")
        msg = SetForce()
        msg.hand_id = self._hand_id
        values = [max(0, min(1000, int(round(float(v))))) for v in forces][:N_CHANNELS]
        values += [0] * (N_CHANNELS - len(values))
        for i, v in enumerate(values):
            msg.joint_values[i] = v
        self._force_pub.publish(msg)

    def get_force_threshold(self) -> list:
        """力控阈值设定值（force_actual，单位 g，文档标注预留/暂无实际用途）。

        不是实测接触力；接触判定请用 :meth:`get_touch` / ``touch_data``。
        """
        return list(self._force_threshold)

    def get_touch(self) -> dict:
        """最新触觉（来自 ``touch_data``）：

        - ``finger_forces`` / ``finger_tangentials``：指尖法向/切向力，单位 0.01N
        - ``finger_angles``：切向力方向（度，65535=无接触）
        - ``finger_proximity``：接近觉 0~16000000
        - ``palm_data``：掌心 9 区（左/中/右各 法向力/切向力/方向）
        """
        return {k: list(v) for k, v in self._touch.items()}

    def contact_state(self, *, force_threshold: float = 50.0,
                      proximity_threshold: int = 100000) -> dict:
        """基于 touch_data 的接触判定（单位见 get_touch）。

        ``force_threshold`` 单位 0.01N（默认 50 = 0.5N）；
        ``proximity_threshold`` 为接近觉原始值（默认 100000）。
        返回每指是否接触（法向力达标或接近觉达标）及汇总。
        """
        touch = self.get_touch()
        forces = touch.get("finger_forces", [])
        prox = touch.get("finger_proximity", [])
        touched = []
        for i in range(5):
            f = forces[i] if i < len(forces) else 0
            p = prox[i] if i < len(prox) else 0
            touched.append(bool(f >= force_threshold or p >= proximity_threshold))
        return {"touched": touched, "count": sum(touched),
                "any_contact": any(touched)}

    def get_status(self) -> Optional[Any]:
        """返回带 ``positions``（原始角码）的状态对象，对齐 brainco 接口。"""
        from types import SimpleNamespace
        if not self._values:
            return None
        return SimpleNamespace(positions=tuple(self._values),
                               joint_names=tuple(self._names))
