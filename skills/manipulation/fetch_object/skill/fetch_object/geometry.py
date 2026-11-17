"""抓取位姿几何（纯 Python，无 numpy / scipy 依赖，便于单测）。

对齐参考实现 ``kaiwu_agent/fetch_object`` 的 ``grasp_bottle``：

- 基准夹爪朝向 ``euler(0, -90°, 0)``（zyx，即 ``Rz·Ry·Rx``）；
- 目标横向方位角经 tanh 映射额外 yaw（上限 42°、scale 28°、gain 1.6），左右手取反号；
- 中心补偿 ``x - 0.01·cf``、``y ± 0.015·cf``（``cf = extra_yaw / max_extra_yaw``）；
- 工具系偏置 ``[dx, dy, dz]`` 按 **目标姿态** 旋转后叠加到目标位置。

四元数统一用 ``(x, y, z, w)``，与 ``thinker_skill_sdk.Pose`` 一致。
"""
from __future__ import annotations

import math
from typing import Any, Dict, Mapping, Sequence, Tuple

Quat = Tuple[float, float, float, float]
Vec3 = Tuple[float, float, float]


# ---------- 四元数基础 ----------
def quat_mul(a: Sequence[float], b: Sequence[float]) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_normalize(q: Sequence[float]) -> Quat:
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(q[0]) / n, float(q[1]) / n, float(q[2]) / n, float(q[3]) / n)


def quat_axis(axis: str, angle_rad: float) -> Quat:
    half = 0.5 * float(angle_rad)
    s = math.sin(half)
    return {
        "x": (s, 0.0, 0.0, math.cos(half)),
        "y": (0.0, s, 0.0, math.cos(half)),
        "z": (0.0, 0.0, s, math.cos(half)),
    }[axis]


def quat_from_zyx(rx_deg: float, ry_deg: float, rz_deg: float) -> Quat:
    """``Rz(rz)·Ry(ry)·Rx(rx)``，角度单位为度（对齐参考 ``euler_to_pose``）。"""
    q = quat_axis("z", math.radians(rz_deg))
    q = quat_mul(q, quat_axis("y", math.radians(ry_deg)))
    q = quat_mul(q, quat_axis("x", math.radians(rx_deg)))
    return quat_normalize(q)


def quat_rotate(quat: Sequence[float], vec: Sequence[float]) -> Vec3:
    """用四元数旋转向量（``v' = q v q*``）。"""
    x, y, z, w = quat_normalize(quat)
    vx, vy, vz = (float(v) for v in vec)
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quat_angle(a: Sequence[float], b: Sequence[float]) -> float:
    """两个姿态之间的夹角（弧度，0..pi）。"""
    qa = quat_normalize(a)
    qb = quat_normalize(b)
    dot = abs(sum(float(x) * float(y) for x, y in zip(qa, qb)))
    return 2.0 * math.acos(min(1.0, dot))


# ---------- 抓取位姿 ----------
def extra_yaw(position: Sequence[float], side: str,
              yaw_map: Mapping[str, Any]) -> Tuple[float, float]:
    """由目标横向方位角映射额外 yaw，返回 ``(yaw_rad, center_factor)``。

    对齐参考 ``_reshape_bottle_grasp_goal_pose``：以 ``ref_point`` 为原点，
    只看指向目标右前方的分量，用 tanh 饱和到 ``max_extra_yaw``。
    """
    use_right = side == "right"
    ref_x = float(yaw_map.get("ref_x", 0.10))
    ref_y = float(yaw_map.get("ref_y_abs", 0.20)) * (-1.0 if use_right else 1.0)
    max_extra = math.radians(float(yaw_map.get("max_extra_yaw_deg", 42.0)))
    angle_scale = math.radians(float(yaw_map.get("angle_scale_deg", 28.0)))
    gain = float(yaw_map.get("tanh_gain", 1.6))

    vx = float(position[0]) - ref_x
    vy = float(position[1]) - ref_y
    raw = math.atan2(vy, max(vx, 1e-4))
    inward = raw if use_right else -raw
    inward = max(0.0, inward)
    extra = max_extra * math.tanh(gain * inward / max(angle_scale, 1e-4))
    signed = extra * (1.0 if use_right else -1.0)
    factor = extra / max_extra if max_extra > 1e-6 else 0.0
    return signed, factor


def grasp_quat(extra_yaw_rad: float, base_rpy_deg: Sequence[float]) -> Quat:
    """基准夹爪姿态叠加一层绕 base z 的额外 yaw。"""
    base = quat_from_zyx(*base_rpy_deg)
    return quat_normalize(quat_mul(quat_axis("z", extra_yaw_rad), base))


def offset_in_tool_frame(position: Sequence[float], quat: Sequence[float],
                         delta: Sequence[float]) -> Vec3:
    """在目标姿态下叠加工具系偏置：``p' = p + R(q)·delta``。"""
    rx, ry, rz = quat_rotate(quat, delta)
    return (float(position[0]) + rx,
            float(position[1]) + ry,
            float(position[2]) + rz)


def delta_y(value: Any, side: str, default: float) -> float:
    """取单侧偏置：支持标量或 ``{left, right}``（参考实现左右手不同）。"""
    if value is None:
        return float(default)
    if isinstance(value, Mapping):
        if side in value:
            return float(value[side])
        return float(value.get("default", default))
    return float(value)


def plan_grasp(position: Sequence[float], side: str,
               p: Mapping[str, Any]) -> Dict[str, Any]:
    """算出接近/抓取位姿与姿态。

    ``p`` 为已合并的抓取参数（默认值 + 类别覆写 + 调用覆写），键：

    ``z_bias`` / ``yaw_map`` / ``center_comp`` / ``base_orientation_deg`` /
    ``tcp_delta_x`` / ``prepare_delta_y`` / ``prepare_delta_z`` /
    ``grasp_delta_y`` / ``grasp_delta_z``

    ``*_delta_y`` 允许写成 ``{left: ..., right: ...}``，按 ``side`` 取值。
    """
    use_right = side == "right"
    sign = -1.0 if use_right else 1.0
    comp = p.get("center_comp") or {}

    goal = [float(position[0]), float(position[1]),
            float(position[2]) + float(p.get("z_bias", 0.0))]
    yaw, factor = extra_yaw(goal, side, p.get("yaw_map") or {})
    goal[0] -= float(comp.get("x_scale", 0.01)) * factor
    goal[1] += float(comp.get("y_scale", 0.015)) * sign * factor

    quat = grasp_quat(yaw, p.get("base_orientation_deg") or (0.0, -90.0, 0.0))
    dx = float(p.get("tcp_delta_x", -0.03))

    approach = offset_in_tool_frame(
        goal, quat,
        (dx, sign * delta_y(p.get("prepare_delta_y"), side, 0.11),
         float(p.get("prepare_delta_z", 0.30))))
    grasp = offset_in_tool_frame(
        goal, quat,
        (dx, sign * delta_y(p.get("grasp_delta_y"), side, 0.07),
         float(p.get("grasp_delta_z", 0.135))))
    return {
        "goal": goal,
        "quat": quat,
        "yaw": yaw,
        "center_factor": factor,
        "approach": approach,
        "grasp": grasp,
    }


def lift_position(position: Sequence[float], dx: float, dz: float) -> Vec3:
    """提拉：base 系直接加偏移（姿态不变），对齐参考 ``pick_x/z_in_delta``。"""
    return (float(position[0]) + float(dx),
            float(position[1]),
            float(position[2]) + float(dz))


# ---------- 位姿误差（到位判据用） ----------
def pose_delta(current: Sequence[float], target: Sequence[float]
               ) -> Tuple[float, float]:
    """``(位置距离, 姿态夹角)``；``current`` / ``target`` 为 7 元组。"""
    dpos = math.sqrt(sum((float(a) - float(b)) ** 2
                         for a, b in zip(current[:3], target[:3])))
    dang = quat_angle(current[3:7], target[3:7])
    return dpos, dang
