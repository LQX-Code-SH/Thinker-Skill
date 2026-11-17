"""fetch_object 技能单测：步骤编排 / 选臂 / 位姿 / 失败透传 / 视觉确认。

不依赖 ROS 与真机：低头与定位用 ``call_skill`` 假实现，动作用 ``FakeBody``。
"""
import math
import os
from types import SimpleNamespace

import pytest

from fetch_object.geometry import plan_grasp, pose_delta, quat_angle
from fetch_object.objects import normalize_object, resolve_params
from fetch_object.plugin import DEFAULTS, FetchObject

from fakes import FakeBody, make_ctx


# ---------- 测试替身 ----------
class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _ok(data=None):
    return SimpleNamespace(state="succeed", code="OK", retryable=False,
                           data=dict(data or {}))


def _fail(code, retryable=False, data=None):
    return SimpleNamespace(state="failed", code=code, retryable=retryable,
                           data=dict(data or {}))


class _Caller:
    """按序消费假响应；队列只剩一条时重复使用。"""

    def __init__(self, responses=None):
        self.calls = []
        self._responses = {k: list(v) for k, v in (responses or {}).items()}

    def __call__(self, name, params):
        self.calls.append((name, dict(params)))
        queue = self._responses.get(name) or [_ok()]
        return queue.pop(0) if len(queue) > 1 else queue[0]


def _ctx(caller, *, cfg=None, params=None, body=None, clock=None):
    return make_ctx(call_skill=caller, cfg=cfg or {},
                    params=params or {}, body=body or FakeBody(),
                    clock=clock or _Clock())


def _steps(skill, ctx, params=None):
    return {s.name: s for s in skill.build_steps(ctx, params or {})}


def _detected(pose):
    return _ok({"object_name": "bottle", "obj_pose": list(pose)})


# ---------- 编排 ----------
def test_step_sequence():
    caller = _Caller()
    skill = FetchObject()
    steps = skill.build_steps(_ctx(caller), {"object_name": "bottle"})
    assert [s.name for s in steps] == [
        "低头预备", "感知定位", "规划抓取", "接近", "抓取位姿",
        "合手", "提拉", "视觉确认", "回收",
    ]
    assert sum(s.weight for s in steps) > 0


def test_missing_call_skill_returns_config_error():
    ctx = make_ctx()                      # 没有 call_skill
    skill = FetchObject()
    steps = _steps(skill, ctx)
    result = steps["低头预备"].run(ctx)
    assert result.status == "failed"
    assert result.code == "SYS_CONFIG_ERROR"


# ---------- 感知定位 ----------
def test_locate_failure_propagates_code_and_retryable():
    caller = _Caller({"detect_object": [
        _fail("PERCEPTION_OBJECT_NOT_FOUND", retryable=True)]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx)
    result = steps["感知定位"].run(ctx)
    assert result.status == "failed"
    assert result.code == "PERCEPTION_OBJECT_NOT_FOUND"
    assert result.retryable is True


def test_locate_falls_back_to_state_last_detection():
    caller = _Caller({"detect_object": [_ok({})]})
    ctx = _ctx(caller)
    ctx.state.set("last_detection", {"obj_pose": [0.4, -0.1, 0.6]})
    steps = _steps(FetchObject(), ctx)
    result = steps["感知定位"].run(ctx)
    assert result.status == "done"
    assert result.data["obj_pose"] == [0.4, -0.1, 0.6]


# ---------- 规划 ----------
def test_plan_chooses_side_by_y_sign():
    for y, side in ((-0.1, "right"), (0.1, "left")):
        caller = _Caller({"detect_object": [_detected([0.4, y, 0.6])]})
        ctx = _ctx(caller)
        steps = _steps(FetchObject(), ctx)
        steps["感知定位"].run(ctx)
        result = steps["规划抓取"].run(ctx)
        assert result.status == "done", result.data
        assert result.data["arm"] == side


def test_plan_respects_explicit_arm():
    caller = _Caller({"detect_object": [_detected([0.4, 0.1, 0.6])]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx, {"arm": "right"})
    steps["感知定位"].run(ctx)
    assert steps["规划抓取"].run(ctx).data["arm"] == "right"


def test_plan_unreachable_returns_ik_error():
    body = FakeBody()
    for arm in body.arms.values():
        arm.reachable = False
    caller = _Caller({"detect_object": [_detected([0.4, -0.1, 0.6])]})
    ctx = _ctx(caller, body=body)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    result = steps["规划抓取"].run(ctx)
    assert result.status == "failed"
    assert result.code == "MOTION_IK_UNREACHABLE"


# ---------- 动作 ----------
def test_motion_steps_settle_and_use_tcp():
    caller = _Caller({"detect_object": [_detected([0.4, -0.12, 0.62])]})
    body = FakeBody()
    ctx = _ctx(caller, body=body, cfg={"settle_ticks": 1})
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)

    for name in ("接近", "抓取位姿", "提拉"):
        result = steps[name].run(ctx)
        assert result.status == "done", name

    right = body.arms["right"]
    assert len(right.pose_targets) == 3
    assert all(tcp for _, tcp in right.pose_targets)          # 全部按 TCP 下发
    for pose, _ in right.pose_targets:
        assert not pose.is_unit_quaternion()                  # 6-DoF 目标

    approach, grasp, lift = (p for p, _ in right.pose_targets)
    # 接近比抓取离目标更远（prepare_delta_z 0.30 > grasp_delta_z 0.135）
    d_approach = math.dist(approach.as_tuple()[:3], grasp.as_tuple()[:3])
    assert d_approach > 0.1
    # 提拉 = 抓取位姿沿 base +z 抬升、-x 后退，姿态不变
    assert lift.z > grasp.z
    assert lift.x < grasp.x
    assert quat_angle(lift.as_tuple()[3:7], grasp.as_tuple()[3:7]) < 1e-6


def test_close_hand_uses_configured_positions():
    caller = _Caller({"detect_object": [_detected([0.4, -0.12, 0.62])]})
    body = FakeBody()
    clock = _Clock()
    positions = [0.5, 0.5, 0.5, 0.5, 0.4, 0.4]
    ctx = _ctx(caller, body=body, clock=clock,
               cfg={"grasp_settle_s": 0.8, "grasp_positions": positions})
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)

    assert steps["合手"].run(ctx).status == "running"      # 等待期内
    assert body.grippers["right"].positions == positions
    clock.advance(0.9)
    assert steps["合手"].run(ctx).status == "done"


# ---------- 视觉确认 ----------
def test_verify_success_when_object_lifted():
    caller = _Caller({"detect_object": [
        _detected([0.40, -0.12, 0.62]),
        _detected([0.38, -0.12, 0.80]),
    ]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    result = steps["视觉确认"].run(ctx)
    assert result.status == "done"
    assert result.data["lifted_z"] == pytest.approx(0.18, abs=1e-6)
    assert result.data["category"] == "bottle"


def test_verify_fails_when_object_stays():
    caller = _Caller({"detect_object": [
        _detected([0.40, -0.12, 0.62]),
        _detected([0.40, -0.12, 0.63]),
    ]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    result = steps["视觉确认"].run(ctx)
    assert result.status == "failed"
    assert result.code == "MOTION_GRASP_FAILED"
    assert result.retryable is True


def test_verify_fails_when_object_lost():
    caller = _Caller({"detect_object": [
        _detected([0.40, -0.12, 0.62]),
        _fail("PERCEPTION_OBJECT_NOT_FOUND", retryable=True),
    ]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    result = steps["视觉确认"].run(ctx)
    assert result.status == "failed"
    assert result.code == "MOTION_GRASP_FAILED"
    assert "未再检测" in result.data["reason"]


def test_verify_fails_when_object_moves_sideways():
    caller = _Caller({"detect_object": [
        _detected([0.40, -0.12, 0.62]),
        _detected([0.70, -0.12, 0.80]),      # 抬升够但水平位移过大
    ]})
    ctx = _ctx(caller)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    result = steps["视觉确认"].run(ctx)
    assert result.status == "failed"
    assert result.data["xy_shift"] > 0.12


# ---------- 回收 ----------
def test_hold_is_noop_without_config():
    caller = _Caller({"detect_object": [_detected([0.4, -0.12, 0.62])]})
    body = FakeBody()
    ctx = _ctx(caller, body=body)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    assert steps["回收"].run(ctx).status == "done"
    assert body.arms["right"].get_joints() == {}


def test_hold_uses_side_joints():
    caller = _Caller({"detect_object": [_detected([0.4, -0.12, 0.62])]})
    body = FakeBody()
    cfg = {"hold_joints": {"right": {"right_shoulder_pitch": 0.1}}}
    ctx = _ctx(caller, body=body, cfg=cfg)
    steps = _steps(FetchObject(), ctx)
    steps["感知定位"].run(ctx)
    steps["规划抓取"].run(ctx)
    assert steps["回收"].run(ctx).status == "done"
    assert body.arms["right"].get_joints()["right_shoulder_pitch"] == 0.1


# ---------- 后置条件 ----------
def test_postcondition_carries_category():
    skill = FetchObject()
    skill.build_steps(_ctx(_Caller()), {"object_name": "苹果"})
    assert skill.postcondition["holding_object"] == "apple"


# ---------- 类别归一化 ----------
@pytest.mark.parametrize("name,expected", [
    ("bottle", "bottle"), ("水瓶", "bottle"), ("罐", "bottle"),
    ("apple", "apple"), ("苹果", "apple"), ("水果", "apple"),
    ("banana", "banana"), ("香蕉", "banana"),
    ("plush toy", "generic"), ("", "generic"), (None, "generic"),
])
def test_normalize_object(name, expected):
    assert normalize_object(name) == expected


def test_resolve_params_layers():
    cfg = {"grasp_delta_z": 0.2,
           "objects": {"apple": {"grasp_delta_z": 0.15, "z_bias": 0.05}}}
    p = resolve_params(DEFAULTS, cfg, "apple", {"overrides": {"z_bias": 0.07}})
    assert p["grasp_delta_z"] == 0.15        # 类别覆写
    assert p["z_bias"] == 0.07               # 调用覆写
    assert p["approach_tol"] == DEFAULTS["approach_tol"]   # 默认值
    assert p["category"] == "apple"

    base = resolve_params(DEFAULTS, cfg, "bottle", None)
    assert base["grasp_delta_z"] == 0.2      # 类别未登记 → 用通用配置
    assert base["z_bias"] == DEFAULTS["z_bias"]


# ---------- 几何 ----------
def test_plan_grasp_offsets_in_tool_frame():
    p = dict(DEFAULTS)
    p["prepare_delta_y"] = 0.095
    p["grasp_delta_y"] = 0.07
    plan = plan_grasp([0.42, -0.14, 0.62], "right", p)
    goal, approach, grasp = plan["goal"], plan["approach"], plan["grasp"]

    # 接近与抓取的偏置差：Δz 0.165、Δy 0.025（工具系，模长可直接算）
    expect = math.hypot(0.30 - 0.135, 0.095 - 0.07)
    assert math.dist(approach, grasp) == pytest.approx(expect, abs=1e-9)
    # 抓取点比接近点更靠近目标
    assert math.dist(grasp, goal) < math.dist(approach, goal)
    # 姿态为单位四元数
    norm = math.sqrt(sum(v * v for v in plan["quat"]))
    assert norm == pytest.approx(1.0, abs=1e-9)


def test_extra_yaw_mirrors_by_side():
    p = dict(DEFAULTS)
    right = plan_grasp([0.42, -0.14, 0.62], "right", p)
    left = plan_grasp([0.42, 0.14, 0.62], "left", p)
    assert right["yaw"] == pytest.approx(-left["yaw"], abs=1e-9)


def test_pose_delta_reports_distance_and_angle():
    a = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    b = (0.03, 0.04, 0.0, 0.0, 0.0, 0.0, 1.0)
    dpos, dang = pose_delta(a, b)
    assert dpos == pytest.approx(0.05)
    assert dang == pytest.approx(0.0, abs=1e-9)


# ---------- 配置与整链 ----------
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_repo_config_section_resolves():
    from thinker_common.config import ConfigLoader

    config = ConfigLoader(os.path.join(ROOT, "configs")).load()
    section = config["skills"]["fetch_object"]
    assert section["objects"]["banana"]["grasp_delta_y"] == 0.05
    assert config["ik"]["rot_tol"] > 0

    params = resolve_params(DEFAULTS, section, "apple", None)
    assert params["grasp_positions"][0] == pytest.approx(0.95)   # 类别覆写
    assert params["grasp_delta_z"] == pytest.approx(0.135)       # 通用配置

    bottle = resolve_params(DEFAULTS, section, "bottle", None)
    assert bottle["z_bias"] == pytest.approx(0.02)


class _AutoClock:
    """每次读时间都前进一个 tick，用于等待类步骤。"""

    def __init__(self, step=0.1):
        self.t = 0.0
        self.step = step

    def __call__(self):
        self.t += self.step
        return self.t


def test_full_pipeline_succeeds():
    """整链（真 pipeline，无 ROS）：低头→定位→规划→动作→合手→确认→回收。"""
    from thinker_skill_core.pipeline import run_pipeline

    caller = _Caller({"detect_object": [
        _detected([0.40, -0.12, 0.62]),
        _detected([0.39, -0.12, 0.79]),
    ]})
    body = FakeBody()
    skill = FetchObject()
    steps_holder = []
    ctx = make_ctx(call_skill=caller, body=body,
                   cfg={"settle_ticks": 1, "grasp_settle_s": 0.2},
                   clock=_AutoClock(), steps=steps_holder)
    steps_holder.extend(skill.build_steps(ctx, {"object_name": "bottle"}))

    result = run_pipeline(steps_holder, ctx, tick=0.001, timeout=10.0)
    assert result.state == "succeed", result.data
    assert result.data["category"] == "bottle"
    assert result.data["lifted_z"] == pytest.approx(0.17, abs=1e-6)
    assert body.grippers["right"].positions          # 手已合上
    assert [name for name, _ in caller.calls] == [
        "look_down_table", "detect_object", "detect_object"]
