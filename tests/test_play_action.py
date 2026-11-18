"""play_action 技能测试（FakeBody + 假时钟，无 ROS / 无真机）。"""
import json

import pytest

from thinker_skill_core import run_pipeline
from thinker_skill_sdk import ErrorCode

from fakes import FakeBody, make_ctx

LIMITS = {
    "shoulder_pitch_l_joint": {"lower": -2.8, "upper": 2.8},
    "elbow_pitch_l_joint": {"lower": -2.5, "upper": 0.2},
    "shoulder_pitch_r_joint": {"lower": -2.8, "upper": 2.8},
    "waist_pitch_joint": {"lower": -0.5, "upper": 0.9},
    "waist_roll_joint": {"lower": -0.4, "upper": 0.4},
    "waist_yaw_joint": {"lower": -2.6, "upper": 3.2},
    "head_yaw_joint": {"lower": -1.0, "upper": 1.0},
    "head_pitch_joint": {"lower": -0.26, "upper": 0.87},
    "hip_pitch_l_joint": {"lower": -3.0, "upper": 2.8},
    "knee_pitch_l_joint": {"lower": -0.1, "upper": 2.5},
    "left_thumb_mcp_joint": {"lower": 0.0, "upper": 0.611},
    "left_thumb_yaw_joint": {"lower": 0.0, "upper": 1.92},
    "left_index_mcp_joint": {"lower": 0.0, "upper": 1.309},
    "left_middle_mcp_joint": {"lower": 0.0, "upper": 1.309},
    "left_ring_mcp_joint": {"lower": 0.0, "upper": 1.309},
    "left_pinky_mcp_joint": {"lower": 0.0, "upper": 1.309},
}


class Clock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def _kf(t_ms, value, mode="auto"):
    return {"timeMs": t_ms, "value": value, "interpolation": mode,
            "inHandle": {"dxMs": -100, "dy": 0.0},
            "outHandle": {"dxMs": 100, "dy": 0.0}}


def _track(joint, keyframes, mode="auto"):
    return {"type": "joint", "jointName": joint, "name": joint,
            "keyframes": [_kf(t, v, mode) for t, v in keyframes]}


def _write(tmp_path, tracks, *, duration=1000, name="goready", play=None,
           robot_name="tienkung_dex_inspire.urdf", filename="action.json"):
    doc = {
        "schemaVersion": 2,
        "robot": {"name": robot_name,
                  "jointNames": sorted(LIMITS), "joints": LIMITS},
        "activeClipId": "c1",
        "clips": [{
            "id": "c1", "name": name, "durationMs": duration, "fps": 30, "loop": True,
            "playRange": play or {"startMs": 0, "endMs": duration},
            "tracks": tracks,
        }],
    }
    path = tmp_path / filename
    path.write_text(json.dumps(doc), encoding="utf-8")
    return str(path)


def _drive(steps, ctx, clock, dt=0.05, limit=500):
    """按 tick 驱动步骤直到全部完成（时钟由测试显式推进）。"""
    runs = []
    for step in steps:
        for _ in range(limit):
            result = step.run(ctx)
            runs.append((step.name, result))
            if result.status != "running":
                assert result.status == "done", (step.name, result.code, result.data)
                break
            clock.t += dt
        else:  # pragma: no cover
            raise AssertionError(f"步骤 {step.name} 未在 {limit} tick 内结束")
    return runs


def _skill():
    from play_action.plugin import PlayAction
    return PlayAction()


def test_blend_starts_from_current_pose(tmp_path):
    path = _write(tmp_path, [_track("shoulder_pitch_l_joint",
                                    [(0, 0.2), (1000, 0.2)])])
    body = FakeBody()
    body.arms["left"]._joints = {"left_shoulder_pitch": 0.8}
    body.arms["right"]._joints = {"right_shoulder_pitch": 0.4}
    clock = Clock()
    ctx = make_ctx(body=body, clock=clock)

    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.4})
    steps[0].run(ctx)                       # 读取动作
    assert steps[1].run(ctx).status == "running"
    # alpha=0：首帧不跳变，仍是当前姿态
    assert body.arms["left"].get_joints()["left_shoulder_pitch"] == pytest.approx(0.8)

    clock.t = 0.2                           # smoothstep(0.5) = 0.5
    assert steps[1].run(ctx).status == "running"
    assert body.arms["left"].get_joints()["left_shoulder_pitch"] == pytest.approx(0.5, abs=1e-6)

    clock.t = 0.4
    assert steps[1].run(ctx).status == "done"
    assert body.arms["left"].get_joints()["left_shoulder_pitch"] == pytest.approx(0.2)
    # 右臂文件里没有轨道 → 不下发
    assert body.arms["right"].get_joints() == {"right_shoulder_pitch": 0.4}


def test_arms_are_batched_into_one_command(tmp_path):
    """左右臂目标必须在同一次下发里合并（否则真机后一条会被丢弃）。"""
    path = _write(tmp_path, [
        _track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)]),
        _track("shoulder_pitch_r_joint", [(0, 0.0), (100, 0.2)]),
    ], duration=100)
    body = FakeBody()
    clock = Clock()
    ctx = make_ctx(body=body, clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.0})
    _drive(steps, ctx, clock)

    assert body.arm_batch_calls
    assert set(body.arm_batch_calls[-1]) == {"left", "right"}


def test_plays_to_clip_end_and_reports(tmp_path):
    path = _write(tmp_path, [
        _track("shoulder_pitch_l_joint", [(0, 0.0), (1000, 1.0)]),
        _track("waist_yaw_joint", [(0, 0.0), (1000, 0.5)]),
        _track("head_pitch_joint", [(0, 0.0), (1000, 0.6)]),
        _track("hip_pitch_l_joint", [(0, -0.2), (1000, 0.3)]),
    ], duration=1000)
    clock = Clock()
    ctx = make_ctx(clock=clock)

    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.2, "speed": 1.0})
    prepare = steps[0].run(ctx)
    assert prepare.status == "done"
    assert prepare.data["clip"] == "goready"
    assert "hip_pitch_l_joint" in prepare.data["ignored_joints"]
    assert prepare.data["leg_motion_rad"] == pytest.approx(0.5, abs=1e-6)
    assert prepare.data["duration_s"] == pytest.approx(1.0)

    runs = _drive(steps[1:], ctx, clock)
    final = runs[-1][1]
    assert final.data["frames"] > 0
    assert final.data["played_s"] == pytest.approx(1.0, abs=0.06)

    # 末帧 = 动作文件最后一帧（腿不参与）
    assert ctx.body.arms["left"].get_joints()["left_shoulder_pitch"] == pytest.approx(1.0, abs=1e-3)
    assert ctx.body.waist.get_pose()[2] == pytest.approx(0.5, abs=1e-3)
    assert ctx.body.head.get_pose()[1] == pytest.approx(0.6, abs=1e-3)


def test_pipeline_runs_to_success(tmp_path):
    path = _write(tmp_path, [_track("shoulder_pitch_r_joint",
                                    [(0, 0.0), (200, 0.3)])], duration=200)
    clock = Clock()
    ctx = make_ctx(clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.1})

    def advance():
        clock.t += 0.05
        return clock.t

    ctx.now = advance
    result = run_pipeline(steps, ctx, tick=0)
    assert result.ok
    assert result.data["clip"] == "goready"
    assert ctx.body.arms["right"].get_joints()["right_shoulder_pitch"] == pytest.approx(0.3, abs=1e-3)


def test_hands_are_played_normalized(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 0.0), (500, 1.3089969)]),
        _track("left_thumb_mcp_joint", [(0, 0.0), (500, 0.6108652)]),
    ], duration=500)
    clock = Clock()
    ctx = make_ctx(clock=clock)

    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.0,
                                       "play_hands": True})
    assert steps[0].run(ctx).data["play_hands"] is True
    _drive(steps[1:], ctx, clock)

    calls = ctx.body.grippers["left"].set_calls
    assert calls, "手部应收到 set_positions"
    last = ctx.body.grippers["left"].positions
    assert len(last) == 6
    assert last[2] == pytest.approx(1.0, abs=0.01)   # index 满行程
    assert last[0] == pytest.approx(1.0, abs=0.01)   # thumb_pitch 满行程
    assert last[5] == pytest.approx(0.0)             # 无轨道通道按张开


def test_play_hands_false_skips_gripper(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 0.0), (200, 1.3)]),
    ], duration=200)
    clock = Clock()
    ctx = make_ctx(clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "play_hands": False,
                                       "transition_s": 0.0})
    assert steps[0].run(ctx).data["play_hands"] is False
    _drive(steps[1:], ctx, clock)
    assert ctx.body.grippers["left"].set_calls == []


def test_missing_action_param():
    steps = _skill().build_steps(make_ctx(), {})
    result = steps[0].run(make_ctx())
    assert result.status == "failed"
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS


def test_missing_file(tmp_path):
    ctx = make_ctx()
    steps = _skill().build_steps(ctx, {"action": str(tmp_path / "nope.json")})
    result = steps[0].run(ctx)
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS


def test_bad_schema(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"schemaVersion": 1, "clips": []}), encoding="utf-8")
    ctx = make_ctx()
    steps = _skill().build_steps(ctx, {"action": str(path)})
    result = steps[0].run(ctx)
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS


def test_only_leg_tracks_rejected(tmp_path):
    path = _write(tmp_path, [_track("hip_pitch_l_joint", [(0, 0.0), (100, 0.2)])],
                  duration=100)
    ctx = make_ctx()
    steps = _skill().build_steps(ctx, {"action": path})
    result = steps[0].run(ctx)
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS


def _shipped_action():
    from pathlib import Path
    return str(Path(__file__).resolve().parents[1]
               / "skills" / "motion" / "play_action" / "actions" / "goready.json")


def test_shipped_goready_parses():
    from play_action.action_file import load_action, select_clip

    clip = select_clip(load_action(_shipped_action()))
    assert clip.name == "goready"
    assert clip.span_ms == pytest.approx(10000.0)
    sample = clip.sample(clip.start_ms)
    assert "shoulder_roll_l_joint" in sample and "hip_pitch_l_joint" in sample
    assert clip.sample(clip.end_ms)["head_pitch_joint"] == pytest.approx(0.786, abs=1e-3)


def test_hermite_uses_handles():
    from play_action.action_file import Track

    track = Track("j", [
        {"timeMs": 0, "value": 0.0, "interpolation": "auto",
         "outHandle": {"dxMs": 1000, "dy": 0.5}, "inHandle": {"dxMs": 0, "dy": 0}},
        {"timeMs": 1000, "value": 1.0, "interpolation": "auto",
         "inHandle": {"dxMs": -1000, "dy": -0.25}, "outHandle": {"dxMs": 0, "dy": 0}},
    ])
    # 切线 = dy/dxMs（inHandle 的 dx 为负，dy 同号）；
    # 两端切线 5e-4 / 2.5e-4 > 割线 1e-3 的一半 → 中点 0.53125，而不是线性 0.5
    assert track.value_at(500.0) == pytest.approx(0.53125, abs=1e-9)
    assert track.value_at(0.0) == pytest.approx(0.0)
    assert track.value_at(1000.0) == pytest.approx(1.0)


def test_linear_and_step_modes():
    from play_action.action_file import Track

    linear = Track("j", [{"timeMs": 0, "value": 0.0, "interpolation": "linear"},
                         {"timeMs": 1000, "value": 1.0, "interpolation": "linear"}])
    step = Track("j", [{"timeMs": 0, "value": 0.0, "interpolation": "step"},
                       {"timeMs": 1000, "value": 1.0, "interpolation": "step"}])
    assert linear.value_at(250.0) == pytest.approx(0.25)
    assert step.value_at(999.0) == pytest.approx(0.0)


def test_resolve_from_skill_dir(tmp_path):
    from play_action.action_file import resolve_action_path

    skill_dir = tmp_path / "skill"
    (skill_dir / "actions").mkdir(parents=True)
    (skill_dir / "actions" / "wave.json").write_text("{}", encoding="utf-8")
    assert resolve_action_path("wave", skill_dir=str(skill_dir)) == \
        str(skill_dir / "actions" / "wave.json")
    assert resolve_action_path("actions/wave.json", skill_dir=str(skill_dir)) == \
        str(skill_dir / "actions" / "wave.json")


def test_clip_selector(tmp_path):
    path = _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
                  duration=100)
    ctx = make_ctx()
    assert _skill().build_steps(ctx, {"action": path, "clip": "goready"})[0].run(ctx).status == "done"
    result = _skill().build_steps(ctx, {"action": path, "clip": "nope"})[0].run(ctx)
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS


# ---------- 手型口径：13 维（因时）/ 6 维（强脑）----------
def _inspire_body() -> FakeBody:
    """按因时手装配：手型声明 inspire13 且提供关节角读写接口。"""
    body = FakeBody(hand_variant="inspire")
    for gripper in body.grippers.values():
        gripper.hand_format = "inspire13"
        gripper.dof = 13
    return body


def test_hand_mode_auto_uses_13d_for_inspire(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 0.2), (500, 1.0)]),
        _track("left_index_yaw_joint", [(0, -0.1), (500, 0.2)]),
        _track("left_thumb_pip_joint", [(0, 0.1), (500, 0.4)]),
    ], duration=500)
    body = _inspire_body()
    body.grippers["left"].joint_positions = {
        "left_index_mcp_joint": 0.5,
        "left_index_yaw_joint": 0.0,
        "left_thumb_pip_joint": 0.0,
    }
    clock = Clock()
    ctx = make_ctx(body=body, clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.0})
    prepare = steps[0].run(ctx)

    assert prepare.data["hand_mode"] == "13d"
    assert prepare.data["hand_plan"] == {"left": "13d", "right": "none"}
    # 13 维通道（侧摆/指中）不再被当作"忽略关节"
    assert "left_index_yaw_joint" not in prepare.data["ignored_joints"]

    _drive(steps[1:], ctx, clock)
    assert body.grippers["left"].set_calls == []          # 不走上采样
    last = body.grippers["left"].joint_positions
    assert last["left_index_mcp_joint"] == pytest.approx(1.0)
    assert last["left_index_yaw_joint"] == pytest.approx(0.2)
    assert last["left_thumb_pip_joint"] == pytest.approx(0.4)


def test_hand_mode_13d_crossfades_from_current(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 1.0), (1000, 1.0)]),
    ], duration=1000)
    body = _inspire_body()
    body.grippers["left"].joint_positions = {"left_index_mcp_joint": 0.4}
    clock = Clock()
    ctx = make_ctx(body=body, clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "transition_s": 0.4})
    assert steps[0].run(ctx).data["hand_mode"] == "13d"

    assert steps[1].run(ctx).status == "running"
    # alpha=0：手指从当前角码出发，不再直接跳到首帧
    assert body.grippers["left"].joint_positions["left_index_mcp_joint"] == \
        pytest.approx(0.4)

    clock.t = 0.2                                       # smoothstep(0.5)=0.5
    steps[1].run(ctx)
    assert body.grippers["left"].joint_positions["left_index_mcp_joint"] == \
        pytest.approx(0.7, abs=1e-6)


def test_hand_mode_13d_rejected_for_brainco(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 0.0), (200, 1.3)]),
    ], duration=200)
    ctx = make_ctx()                                     # FakeGripper 默认非因时手
    steps = _skill().build_steps(ctx, {"action": path, "hand_mode": "13d"})
    result = steps[0].run(ctx)
    assert result.status == "failed"
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert "13 维" in result.data["error"]


def test_hand_mode_6d_ignores_13d_tracks(tmp_path):
    path = _write(tmp_path, [
        _track("left_index_mcp_joint", [(0, 0.0), (500, 1.3089969)]),
        _track("left_index_yaw_joint", [(0, 0.0), (500, 0.2)]),
    ], duration=500)
    body = _inspire_body()
    clock = Clock()
    ctx = make_ctx(body=body, clock=clock)
    steps = _skill().build_steps(ctx, {"action": path, "hand_mode": "6d",
                                       "transition_s": 0.0})
    prepare = steps[0].run(ctx)
    assert prepare.data["hand_mode"] == "6d"
    assert "left_index_yaw_joint" in prepare.data["ignored_joints"]

    _drive(steps[1:], ctx, clock)
    assert body.grippers["left"].set_calls            # 走 6 维归一化
    assert body.grippers["left"].joint_calls == []


def test_hand_mode_invalid_value_rejected(tmp_path):
    path = _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
                  duration=100)
    ctx = make_ctx()
    result = _skill().build_steps(ctx, {"action": path, "hand_mode": "15d"})[0].run(ctx)
    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert "hand_mode" in result.data["error"]


def test_action_file_model_warning(tmp_path):
    path = _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
                  duration=100, robot_name="other_robot.urdf")
    ctx = make_ctx()
    prepare = _skill().build_steps(ctx, {"action": path})[0].run(ctx)
    assert prepare.data["robot_name"] == "other_robot.urdf"
    assert prepare.data["warnings"]
    assert "other_robot.urdf" in prepare.data["warnings"][0]


# ---------- 动作清单（actions.yaml）：别名、默认值、机型/末端绑定 ----------
def _write_registry(tmp_path, actions, *, version=1, filename="actions.yaml"):
    import yaml
    path = tmp_path / filename
    path.write_text(yaml.safe_dump({"version": version, "actions": actions},
                                   allow_unicode=True), encoding="utf-8")
    return str(path)


def _registry_ctx(tmp_path, body=None):
    return make_ctx(body=body or FakeBody(),
                    cfg={"actions_dir": str(tmp_path)})


def test_registry_alias_and_defaults(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (1000, 0.2)])],
           duration=1000, filename="wave.json")
    _write_registry(tmp_path, {
        "wave_fast": {
            "file": "wave.json", "clip": "goready",
            "display_name": "挥手", "aliases": ["挥手动作"],
            "models": ["tienkung_dex"], "hand_variants": ["brainco"],
            "hand_mode": "6d",
            "defaults": {"speed": 2.0, "transition_s": 0.0},
        }
    })
    body = FakeBody(hand_variant="brainco")
    ctx = _registry_ctx(tmp_path, body)
    prepare = _skill().build_steps(ctx, {"action": "挥手动作"})[0].run(ctx)

    assert prepare.status == "done"
    assert prepare.data["registry_key"] == "wave_fast"
    assert prepare.data["display_name"] == "挥手"
    assert prepare.data["registry"] == str(tmp_path / "actions.yaml")
    assert prepare.data["speed"] == pytest.approx(2.0)
    assert prepare.data["transition_s"] == pytest.approx(0.0)
    assert prepare.data["duration_s"] == pytest.approx(0.5)   # 1s / 2x
    assert prepare.data["compat"]["status"] == "ok"


def test_registry_defaults_are_overridable(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (1000, 0.2)])],
           duration=1000, filename="wave.json")
    _write_registry(tmp_path, {"wave_fast": {
        "file": "wave.json", "defaults": {"speed": 2.0}}})
    ctx = _registry_ctx(tmp_path)
    prepare = _skill().build_steps(
        ctx, {"action": "wave_fast", "speed": 1.0})[0].run(ctx)
    assert prepare.data["speed"] == pytest.approx(1.0)


def test_registry_model_mismatch_strict(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
           duration=100, filename="a.json")
    _write_registry(tmp_path, {"a": {"file": "a.json",
                                     "models": ["other_model"]}})
    ctx = _registry_ctx(tmp_path)
    result = _skill().build_steps(ctx, {"action": "a"})[0].run(ctx)

    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert "机型不匹配" in result.data["error"]
    assert result.data["registry_key"] == "a"
    assert result.data["compat"]["model_ok"] is False


def test_registry_mismatch_warn_policy(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
           duration=100, filename="a.json")
    _write_registry(tmp_path, {"a": {
        "file": "a.json", "models": ["other_model"],
        "on_mismatch": "warn"}})
    ctx = _registry_ctx(tmp_path)
    prepare = _skill().build_steps(ctx, {"action": "a"})[0].run(ctx)

    assert prepare.status == "done"
    assert prepare.data["compat"]["status"] == "warn"
    assert any("机型不匹配" in w for w in prepare.data["warnings"])


def test_registry_hand_variant_mismatch(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
           duration=100, filename="a.json")
    _write_registry(tmp_path, {"a": {"file": "a.json",
                                     "hand_variants": ["inspire"]}})
    body = FakeBody(hand_variant="brainco")
    ctx = _registry_ctx(tmp_path, body)
    result = _skill().build_steps(ctx, {"action": "a"})[0].run(ctx)

    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert "末端类型不匹配" in result.data["error"]
    assert result.data["compat"]["hand_ok"] is False


def test_registry_missing_file_reports_key(tmp_path):
    _write_registry(tmp_path, {"a": {"file": "nope.json"}})
    ctx = _registry_ctx(tmp_path)
    result = _skill().build_steps(ctx, {"action": "a"})[0].run(ctx)

    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert result.data["registry_key"] == "a"
    assert "nope.json" in result.data["error"]


def test_registry_unregistered_name_warns(tmp_path):
    _write(tmp_path, [_track("shoulder_pitch_l_joint", [(0, 0.0), (100, 0.1)])],
           duration=100, filename="extra.json")
    _write_registry(tmp_path, {"a": {"file": "extra.json"}})
    ctx = _registry_ctx(tmp_path)
    prepare = _skill().build_steps(ctx, {"action": "extra"})[0].run(ctx)

    assert prepare.status == "done"
    assert prepare.data["registry_key"] is None
    assert any("未在动作清单中注册" in w for w in prepare.data["warnings"])


def test_registry_bad_yaml_rejected(tmp_path):
    (tmp_path / "actions.yaml").write_text("version: [", encoding="utf-8")
    ctx = _registry_ctx(tmp_path)
    result = _skill().build_steps(
        ctx, {"action": str(tmp_path / "a.json")})[0].run(ctx)

    assert result.code == ErrorCode.INPUT_INVALID_PARAMS
    assert "动作清单非法" in result.data["error"]


def test_registry_fixed_hand_skips_gripper(tmp_path):
    _write(tmp_path, [_track("left_index_mcp_joint", [(0, 0.0), (200, 1.3)])],
           duration=200, filename="a.json")
    _write_registry(tmp_path, {"a": {"file": "a.json",
                                     "hand_variants": ["fixed"]}})
    body = FakeBody(hand_variant="fixed")
    ctx = _registry_ctx(tmp_path, body)
    prepare = _skill().build_steps(ctx, {"action": "a"})[0].run(ctx)

    assert prepare.status == "done"
    assert prepare.data["hand_variant"] == "fixed"
    assert prepare.data["play_hands"] is False
    assert prepare.data["hand_plan"] == {}


def test_shipped_registry_parses():
    from pathlib import Path
    from play_action.registry import load_registry

    root = Path(__file__).resolve().parents[1] / "skills" / "motion" \
        / "play_action" / "actions"
    registry = load_registry(str(root / "actions.yaml"))
    assert registry.resolve("就绪").key == "goready"
    assert registry.resolve("goready").clip == "goready"
    assert registry.resolve("回零").key == "gohome"
    for entry in registry.entries.values():
        assert (root / entry.file).is_file(), entry.file


def test_shipped_actions_are_one_clip_per_file():
    """约定：一个动作一个文件，文件内只保留一个片段且 activeClipId 指向它。"""
    from pathlib import Path
    from play_action.action_file import load_action, select_clip
    from play_action.registry import load_registry

    root = Path(__file__).resolve().parents[1] / "skills" / "motion" \
        / "play_action" / "actions"
    registry = load_registry(str(root / "actions.yaml"))
    for entry in registry.entries.values():
        doc = load_action(str(root / entry.file))
        clips = doc.get("clips") or []
        assert len(clips) == 1, (entry.file, [c.get("name") for c in clips])
        assert clips[0].get("id") == doc.get("activeClipId")
        assert select_clip(doc).name == entry.clip


def test_multi_clip_file_warns(tmp_path):
    doc = {
        "schemaVersion": 2,
        "robot": {"name": "tienkung_dex_inspire.urdf",
                  "jointNames": sorted(LIMITS), "joints": LIMITS},
        "activeClipId": "c1",
        "clips": [
            {"id": "c1", "name": "goready", "durationMs": 100, "fps": 30,
             "loop": True, "playRange": {"startMs": 0, "endMs": 100},
             "tracks": [_track("shoulder_pitch_l_joint",
                               [(0, 0.0), (100, 0.1)])]},
            {"id": "c2", "name": "other", "durationMs": 100, "fps": 30,
             "loop": True, "playRange": {"startMs": 0, "endMs": 100},
             "tracks": [_track("shoulder_pitch_l_joint",
                               [(0, 0.0), (100, 0.2)])]},
        ],
    }
    path = tmp_path / "multi.json"
    path.write_text(json.dumps(doc), encoding="utf-8")
    ctx = make_ctx()
    prepare = _skill().build_steps(ctx, {"action": str(path)})[0].run(ctx)

    assert prepare.status == "done"
    assert prepare.data["clip"] == "goready"
    assert any("一个动作一个文件" in w for w in prepare.data["warnings"])
