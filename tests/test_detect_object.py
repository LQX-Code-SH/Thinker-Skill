"""detect_object 测试（假检测器 + FakeSensor，无 ROS）。"""
import numpy as np
import pytest

from thinker_skill_core import run_pipeline
from thinker_skill_sdk import ErrorCode, Frame

from fakes import FakeSensor, make_ctx

K = [600.0, 0, 320.0, 0, 600.0, 240.0, 0, 0, 1.0]
DET = [{"box": [300, 220, 340, 260], "score": 0.9, "label": "bottle"}]


class _Det:
    def __init__(self, dets):
        self.dets = dets

    def detect(self, image, prompt):
        return list(self.dets)


class _Sensor(FakeSensor):
    def transform_to_base(self, p):
        return np.asarray(p, float) + np.array([0.5, 0.0, -0.2])


def _frame(depth=1000, k=K):
    return Frame(
        color=np.zeros((480, 640, 3), np.uint8),
        depth=np.full((480, 640), depth, np.uint16),
        stamp=0.0, frame_id="ob_camera_head", k=k)


def _run(dets, params=None, frame=None):
    from detect_object.plugin import DetectObject
    ctx = make_ctx(sensor=_Sensor(frames=[frame or _frame()]))
    steps = DetectObject(detector=_Det(dets)).build_steps(ctx, params or {})
    return run_pipeline(steps, ctx)


def test_hit_center():
    r = _run(DET)
    assert r.ok
    pose = r.data["obj_pose"]
    assert pose[0] == pytest.approx(0.5, abs=1e-3)
    assert pose[2] == pytest.approx(0.83, abs=0.01)


def test_not_found():
    r = _run([])
    assert r.state == "failed"
    assert r.code == ErrorCode.PERCEPTION_OBJECT_NOT_FOUND


def test_toy_alias():
    r = _run([{"box": [300, 220, 340, 260], "score": 0.8,
               "label": "plush toy"}], {"object_name": "toy"})
    assert r.ok and r.data["object_name"] == "plush toy"


def test_invalid_depth():
    r = _run(DET, frame=_frame(depth=0))
    assert r.state == "failed"
    assert r.code == ErrorCode.PERCEPTION_POSE_INVALID


def test_out_of_range():
    r = _run(DET, frame=_frame(depth=3000))
    assert r.state == "failed"
    assert r.code == ErrorCode.PERCEPTION_POSE_INVALID


def test_does_not_move_head():
    """检测技能只读：不改变头部姿态。"""
    from detect_object.plugin import DetectObject

    ctx = make_ctx(sensor=_Sensor(frames=[_frame()]))
    before = ctx.body.head.get_pose()
    steps = DetectObject(detector=_Det(DET)).build_steps(ctx, {})
    run_pipeline(steps, ctx)
    assert ctx.body.head.get_pose() == before
