"""look_down_table 技能测试（FakeBody，无 ROS / 无真机）。"""
import math

import pytest

from thinker_skill_core import run_pipeline

from fakes import FakeBody, make_ctx


def _run(params):
    from look_down_table.plugin import LookDownTable

    ctx = make_ctx()
    result = run_pipeline(LookDownTable().build_steps(ctx, params), ctx)
    return ctx, result


def test_default_50deg():
    ctx, r = _run({})
    assert r.ok
    assert ctx.body.head.pitch == pytest.approx(math.radians(50), abs=1e-6)
    assert ctx.body.head.yaw == 0.0


def test_custom_angle():
    ctx, _ = _run({"pitch_deg": 30})
    assert ctx.body.head.pitch == pytest.approx(math.radians(30), abs=1e-6)


def test_clamp_upper():
    ctx, _ = _run({"pitch_deg": 90})
    assert ctx.body.head.pitch == pytest.approx(math.radians(58), abs=1e-6)


def test_clamp_lower():
    ctx, _ = _run({"pitch_deg": -30})
    assert ctx.body.head.pitch == pytest.approx(math.radians(-15), abs=1e-6)


class _SlowHead:
    """模拟真机：实际角度逐步逼近目标。"""

    def __init__(self):
        self.target = (0.0, 0.0)
        self._cur = (0.0, 0.0)

    def look_at(self, *, yaw, pitch):
        self.target = (yaw, pitch)

    def look_down(self):
        self.look_at(yaw=0.0, pitch=0.6)

    def get_pose(self):
        cy, cp = self._cur
        ty, tp = self.target
        self._cur = ((cy + ty) / 2, (cp + tp) / 2)
        return self._cur


def test_waits_until_reached():
    from look_down_table.plugin import LookDownTable

    body = FakeBody()
    body.head = _SlowHead()
    ctx = make_ctx(body=body)
    r = run_pipeline(LookDownTable().build_steps(ctx, {}), ctx, tick=0.001)
    assert r.ok
    assert r.data["actual_pitch_deg"] == pytest.approx(50.0, abs=2.0)
