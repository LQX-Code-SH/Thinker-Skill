"""look_down_table 技能测试（FakeBody，无 ROS / 无真机）。"""
import math

import pytest

from thinker_skill_core import run_pipeline

from fakes import make_ctx


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
