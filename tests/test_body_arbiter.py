"""body_arbiter 三区域仲裁测试。"""
import time

import pytest

from thinker_skill_core.body_arbiter import (
    BodyArbiter,
    BodyBusyError,
    LeaseExpired,
    areas_for,
)


class _Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def test_areas_for():
    assert areas_for(["body.arm", "body.gripper", "body.waist"]) == {"upper"}
    assert areas_for(["sensor.rgbd"]) == set()
    assert areas_for(["body.head", "body.base"]) == {"head", "lower"}


def test_readonly_exempt():
    arb = BodyArbiter()
    assert arb.acquire("cam", ["sensor.rgbd"]) is None
    assert arb.acquire("cam2", ["sensor.rgbd"]) is None


def test_single_area():
    arb = BodyArbiter()
    lease = arb.acquire("s1", ["body.arm"])
    assert arb.holder("upper") == "s1"
    arb.release(lease)
    assert arb.holder("upper") is None


def test_conflict_raises():
    arb = BodyArbiter()
    arb.acquire("s1", ["body.arm"])
    with pytest.raises(BodyBusyError):
        arb.acquire("s2", ["body.arm"])


def test_cross_area_parallel():
    arb = BodyArbiter()
    assert arb.acquire("arm", ["body.arm"]) is not None
    assert arb.acquire("nav", ["body.base"]) is not None
    assert arb.snapshot() == {"upper": "arm", "lower": "nav"}


def test_atomic_multi_area():
    arb = BodyArbiter()
    arb.acquire("nav", ["body.base"])
    with pytest.raises(BodyBusyError):
        arb.acquire("fetch", ["body.arm", "body.base"])
    assert arb.holder("upper") is None      # 失败后不残留


def test_ttl_expiry():
    clock = _Clock()
    arb = BodyArbiter(default_ttl=5.0, clock=clock)
    arb.acquire("s1", ["body.arm"])
    assert arb.holder("upper") == "s1"
    clock.advance(6.0)
    assert arb.holder("upper") is None


def test_renew():
    clock = _Clock()
    arb = BodyArbiter(default_ttl=5.0, clock=clock)
    lease = arb.acquire("s1", ["body.arm"])
    clock.advance(4.0)
    arb.renew(lease)
    clock.advance(4.0)
    assert arb.holder("upper") == "s1"


def test_renew_released_raises():
    arb = BodyArbiter()
    lease = arb.acquire("s1", ["body.arm"])
    arb.release(lease)
    with pytest.raises(LeaseExpired):
        arb.renew(lease)


def test_preempt():
    arb = BodyArbiter()
    arb.acquire("old", ["body.arm"])
    seen = []
    arb.preempt("new", ["body.arm"], on_preempt=lambda l: seen.append(l.owner))
    assert seen == ["old"]
    assert arb.holder("upper") == "new"


def test_wait_timeout():
    arb = BodyArbiter()
    arb.acquire("s1", ["body.arm"])
    t0 = time.monotonic()
    with pytest.raises(BodyBusyError):
        arb.acquire("s2", ["body.arm"], timeout=0.1)
    assert time.monotonic() - t0 >= 0.05
