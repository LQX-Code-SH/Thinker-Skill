"""Pipeline 行为测试（无 ROS / 无真机）。"""
import time

from thinker_skill_core import run_pipeline
from thinker_skill_sdk import Cancelled, ErrorCode, Step, StepResult

from fakes import make_ctx


def _done(name="s", weight=1.0, **kw):
    return Step(name=name, run=lambda ctx: StepResult(status="done"), weight=weight, **kw)


def test_all_success():
    steps = [_done("a"), _done("b")]
    r = run_pipeline(steps, make_ctx())
    assert r.state == "succeed"
    assert r.code == ErrorCode.OK


def test_feedback_progress():
    seen = []
    steps = [_done("a", weight=1), _done("b", weight=1)]
    run_pipeline(steps, make_ctx(), on_feedback=lambda f: seen.append((f.phase, f.progress)))
    assert seen[-1] == ("b", 1.0)


def test_failed_abort():
    def fail(ctx):
        return StepResult(status="failed", code=ErrorCode.MOTION_GRASP_FAILED)

    r = run_pipeline([Step(name="grasp", run=fail)], make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.MOTION_GRASP_FAILED


def test_failed_skip():
    def fail(ctx):
        return StepResult(status="failed", code="X")

    steps = [Step(name="a", run=fail, on_failure="skip"), _done("b")]
    r = run_pipeline(steps, make_ctx())
    assert r.state == "succeed"
    assert any("skipped" in w for w in r.warnings)


def test_retry_then_success():
    calls = {"n": 0}

    def flaky(ctx):
        calls["n"] += 1
        if calls["n"] < 3:
            return StepResult(status="failed", code="TEMP", retryable=True)
        return StepResult(status="done")

    r = run_pipeline([Step(name="flaky", run=flaky, retries=2)], make_ctx())
    assert r.state == "succeed"
    assert calls["n"] == 3


def test_timeout():
    steps = [Step(name="hang", run=lambda c: StepResult(status="running"), timeout=0.05)]
    t0 = time.monotonic()
    r = run_pipeline(steps, make_ctx(), tick=0.005)
    assert r.state == "failed"
    assert r.code == ErrorCode.SYS_TIMEOUT
    assert time.monotonic() - t0 < 1.0


def test_cancel_flag():
    r = run_pipeline([_done("a")], make_ctx(cancelled=True))
    assert r.state == "cancelled"
    assert r.code == ErrorCode.SYS_CANCELLED


def test_cancel_exception():
    def raise_cancel(ctx):
        raise Cancelled()

    r = run_pipeline([Step(name="a", run=raise_cancel)], make_ctx())
    assert r.state == "cancelled"


def test_tick_budget_warning():
    def slow(ctx):
        time.sleep(0.02)
        return StepResult(status="done")

    r = run_pipeline([Step(name="slow", run=slow, tick_budget=0.001)], make_ctx())
    assert r.state == "succeed"
    assert any("tick_budget" in w for w in r.warnings)


def test_dynamic_append():
    steps = []
    ctx = make_ctx(steps=steps)
    appended = {"done": False}

    def first(c):
        if not appended["done"]:
            appended["done"] = True
            c.append_steps([Step(name="ext", run=lambda cc: StepResult(status="done"))])
        return StepResult(status="done")

    steps.append(Step(name="first", run=first))
    r = run_pipeline(steps, ctx)
    assert r.state == "succeed"
    assert [s.name for s in steps] == ["first", "ext"]


def test_exception_converged():
    def boom(ctx):
        raise RuntimeError("boom")

    r = run_pipeline([Step(name="boom", run=boom)], make_ctx())
    assert r.state == "failed"
    assert r.code == ErrorCode.SYS_INTERNAL_ERROR
