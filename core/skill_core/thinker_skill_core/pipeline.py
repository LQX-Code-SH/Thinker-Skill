"""Pipeline：驱动声明式 Step 列表（D-004）。

职责：10Hz tick 驱动、进度（weight 累加）、单步超时、重试、失败策略、
取消响应、``tick_budget`` 违规告警、运行期追加步骤。

技能不直接操作 Pipeline，只产出 ``Step`` 列表。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from thinker_skill_sdk import Cancelled, ErrorCode, Step, StepCtx, StepResult


@dataclass
class Feedback:
    """一次进度反馈（映射到 Action feedback）。"""

    phase: str
    progress: float
    msg: str = ""


@dataclass
class PipelineResult:
    """Pipeline 执行结果（映射到统一信封）。"""

    state: str                       # succeed | failed | cancelled
    code: str = ErrorCode.OK
    msg: str = ""
    data: dict = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.state == "succeed"


def _is_cancelled(ctx: StepCtx) -> bool:
    if ctx.cancelled:
        return True
    try:
        ctx.check_cancel()
    except Cancelled:
        return True
    return False


def _abort(on_abort: Optional[Callable[[], None]]) -> None:
    if on_abort is not None:
        try:
            on_abort()
        except Exception:  # 软急停不应掩盖原始失败
            pass


def run_pipeline(
    steps: List[Step],
    ctx: StepCtx,
    *,
    tick: float = 0.1,
    timeout: Optional[float] = None,
    on_feedback: Optional[Callable[[Feedback], None]] = None,
    on_abort: Optional[Callable[[], None]] = None,
) -> PipelineResult:
    """驱动步骤列表直到完成、失败或取消。

    ``steps`` 与 ``ctx.append_steps`` 应指向**同一个列表**，以支持运行期追加。
    """
    warnings: List[str] = []
    total = sum(s.weight for s in steps) or 1.0
    done_w = 0.0

    def emit(phase: str, progress: float, msg: str = "") -> None:
        if on_feedback is not None:
            on_feedback(Feedback(phase=phase, progress=progress, msg=msg))

    i = 0
    while i < len(steps):           # len 动态增长，支持 ctx.append_steps
        step = steps[i]
        emit(step.name, done_w / total)

        deadline = time.monotonic() + (
            step.timeout if step.timeout is not None else (timeout or float("inf"))
        )
        retries_left = step.retries

        while True:
            if _is_cancelled(ctx):
                _abort(on_abort)
                return PipelineResult(
                    "cancelled", ErrorCode.SYS_CANCELLED, "已取消", warnings=warnings
                )

            t0 = time.monotonic()
            try:
                result = step.run(ctx)
            except Cancelled:
                _abort(on_abort)
                return PipelineResult(
                    "cancelled", ErrorCode.SYS_CANCELLED, "已取消", warnings=warnings
                )
            except Exception as exc:  # 失败不抛异常，收敛
                result = StepResult(
                    status="failed",
                    code=ErrorCode.SYS_INTERNAL_ERROR,
                    retryable=False,
                    data={"error": str(exc)},
                )
            cost = time.monotonic() - t0

            if cost > step.tick_budget:
                warnings.append(
                    f"step '{step.name}' run {cost:.3f}s > tick_budget {step.tick_budget}s"
                )

            if result.status == "done":
                break

            if result.status == "failed":
                code = result.code or ErrorCode.SYS_INTERNAL_ERROR
                if result.retryable and retries_left > 0:
                    retries_left -= 1
                    continue
                if step.on_failure == "skip":
                    warnings.append(f"step '{step.name}' skipped: {code}")
                    break
                _abort(on_abort)
                return PipelineResult(
                    "failed", code, data=result.data, warnings=warnings
                )

            # status == "running"
            if time.monotonic() > deadline:
                _abort(on_abort)
                return PipelineResult(
                    "failed",
                    ErrorCode.SYS_TIMEOUT,
                    data={"step": step.name},
                    warnings=warnings,
                )
            time.sleep(tick)

        done_w += step.weight
        emit(step.name, done_w / total)
        i += 1

    return PipelineResult("succeed", ErrorCode.OK, data={"progress": 1.0}, warnings=warnings)
