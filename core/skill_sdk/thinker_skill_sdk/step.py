"""声明式步骤契约（D-004）。

技能只产出 ``Step`` 列表；循环、计时、进度、失败处理由 Pipeline 负责。
``run`` 仍是**每 tick 调用一次**的函数（声明式的是结构，不是执行方式）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Literal, Optional

if TYPE_CHECKING:  # 避免与 ctx 循环导入
    from .ctx import StepCtx

StepStatus = Literal["running", "done", "failed"]
StepKind = Literal["motion", "nav", "perception", "wait", "io"]
FailurePolicy = Literal["abort", "skip"]


@dataclass
class StepResult:
    """单次 ``run`` 的返回。"""

    status: StepStatus
    code: Optional[str] = None       # status=failed 时必填
    retryable: bool = False          # 决定 retries 是否消耗
    data: dict = field(default_factory=dict)


@dataclass
class Step:
    """一个可声明、可驱动的步骤。"""

    name: str                                     # 唯一阶段名 → feedback.phase
    run: "Callable[[StepCtx], StepResult]"        # 每 tick 调一次
    kind: StepKind = "motion"
    weight: float = 1.0                           # 进度权重
    timeout: Optional[float] = None               # 单步超时，缺省继承实际超时
    retries: int = 0
    on_failure: FailurePolicy = "abort"
    tick_budget: float = 0.1                      # 单次 run 的最长耗时（秒）
