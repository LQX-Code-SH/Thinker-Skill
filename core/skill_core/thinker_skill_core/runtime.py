"""runtime：技能单一执行出口（能力匹配 → 租约 → Pipeline → 状态写回）。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from thinker_skill_sdk import ErrorCode

from .body_arbiter import BodyArbiter, BodyBusyError
from .pipeline import run_pipeline
from .state_store import DEFAULT_STATE_KEYS, StateStore


@dataclass
class ExecutionResult:
    state: str
    code: str = ErrorCode.OK
    msg: str = ""
    data: dict = field(default_factory=dict)
    retryable: bool = False

    @property
    def ok(self) -> bool:
        return self.state == "succeed"


class SkillRuntime:
    def __init__(self, *, arbiter=None, state=None, capabilities=None,
                 tick: float = 0.1, estop: Optional[Callable[[], None]] = None,
                 catalog=None):
        self._arbiter = arbiter or BodyArbiter()
        self._state = state or StateStore(DEFAULT_STATE_KEYS)
        self._capabilities = set(capabilities) if capabilities else set()
        self._tick = tick
        self._estop = estop
        self._catalog = catalog

    def _on_preempt(self, _lease):
        if self._estop:
            try:
                self._estop()
            except Exception:
                pass

    def execute(self, skill, params, ctx, *, timeout=None, preempt: bool = False,
                on_feedback=None) -> ExecutionResult:
        requires = list(getattr(skill, "requires", []) or [])
        if self._capabilities:
            missing = [c for c in requires if c not in self._capabilities]
            if missing:
                return ExecutionResult(
                    "failed", ErrorCode.PRECOND_CAPABILITY_MISSING,
                    data={"missing": missing})

        try:
            if preempt:
                lease = self._arbiter.preempt(
                    skill.name, requires, on_preempt=self._on_preempt)
            else:
                lease = self._arbiter.acquire(skill.name, requires)
        except BodyBusyError as exc:
            return ExecutionResult(
                "failed", ErrorCode.PRECOND_BODY_BUSY,
                data={"areas": sorted(exc.areas), "holder": exc.holder})

        try:
            steps = skill.build_steps(ctx, params)
            ctx.append_steps = steps.extend
            result = run_pipeline(
                steps, ctx, tick=self._tick,
                timeout=timeout if timeout is not None else getattr(skill, "timeout", None),
                on_feedback=on_feedback, on_abort=self._estop)
        except Exception as exc:
            return ExecutionResult(
                "failed", ErrorCode.SYS_INTERNAL_ERROR, data={"error": str(exc)})
        finally:
            self._arbiter.release(lease)

        if result.ok:
            for key, value in (getattr(skill, "postcondition", {}) or {}).items():
                self._state.set(key, value)

        msg = result.msg
        retryable = False
        if self._catalog is not None:
            rendered, retryable = self._catalog.render(result.code, **(result.data or {}))
            if not msg:
                msg = rendered
        return ExecutionResult(result.state, result.code, msg, result.data, retryable)
