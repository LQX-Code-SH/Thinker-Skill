"""host：装配 registry / runtime / body / sensor，提供统一 execute。

三条通道（MCP / Action / Service）只需调用 ``host.execute(skill, params)``。
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

from thinker_skill_sdk import Cancelled, ErrorCode, StepCtx

from .runtime import ExecutionResult
from .state_store import DEFAULT_STATE_KEYS, StateStore


def _make_checker(event: threading.Event):
    def _check():
        if event.is_set():
            raise Cancelled()
    return _check


class Host:
    def __init__(self, *, registry, runtime, body, sensor, state=None,
                 config=None, logger=None, say=None, clock=None,
                 tick_rate: float = 10.0):
        self._registry = registry
        self._runtime = runtime
        self._body = body
        self._sensor = sensor
        self._state = state or StateStore(DEFAULT_STATE_KEYS)
        self._config = config or {}
        self._log = logger
        self._say = say or (lambda s: None)
        self._clock = clock
        self._tick_rate = tick_rate

    def _cfg_for(self, skill_name: str) -> Dict[str, Any]:
        skills_cfg = (self._config or {}).get("skills", {})
        return dict(skills_cfg.get(skill_name, {}) or {})

    def make_ctx(self, skill_name: str, params: dict,
                 cancel_event: Optional[threading.Event] = None) -> StepCtx:
        event = cancel_event or threading.Event()
        return StepCtx(
            skill=skill_name,
            params=params or {},
            cfg=self._cfg_for(skill_name),
            globals={"tick_rate": self._tick_rate},
            body=self._body,
            sensor=self._sensor,
            state=self._state,
            log=self._log,
            say=self._say,
            cancelled=False,
            check_cancel=_make_checker(event),
            now=self._clock or time.monotonic,
            append_steps=lambda steps: None,
        )

    def execute(self, skill_name: str, params: dict, *,
                cancel_event: Optional[threading.Event] = None,
                preempt: bool = False,
                on_feedback: Optional[Callable] = None) -> ExecutionResult:
        skill = self._registry.get(skill_name)
        if skill is None:
            return ExecutionResult(
                "failed", ErrorCode.INPUT_UNSUPPORTED_COMMAND,
                f"未知技能: {skill_name}")
        ctx = self.make_ctx(skill_name, params, cancel_event)
        return self._runtime.execute(
            skill, params or {}, ctx, preempt=preempt, on_feedback=on_feedback)
