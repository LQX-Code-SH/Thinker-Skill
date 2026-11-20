"""runtime：技能单一执行出口（能力匹配 → 租约 → Pipeline → 状态写回）。"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from thinker_skill_sdk import ErrorCode

from .body_arbiter import BodyArbiter, BodyBusyError
from .pipeline import run_pipeline
from .state_store import DEFAULT_STATE_KEYS, StateStore


@dataclass(eq=False)
class _Watch:
    deadline: float
    on_timeout: Callable[[], None]
    done: bool = False


class _Watchdog:
    """全局看门狗：单线程轮询，超时触发回调。

    只守护持身体租约的技能；Pipeline 自身超时先失效时，它负责兜底
    （例如 ``Step.run`` 阻塞导致 Pipeline 无法按 tick 检查 deadline）。
    """

    def __init__(self, interval: float = 0.05):
        self._interval = float(interval)
        self._lock = threading.Lock()
        self._items: list = []
        self._thread: Optional[threading.Thread] = None

    def add(self, deadline: float, on_timeout: Callable[[], None]) -> _Watch:
        item = _Watch(deadline, on_timeout)
        with self._lock:
            self._items.append(item)
        self._ensure_thread()
        return item

    def remove(self, item: _Watch) -> None:
        with self._lock:
            item.done = True
            if item in self._items:
                self._items.remove(item)

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._loop, name="skill-watchdog", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while True:
            time.sleep(self._interval)
            now = time.monotonic()
            with self._lock:
                due = [i for i in self._items if not i.done and i.deadline <= now]
            for item in due:
                with self._lock:
                    if item.done:
                        continue
                    item.done = True
                    if item in self._items:
                        self._items.remove(item)
                try:
                    item.on_timeout()
                except Exception:  # noqa: BLE001 - 看门狗不能因回调异常退出
                    pass


_WATCHDOG = _Watchdog()


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
                 catalog=None, default_timeout: float = 300.0,
                 watchdog_factor: float = 1.2):
        self._arbiter = arbiter or BodyArbiter()
        self._state = state or StateStore(DEFAULT_STATE_KEYS)
        self._capabilities = set(capabilities) if capabilities else set()
        self._tick = tick
        self._estop = estop
        self._catalog = catalog
        self._default_timeout = float(default_timeout)
        self._watchdog_factor = float(watchdog_factor)

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

        effective_timeout = timeout if timeout is not None else getattr(
            skill, "timeout", None)
        if effective_timeout is None:
            effective_timeout = self._default_timeout

        reason: dict = {}
        watch: Optional[_Watch] = None
        if lease is not None and effective_timeout and effective_timeout > 0:
            def _on_timeout():
                reason["timeout"] = True
                ctx.cancelled = True
                if self._estop is not None:
                    try:
                        self._estop()
                    except Exception:  # noqa: BLE001 - 软急停失败不掩盖超时
                        pass
                # 立即释放，避免看门狗兜底后身体仍被长期占用
                self._arbiter.release(lease)

            watch = _WATCHDOG.add(
                time.monotonic() + float(effective_timeout) * self._watchdog_factor,
                _on_timeout)

        try:
            steps = skill.build_steps(ctx, params)
            ctx.append_steps = steps.extend
            result = run_pipeline(
                steps, ctx, tick=self._tick,
                timeout=effective_timeout,
                on_feedback=on_feedback, on_abort=self._estop)
        except Exception as exc:
            return ExecutionResult(
                "failed", ErrorCode.SYS_INTERNAL_ERROR, data={"error": str(exc)})
        finally:
            if watch is not None:
                _WATCHDOG.remove(watch)
            self._arbiter.release(lease)

        state, code, data = result.state, result.code, result.data
        if state == "cancelled" and reason.get("timeout"):
            state, code = "failed", ErrorCode.SYS_TIMEOUT
            data = {"skill": skill.name, "timeout": effective_timeout}

        if state == "succeed":
            for key, value in (getattr(skill, "postcondition", {}) or {}).items():
                self._state.set(key, value)

        msg = result.msg
        retryable = False
        if self._catalog is not None:
            rendered, retryable = self._catalog.render(code, **(data or {}))
            if not msg:
                msg = rendered
        return ExecutionResult(state, code, msg, data, retryable)
