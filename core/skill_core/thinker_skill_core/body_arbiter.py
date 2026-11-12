"""身体仲裁：三区域互斥 + 租约（D-008 / D-010）。

- 互斥单位是区域（head / upper / lower），不拆到关节；
- 能力 → 区域映射，只读能力（sensor.*）豁免仲裁；
- 多区域原子获取（全拿或全不拿）；
- 租约带 TTL，Pipeline 每 tick 续约，到期自动释放；
- 抢占 = 释放旧持有者（回调软急停）→ 交接。
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Optional, Set

AREAS = ("head", "upper", "lower")

CAPABILITY_TO_AREA: Dict[str, str] = {
    "body.head": "head",
    "body.arm": "upper",
    "body.gripper": "upper",
    "body.waist": "upper",
    "body.base": "lower",
}


class BodyBusyError(Exception):
    """区域被占用（映射 PRECOND_BODY_BUSY）。"""

    def __init__(self, areas: Set[str], holder: Optional[str]):
        self.areas = set(areas)
        self.holder = holder
        super().__init__(f"body areas {sorted(areas)} busy (holder={holder})")


class LeaseExpired(Exception):
    pass


def areas_for(capabilities: Iterable[str]) -> Set[str]:
    """能力名 → 区域集合（只读能力返回空集）。"""
    return {CAPABILITY_TO_AREA[c] for c in capabilities if c in CAPABILITY_TO_AREA}


@dataclass
class Lease:
    owner: str
    areas: Set[str]
    ttl: float
    expires_at: float
    released: bool = False


class BodyArbiter:
    """三区域互斥的租约仲裁器（线程安全）。"""

    def __init__(self, *, default_ttl: float = 30.0, clock=time.monotonic):
        self._ttl = float(default_ttl)
        self._clock = clock
        self._cond = threading.Condition(threading.Lock())
        self._held: Dict[str, Lease] = {}

    def _now(self) -> float:
        return self._clock()

    def _sweep(self) -> None:
        now = self._now()
        for area in [a for a, l in self._held.items() if l.expires_at <= now]:
            self._held.pop(area, None)

    def acquire(
        self,
        owner: str,
        capabilities,
        *,
        ttl: Optional[float] = None,
        timeout: float = 0.0,
    ) -> Optional[Lease]:
        """原子获取区域。``timeout=0`` 失败即抛；``>0`` 等待。"""
        areas = areas_for(capabilities)
        if not areas:
            return None
        ttl = self._ttl if ttl is None else float(ttl)
        deadline = self._now() + max(0.0, timeout)
        with self._cond:
            while True:
                self._sweep()
                conflict = next((self._held[a] for a in areas if a in self._held), None)
                if conflict is None:
                    lease = Lease(owner, areas, ttl, self._now() + ttl)
                    for a in areas:
                        self._held[a] = lease
                    return lease
                if self._now() >= deadline:
                    raise BodyBusyError(areas, conflict.owner)
                self._cond.wait(min(0.05, max(0.0, deadline - self._now())))

    def renew(self, lease: Optional[Lease], *, ttl: Optional[float] = None) -> None:
        if lease is None:
            return
        with self._cond:
            if lease.released:
                raise LeaseExpired("lease already released")
            lease.ttl = self._ttl if ttl is None else float(ttl)
            lease.expires_at = self._now() + lease.ttl
            self._cond.notify_all()

    def release(self, lease: Optional[Lease]) -> None:
        if lease is None:
            return
        with self._cond:
            if lease.released:
                return
            lease.released = True
            for a in lease.areas:
                if self._held.get(a) is lease:
                    self._held.pop(a, None)
            self._cond.notify_all()

    def preempt(
        self,
        owner: str,
        capabilities,
        *,
        on_preempt: Optional[Callable[[Lease], None]] = None,
        ttl: Optional[float] = None,
    ) -> Optional[Lease]:
        """抢占：释放冲突持有者 → 回调软急停 → 获取。"""
        areas = areas_for(capabilities)
        if not areas:
            return None
        with self._cond:
            self._sweep()
            conflict = next((self._held[a] for a in areas if a in self._held), None)
            if conflict is not None:
                conflict.released = True
                for a in list(conflict.areas):
                    if self._held.get(a) is conflict:
                        self._held.pop(a, None)
                self._cond.notify_all()
        if conflict is not None and on_preempt is not None:
            try:
                on_preempt(conflict)
            except Exception:
                pass
        return self.acquire(owner, capabilities, ttl=ttl, timeout=0.1)

    def holder(self, area: str) -> Optional[str]:
        with self._cond:
            self._sweep()
            lease = self._held.get(area)
            return lease.owner if lease else None

    def snapshot(self) -> Dict[str, str]:
        with self._cond:
            self._sweep()
            return {a: l.owner for a, l in self._held.items()}
