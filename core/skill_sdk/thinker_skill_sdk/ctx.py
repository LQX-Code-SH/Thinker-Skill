"""技能执行上下文（能力注入，D-005）。

技能经 ``StepCtx`` 获取 body / sensor / state / 配置，不 import 任何 core 实现包。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, Mapping, Optional

if TYPE_CHECKING:
    from .body import BodyAPI
    from .sensor import SensorAPI
    from .state import StateAPI


@dataclass
class StepCtx:
    skill: str                                     # 技能名
    params: dict                                   # 本次调用入参
    cfg: Mapping[str, Any]                         # 仅本技能私有配置（只读）
    globals: Mapping[str, Any]                     # 全局白名单项（open_nav / tick_rate）

    body: "BodyAPI"                                # 抽象能力，不暴露具体控制器
    sensor: "SensorAPI"                            # rgbd() / transform_to_base()

    state: "StateAPI"                              # get/set，key 白名单
    log: Any                                       # 结构化日志（带 skill + trace_id）
    say: Callable[[str], None]                     # 语音播报

    cancelled: bool                                # 取消标志
    check_cancel: Callable[[], None]               # 抛 Cancelled，长步骤内部也要调
    now: Callable[[], float]                       # 可注入假时钟
    append_steps: Callable[[list], None]           # 运行期追加步骤
    call_skill: Optional[Callable[..., Any]] = None  # 调用其他技能（场景技能用）
