"""Thinker-Skill 技能契约层 —— 技能作者唯一依赖。

纯 Python，无 ROS 运行时依赖，可 ``pip install -e`` 供离机单测。
"""
from .body import ArmAPI, BaseAPI, BodyAPI, GripperAPI, HeadAPI, Pose, WaistAPI
from .ctx import StepCtx
from .errors import ErrorCode
from .exceptions import Cancelled
from .sensor import Frame, SensorAPI
from .state import StateAPI
from .step import FailurePolicy, Step, StepKind, StepResult, StepStatus

__version__ = "0.1.0"

__all__ = [
    "ArmAPI",
    "BaseAPI",
    "BodyAPI",
    "Cancelled",
    "ErrorCode",
    "FailurePolicy",
    "Frame",
    "GripperAPI",
    "HeadAPI",
    "Pose",
    "SensorAPI",
    "StateAPI",
    "Step",
    "StepCtx",
    "StepKind",
    "StepResult",
    "StepStatus",
    "WaistAPI",
    "__version__",
]
