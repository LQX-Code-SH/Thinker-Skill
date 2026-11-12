"""thinker_skill_core —— 执行机制。

registry / runtime / pipeline / body_arbiter / state_store / frontends 逐步补齐。
"""
from .pipeline import Feedback, PipelineResult, run_pipeline

__all__ = ["Feedback", "PipelineResult", "run_pipeline"]
