"""robosim —— robot_sim MuJoCo 后端（backend: robosim）。"""
from .adapter import SimAdapter, create_adapter, load_mapping

__all__ = ["SimAdapter", "create_adapter", "load_mapping"]
