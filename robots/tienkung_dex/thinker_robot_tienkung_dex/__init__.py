"""天工 DEX（tienkung_dex）机型适配。"""
from .adapter import TienkungDexAdapter, create_adapter
from .body import TienkungDexBody
from .sensor import TienkungDexSensor

__all__ = [
    "TienkungDexAdapter",
    "TienkungDexBody",
    "TienkungDexSensor",
    "create_adapter",
]
