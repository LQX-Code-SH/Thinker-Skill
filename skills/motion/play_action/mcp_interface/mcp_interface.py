"""play_action 的 MCP 工具定义：**签名即 schema**（函数体由 host 注入）。

docstring 即 LLM 可见的工具描述；改动签名必须同步改 docstring。
"""
from typing import Any, Dict


def play_action(action: str, clip: str = "", speed: float = 1.0,
                transition_s: float = 2.0, play_hands: bool = True) -> Dict[str, Any]:
    """播放录制好的固定动作文件（腰、头、双臂、手指），用于打招呼/比心等固定动作。

    技能从动作文件里取关节关键帧并按时间回放；开始播放前会先用 transition_s
    秒把机器人从**当前姿态**平滑过渡到动作起始帧，不会突然跳过去。一次调用只
    播一遍，播完停在最后一帧。

    动作文件由录制端导出（schemaVersion 2 JSON）。`action` 可以是绝对路径、
    技能包 `actions/` 下的文件名（自动补 .json），或配置里的默认动作。

    **只驱动上半身**：腿部动作（hip/knee/ankle）由运控持有，文件里的腿轨迹不会
    执行；执行期间上半身被占用，失败会返回可读原因。文件时长超过技能超时（300s）
    时会被超时中断。

    Args:
        action: 动作文件名或路径，如 `goready` 或 `/data/actions/wave.json`。
        clip: 片段名或 id；留空用文件里的默认片段（activeClipId）。
        speed: 播放速度倍率，默认 1.0（0.05~3.0），2.0 表示两倍速。
        transition_s: 起步衔接时长（秒），默认 2.0，0 表示不做过渡直接跳到首帧。
        play_hands: 是否同时驱动手指，默认 true；无手部或动作文件无手部轨道时自动跳过。
    """
    raise NotImplementedError("由 host 注入执行")
