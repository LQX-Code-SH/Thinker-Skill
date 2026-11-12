"""取消信号。"""


class Cancelled(Exception):
    """取消信号。

    技能在长步骤内部可主动抛出，Pipeline 捕获后统一转为 ``state=cancelled``。
    """
