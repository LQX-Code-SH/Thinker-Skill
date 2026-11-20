"""异常信号。"""


class Cancelled(Exception):
    """取消信号。

    技能在长步骤内部可主动抛出，Pipeline 捕获后统一转为 ``state=cancelled``。
    """


class SkillError(RuntimeError):
    """带错误码的失败信号。

    能力层（如 IK 不可达）或技能可抛出，Pipeline 会转成 ``status=failed`` +
    对应 ``code``，而不是笼统的 ``SYS_INTERNAL_ERROR``。
    继承 ``RuntimeError`` 以兼容既有调用方。
    """

    def __init__(self, code: str, msg: str = ""):
        self.code = code
        self.msg = msg
        super().__init__(msg or code)
