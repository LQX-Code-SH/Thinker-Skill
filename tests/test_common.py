"""common 测试：错误码渲染 + 配置合并。"""
import os
import textwrap

from thinker_common import ConfigLoader, ErrorCatalog


def test_error_catalog_builtin():
    c = ErrorCatalog()
    msg, retry = c.render("PRECOND_BODY_BUSY")
    assert "占用" in msg
    assert retry is True


def test_error_catalog_real_config():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    c = ErrorCatalog(os.path.join(root, "configs", "errors.yaml"))
    msg, retry = c.render("PERCEPTION_OBJECT_NOT_FOUND", object_name="瓶子")
    assert "瓶子" in msg
    assert retry is True


def test_error_catalog_unknown_code():
    c = ErrorCatalog()
    msg, retry = c.render("NO_SUCH_CODE")
    assert msg == ""
    assert retry is False


def test_config_loader_includes(tmp_path):
    (tmp_path / "robot.yaml").write_text("tick_rate: 10\na: {x: 1}\n", encoding="utf-8")
    (tmp_path / "main.yaml").write_text(
        textwrap.dedent("""
            includes: [robot.yaml]
            tick_rate: 20
            b: 2
        """), encoding="utf-8")
    cfg = ConfigLoader(str(tmp_path)).load()
    assert cfg["tick_rate"] == 20          # main 覆盖 include
    assert cfg["a"] == {"x": 1}            # 嵌套保留
    assert cfg["b"] == 2
