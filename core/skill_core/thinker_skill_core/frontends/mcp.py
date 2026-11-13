"""MCP frontend（fastmcp）。

工具函数由技能的 ``mcp_interface/mcp_interface.py`` 提供（**函数签名即 schema**），
本模块负责加载工具、绑定到 host、注册到 FastMCP。
"""
from __future__ import annotations

import functools
import importlib.util
import inspect
import pathlib
import socket
import threading
from typing import Callable, Iterable, List, Tuple


def can_bind(host: str, port: int) -> bool:
    """检查端口是否可绑定（避免 MCP 线程静默失败）。"""
    bind_host = "127.0.0.1" if host in ("localhost", "") else host
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((bind_host, port))
            return True
        except OSError:
            return False


def create_server(name: str = "thinker_skills"):
    """创建 FastMCP server（延迟导入，未安装 fastmcp 时抛错）。"""
    from fastmcp import FastMCP  # noqa: F401
    return FastMCP(name)


def load_tools(skill_dir) -> List[Callable]:
    """从 ``<skill_dir>/mcp_interface/mcp_interface.py`` 加载公开工具函数。"""
    path = pathlib.Path(skill_dir) / "mcp_interface" / "mcp_interface.py"
    if not path.is_file():
        return []
    mod_name = "_thinker_mcp_" + path.parent.parent.name.replace("-", "_")
    spec = importlib.util.spec_from_file_location(mod_name, path)
    if spec is None or spec.loader is None:
        return []
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return [
        obj for name, obj in vars(module).items()
        if callable(obj) and not name.startswith("_")
        and getattr(obj, "__module__", None) == mod_name
    ]


def bind_tool(host, skill_name: str, fn: Callable) -> Callable:
    """把工具函数绑定到 ``host.execute``，保留原签名供 FastMCP 反射。"""
    @functools.wraps(fn)
    def wrapper(**kwargs):
        return tool_result(host.execute(skill_name, kwargs))

    wrapper.__signature__ = inspect.signature(fn)
    return wrapper


def register_tools(server, tools: Iterable[Callable]) -> int:
    """注册工具函数，返回数量。"""
    count = 0
    for fn in tools:
        server.tool()(fn)
        count += 1
    return count


def tool_result(result) -> dict:
    """ExecutionResult → MCP 返回值（统一信封）。"""
    return {
        "state": result.state,
        "code": result.code,
        "msg": result.msg,
        "retryable": bool(getattr(result, "retryable", False)),
        "data": result.data or {},
    }


def build_server(host, skills: Iterable[Tuple[str, str]],
                 name: str = "thinker_skills"):
    """``skills``: ``[(skill_name, skill_dir)]``；返回已注册工具的 server。"""
    server = create_server(name)
    for skill_name, skill_dir in skills:
        for fn in load_tools(skill_dir):
            server.tool()(bind_tool(host, skill_name, fn))
    return server


def run_server(server, *, host: str = "localhost", port: int = 8000) -> None:
    """起 streamable-http 服务；对外暴露需显式设 host 并前置反代。"""
    server.run(transport="streamable-http", host=host, port=port)


def start_http(server, *, host: str = "localhost", port: int = 8000,
               name: str = "mcp-http") -> threading.Thread:
    """在后台线程启动 streamable-http 服务（不阻塞 ROS executor）。"""
    def _serve():
        try:
            server.run(transport="streamable-http", host=host, port=port,
                       show_banner=False)
        except Exception as exc:  # noqa: BLE001 - 端口占用等不应拖垮 ROS 通道
            print(f"[mcp] server stopped: {exc}", flush=True)

    thread = threading.Thread(target=_serve, name=name, daemon=True)
    thread.start()
    return thread
