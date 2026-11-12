"""MCP frontend（fastmcp）。

工具函数由技能的 ``mcp_interface/mcp_interface.py`` 提供（函数签名即 schema），
本模块只负责聚合注册与起 streamable-http 服务。
"""
from __future__ import annotations

from typing import Callable, Iterable, Optional


def create_server(name: str = "thinker_skills"):
    """创建 FastMCP server（延迟导入，未安装 fastmcp 时抛错）。"""
    from fastmcp import FastMCP  # noqa: F401
    return FastMCP(name)


def register_tools(server, tools: Iterable[Callable]) -> int:
    """把技能工具函数注册到 server，返回注册数量。"""
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


def run_server(server, *, host: str = "localhost", port: int = 8000) -> None:
    """起 streamable-http 服务；对外暴露需显式设 host 并前置反代。"""
    server.run(transport="streamable-http", host=host, port=port)
