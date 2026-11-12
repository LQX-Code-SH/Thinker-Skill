"""服务层 frontends：MCP / Action / Service。"""
from .action import SkillActionFrontend, create_actions, handle_goal
from .mcp import (
    bind_tool,
    build_server,
    create_server,
    load_tools,
    register_tools,
    run_server,
    tool_result,
)
from .service import CommandServiceFrontend, create_services, handle_request

__all__ = [
    "CommandServiceFrontend",
    "SkillActionFrontend",
    "bind_tool",
    "build_server",
    "create_actions",
    "create_server",
    "create_services",
    "handle_goal",
    "handle_request",
    "load_tools",
    "register_tools",
    "run_server",
    "tool_result",
]
