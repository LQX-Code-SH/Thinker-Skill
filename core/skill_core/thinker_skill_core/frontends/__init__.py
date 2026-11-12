"""服务层 frontends：MCP / Action / Service。"""
from .action import SkillActionFrontend, create_actions, handle_goal
from .mcp import create_server, register_tools, run_server, tool_result
from .service import CommandServiceFrontend, create_services, handle_request

__all__ = [
    "CommandServiceFrontend",
    "SkillActionFrontend",
    "create_actions",
    "create_server",
    "create_services",
    "handle_goal",
    "handle_request",
    "register_tools",
    "run_server",
    "tool_result",
]
