"""服务层 frontends：MCP / Action / Service。"""
from .service import CommandServiceFrontend, create_services, handle_request

__all__ = ["CommandServiceFrontend", "create_services", "handle_request"]
