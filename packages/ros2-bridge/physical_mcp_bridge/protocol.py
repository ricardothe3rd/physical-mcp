"""Shared protocol between TS MCP server and Python bridge.

This mirrors packages/mcp-server/src/bridge/protocol.ts exactly.
"""

import json
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import Any, Optional


class CommandType(str, Enum):
    TOPIC_LIST = 'topic.list'
    TOPIC_INFO = 'topic.info'
    TOPIC_SUBSCRIBE = 'topic.subscribe'
    TOPIC_PUBLISH = 'topic.publish'
    TOPIC_ECHO = 'topic.echo'
    SERVICE_LIST = 'service.list'
    SERVICE_INFO = 'service.info'
    SERVICE_CALL = 'service.call'
    ACTION_LIST = 'action.list'
    ACTION_SEND_GOAL = 'action.send_goal'
    ACTION_CANCEL = 'action.cancel'
    ACTION_STATUS = 'action.status'
    NODE_LIST = 'node.list'
    GET_PARAMS = 'params.get'
    PING = 'ping'
    EMERGENCY_STOP = 'emergency_stop'


@dataclass
class Command:
    id: str
    type: CommandType
    params: dict = field(default_factory=dict)


def parse_command(raw: str) -> Command:
    """Parse a raw JSON string into a Command."""
    data = json.loads(raw)
    return Command(
        id=data['id'],
        type=CommandType(data['type']),
        params=data.get('params', {}),
    )


def build_response(cmd_id: Optional[str], status: str, data: Any) -> dict:
    """Build a response dict to send back over WebSocket."""
    return {
        'id': cmd_id,
        'status': status,
        'data': data,
        'timestamp': time.time(),
    }
