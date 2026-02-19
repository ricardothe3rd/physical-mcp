"""Main bridge node: rclpy + WebSocket server.

Receives commands from the TypeScript MCP server over WebSocket
and executes them against the ROS2 graph.
"""

import asyncio
import json
import os
import signal
import sys
import threading

import rclpy
from rclpy.node import Node
import websockets
from websockets.server import serve

from .protocol import CommandType, parse_command, build_response
from .discovery import DiscoveryHandler
from .topic_handler import TopicHandler
from .service_handler import ServiceHandler
from .action_handler import ActionHandler
from .telemetry import TelemetryReporter


class BridgeNode(Node):
    """ROS2 node that serves as the WebSocket bridge."""

    def __init__(self):
        super().__init__('physical_mcp_bridge')
        self.get_logger().info('PhysicalMCP Bridge starting...')

        self.discovery = DiscoveryHandler(self)
        self.topics = TopicHandler(self)
        self.services = ServiceHandler(self)
        self.actions = ActionHandler(self)
        self.telemetry = TelemetryReporter()

        # E-stop flag (secondary safety, primary is in TS server)
        self.emergency_stop = False

    async def handle_command(self, raw: str) -> str:
        """Parse and execute a command, return JSON response."""
        try:
            cmd = parse_command(raw)
        except Exception as e:
            return json.dumps(build_response(None, 'error', {'error': f'Parse error: {e}'}))

        self.telemetry.record_command()

        try:
            data = await self._dispatch(cmd.type, cmd.params)
            return json.dumps(build_response(cmd.id, 'ok', data))
        except Exception as e:
            self.telemetry.record_error()
            self.get_logger().error(f'Command {cmd.type} failed: {e}')
            return json.dumps(build_response(cmd.id, 'error', {'error': str(e)}))

    async def _dispatch(self, cmd_type: CommandType, params: dict):
        """Route command to the right handler."""
        match cmd_type:
            # Discovery
            case CommandType.TOPIC_LIST:
                return self.discovery.list_topics()
            case CommandType.TOPIC_INFO:
                return self.discovery.get_topic_info(params['topic'])
            case CommandType.SERVICE_LIST:
                return self.discovery.list_services()
            case CommandType.SERVICE_INFO:
                return self.discovery.get_service_info(params['service'])
            case CommandType.ACTION_LIST:
                return self.discovery.list_actions()
            case CommandType.NODE_LIST:
                return self.discovery.list_nodes()

            # Topics
            case CommandType.TOPIC_SUBSCRIBE:
                return self.topics.subscribe(
                    params['topic'],
                    params['message_type'],
                    params.get('count', 1),
                    params.get('timeout_sec', 5.0),
                )
            case CommandType.TOPIC_ECHO:
                return self.topics.echo(
                    params['topic'],
                    params['message_type'],
                    params.get('timeout_sec', 3.0),
                )
            case CommandType.TOPIC_PUBLISH:
                if self.emergency_stop:
                    return {'error': 'Emergency stop active on bridge'}
                return self.topics.publish(
                    params['topic'],
                    params['message_type'],
                    params['message'],
                )

            # Services
            case CommandType.SERVICE_CALL:
                if self.emergency_stop:
                    return {'error': 'Emergency stop active on bridge'}
                return self.services.call(
                    params['service'],
                    params['service_type'],
                    params.get('args', {}),
                )

            # Actions
            case CommandType.ACTION_SEND_GOAL:
                if self.emergency_stop:
                    return {'error': 'Emergency stop active on bridge'}
                return self.actions.send_goal(
                    params['action'],
                    params['action_type'],
                    params['goal'],
                )
            case CommandType.ACTION_CANCEL:
                return self.actions.cancel_goal(
                    params['action'],
                    params.get('goal_id'),
                )
            case CommandType.ACTION_STATUS:
                return self.actions.get_status(params['action'])

            # System
            case CommandType.PING:
                return {
                    'pong': True,
                    'bridge': 'physical-mcp-bridge',
                    'version': '0.1.0',
                    'telemetry': self.telemetry.get_status(),
                }
            case CommandType.GET_PARAMS:
                return {}

            # Emergency stop
            case CommandType.EMERGENCY_STOP:
                self.emergency_stop = True
                self.actions.cancel_all()
                # Publish zero velocity
                try:
                    self.topics.publish(
                        '/cmd_vel',
                        'geometry_msgs/msg/Twist',
                        {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
                    )
                except Exception:
                    pass
                self.get_logger().warn('EMERGENCY STOP ACTIVATED')
                return {'emergency_stop': True}

            case _:
                return {'error': f'Unknown command: {cmd_type}'}


async def ws_handler(bridge: BridgeNode, websocket):
    """Handle a single WebSocket connection."""
    bridge.get_logger().info(f'Client connected: {websocket.remote_address}')
    try:
        async for message in websocket:
            response = await bridge.handle_command(message)
            await websocket.send(response)
    except websockets.ConnectionClosed:
        bridge.get_logger().info('Client disconnected')


async def run_server(bridge: BridgeNode, host: str = '0.0.0.0', port: int = 9090,
                     shutdown_event: asyncio.Event | None = None):
    """Run the WebSocket server."""
    bridge.get_logger().info(f'WebSocket server starting on ws://{host}:{port}')

    if shutdown_event is None:
        shutdown_event = asyncio.Event()

    async with serve(lambda ws: ws_handler(bridge, ws), host, port):
        await shutdown_event.wait()


def spin_ros(bridge: BridgeNode):
    """Spin ROS2 node in a separate thread."""
    rclpy.spin(bridge)


def _setup_signal_handlers(bridge: BridgeNode, shutdown_event: asyncio.Event,
                           loop: asyncio.AbstractEventLoop):
    """Register SIGTERM/SIGINT handlers for graceful shutdown."""

    def _handle_signal(signum, frame):
        sig_name = signal.Signals(signum).name
        bridge.get_logger().info(f'Shutdown requested via {sig_name}')

        # Cancel all active goals
        bridge.actions.cancel_all()

        # Publish zero velocity as safety measure
        try:
            bridge.topics.publish(
                '/cmd_vel',
                'geometry_msgs/msg/Twist',
                {'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                 'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}},
            )
        except Exception:
            pass

        # Signal the server to stop
        loop.call_soon_threadsafe(shutdown_event.set)

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)


def main():
    rclpy.init()
    bridge = BridgeNode()

    # Spin ROS2 in background thread
    ros_thread = threading.Thread(target=spin_ros, args=(bridge,), daemon=True)
    ros_thread.start()

    host = os.environ.get('PHYSICAL_MCP_BRIDGE_HOST', '0.0.0.0')
    port = int(os.environ.get('PHYSICAL_MCP_BRIDGE_PORT', '9090'))

    bridge.get_logger().info(f'PhysicalMCP Bridge v0.1.0 ready on ws://{host}:{port}')

    shutdown_event = asyncio.Event()
    loop = asyncio.new_event_loop()

    _setup_signal_handlers(bridge, shutdown_event, loop)

    try:
        loop.run_until_complete(run_server(bridge, host, port, shutdown_event))
    except KeyboardInterrupt:
        bridge.get_logger().info('Bridge shutting down...')
    finally:
        loop.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
