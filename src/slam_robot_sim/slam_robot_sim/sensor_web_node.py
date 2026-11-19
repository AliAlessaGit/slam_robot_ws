"""Serve the live sensor dashboard and WebSocket updates from ROS 2."""

# الوظيفة: تقديم صفحة مراقبة الحساسات وبث البيانات عبر WebSocket.
# الغرض: ربط بيانات ROS بالمتصفح لعرض مصفوفات ToF وقراءات BNO055.

import asyncio
import json
from pathlib import Path
import threading

from aiohttp import web

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from slam_robot_sim import esp32_link

from std_msgs.msg import String

DASHBOARD_PATH = Path(__file__).parent / 'web' / 'index.html'

DEFAULT_PORTS = [8080, 8081, 8082, 8083, 8084]


def frame_event(line):
    """Return a dashboard event for a raw serial JSON line, or None."""
    try:
        message = esp32_link.parse_line(line)
    except esp32_link.ProtocolError:
        return None

    if message.kind == 'frame':
        return {'type': 'frame', 'data': json.loads(line)}
    return {'type': 'status', 'text': message.text}


class SensorWebNode(Node):
    """Subscribe to parsed serial data and broadcast it to browser clients."""

    def __init__(self, *, parameter_overrides=None):
        """Start the dashboard server and subscribe to raw sensor messages."""
        super().__init__('sensor_web', parameter_overrides=parameter_overrides)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('auto_port', True)
        self._host = str(self.get_parameter('host').value)
        self._requested_port = int(self.get_parameter('port').value)
        self._auto_port = bool(self.get_parameter('auto_port').value)

        if not 0 <= self._requested_port <= 65535:
            raise ValueError('port must be between 0 and 65535')

        self._latest_frame = None
        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._server_error = None
        self._websocket_clients = set()
        self._app = web.Application()
        self._app.router.add_get('/', self._index)
        self._app.router.add_get('/ws', self._websocket)
        self._server_thread = threading.Thread(
            target=self._run_server, name='sensor-websocket', daemon=True)
        self._server_thread.start()
        self._ready.wait(timeout=10.0)
        if self._server_error is not None:
            raise RuntimeError(
                f'Cannot start dashboard server: {self._server_error}')
        if not self._ready.is_set():
            raise RuntimeError(
                'Dashboard server did not start within 10 seconds')

        self._port = self._actual_port
        self._subscription = self.create_subscription(
            String, '/sensors/raw_json', self._on_raw_json, 10)
        self.get_logger().info(
            f'Dashboard: http://localhost:{self._port} '
            f'(WebSocket: ws://localhost:{self._port}/ws)')

    async def _index(self, _request):
        return web.FileResponse(DASHBOARD_PATH)

    async def _websocket(self, request):
        socket = web.WebSocketResponse(heartbeat=30)
        await socket.prepare(request)
        self._websocket_clients.add(socket)
        if self._latest_frame is not None:
            await socket.send_json(self._latest_frame)
        try:
            async for _message in socket:
                pass
        finally:
            self._websocket_clients.discard(socket)
        return socket

    def _on_raw_json(self, message):
        event = frame_event(message.data)
        if event is None:
            return
        if event['type'] == 'frame':
            self._latest_frame = event
        try:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(event), self._loop)
        except RuntimeError:
            self.get_logger().warning('WebSocket server is shutting down')

    async def _broadcast(self, event):
        for socket in tuple(self._websocket_clients):
            try:
                await socket.send_json(event)
            except (ConnectionError, RuntimeError):
                self._websocket_clients.discard(socket)

    @property
    def _actual_port(self):
        """Return the port the server is actually running on."""
        return getattr(self, '_bound_port', self._requested_port)

    def _run_server(self):
        asyncio.set_event_loop(self._loop)
        ports_to_try = [self._requested_port]
        if self._auto_port:
            ports_to_try.extend(
                p for p in DEFAULT_PORTS if p != self._requested_port)

        last_error = None
        for port in ports_to_try:
            try:
                self._runner = web.AppRunner(self._app)
                self._loop.run_until_complete(self._runner.setup())
                site = web.TCPSite(self._runner, self._host, port)
                self._loop.run_until_complete(site.start())
                self._bound_port = port
                self._ready.set()
                break
            except OSError as exc:
                last_error = exc
                self.get_logger().warning(
                    f'Port {port} is in use, trying next port...')
                continue
            except Exception as exc:  # noqa: BLE001 - report startup errors
                last_error = exc
                break

        if last_error is not None and not self._ready.is_set():
            self._server_error = last_error
            self._ready.set()
            self._loop.close()
            return

        self._loop.run_forever()
        self._loop.run_until_complete(self._runner.cleanup())
        self._loop.close()

    def destroy_node(self):
        """Stop the dashboard server and release the ROS node."""
        if self._runner:
            try:
                self._loop.call_soon_threadsafe(self._runner.cleanup)
            except Exception:
                pass
        if self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    """Run the ROS-to-WebSocket dashboard bridge."""
    rclpy.init(args=args)
    node = SensorWebNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
