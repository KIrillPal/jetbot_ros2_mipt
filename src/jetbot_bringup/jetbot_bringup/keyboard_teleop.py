#!/usr/bin/env python3
"""WASD teleop for the in-repo teleop page.

The page posts /api/keyboard, /api/car/control, and /api/car/stop.
One browser owns the robot. A second client is rejected until the first
one releases or its status heartbeat goes stale.
"""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import re
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.signals import SignalHandlerOptions

CLIENT_ID = re.compile(r'[A-Za-z0-9-]{8,64}')
MOTION_KEYS = ('w', 'a', 's', 'd', 'shift')


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear', 0.08)
        self.declare_parameter('angular', 0.8)
        self.declare_parameter('turbo_scale', 1.5)
        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port', 8081)
        self.declare_parameter('client_timeout', 1.0)
        self.linear = float(self.get_parameter('linear').value)
        self.angular = float(self.get_parameter('angular').value)
        self.turbo_scale = float(self.get_parameter('turbo_scale').value)
        self.listen_host = str(self.get_parameter('listen_host').value)
        self.listen_port = int(self.get_parameter('listen_port').value)
        self.client_timeout = float(self.get_parameter('client_timeout').value)
        # twist_mux subscribes with SystemDefaultsQoS (BEST_EFFORT here).
        self.pub = self.create_publisher(
            Twist, 'cmd_vel_joy', qos_profile_system_default)
        self._lock = threading.Lock()
        self._owner = None
        self._last_rx = 0.0
        self._keys = {key: False for key in MOTION_KEYS}
        self._twist = Twist()
        self.create_timer(0.1, self._publish)

    def apply_key(self, client_id, key, state):
        key = key.lower()
        if key == 'control':
            key = 'ctrl'
        now = time.monotonic()
        message = None
        with self._lock:
            ok, dropped = self._claim(client_id, now)
            if dropped:
                message = 'client heartbeat lost, motors stopped'
            if not ok:
                return False, message
            if key in self._keys and self._keys[key] != bool(state):
                self._keys[key] = bool(state)
                self._recompute()
                held = ''.join(name for name in MOTION_KEYS if self._keys[name]) or '-'
                message = (
                    f'client={client_id[:8]} keys={held} '
                    f'lin={self._twist.linear.x:.2f} ang={self._twist.angular.z:.2f}'
                )
        if message:
            self.get_logger().info(message)
        return True, None

    def apply_axes(self, client_id, speed, steering):
        now = time.monotonic()
        message = None
        with self._lock:
            ok, dropped = self._claim(client_id, now)
            if dropped:
                message = 'client heartbeat lost, motors stopped'
            if not ok:
                return False, message
            self._keys['w'] = speed > 0.0
            self._keys['s'] = speed < 0.0
            self._keys['a'] = steering < 0.0
            self._keys['d'] = steering > 0.0
            self._recompute()
            held = ''.join(name for name in MOTION_KEYS if self._keys[name]) or '-'
            message = (
                f'client={client_id[:8]} keys={held} '
                f'lin={self._twist.linear.x:.2f} ang={self._twist.angular.z:.2f}'
            )
        self.get_logger().info(message)
        return True, None

    def clear_motion(self, client_id):
        now = time.monotonic()
        with self._lock:
            ok, _dropped = self._claim(client_id, now)
            if not ok:
                return False
            for key in ('w', 'a', 's', 'd', 'shift'):
                self._keys[key] = False
            self._recompute()
        self.get_logger().info('motors stopped')
        return True

    def release(self, client_id):
        now = time.monotonic()
        message = None
        with self._lock:
            self._drop_if_stale(now)
            if self._owner not in (None, client_id):
                return False
            if self._owner == client_id:
                self._owner = None
                for key in self._keys:
                    self._keys[key] = False
                self._twist = Twist()
                message = 'client released, motors stopped'
        if message:
            self.get_logger().info(message)
        return True

    def status(self, client_id):
        now = time.monotonic()
        dropped = False
        with self._lock:
            dropped = self._drop_if_stale(now)
            if self._owner == client_id:
                self._last_rx = now
            twist = Twist()
            twist.linear.x = self._twist.linear.x
            twist.angular.z = self._twist.angular.z
        if dropped:
            self.get_logger().info('client heartbeat lost, motors stopped')
        return {
            'success': True,
            'mode': 'teleoperation',
            'command': {
                'speed': twist.linear.x,
                'steering': twist.angular.z,
                'head_pan': 0.0,
                'head_tilt': 0.0,
                'head_pan_deg': 0.0,
                'head_tilt_deg': 0.0,
            },
            'telemetry': {
                'speed_mps': 0.0,
                'steering_rad': 0.0,
                'steering_deg': 0.0,
                'imu_yaw_rate': 0.0,
                'head_command_latency_ms': None,
            },
            'profile_state': {},
        }

    def stop(self):
        with self._lock:
            self._owner = None
            for key in self._keys:
                self._keys[key] = False
            self._twist = Twist()
            twist = Twist()
        if self.context.ok():
            self.pub.publish(twist)

    def _claim(self, client_id, now):
        dropped = self._drop_if_stale(now)
        if self._owner is None:
            self._owner = client_id
            self._last_rx = now
            return True, dropped
        if self._owner == client_id:
            self._last_rx = now
            return True, dropped
        return False, dropped

    def _recompute(self):
        linear = 0.0
        angular = 0.0
        if self._keys['w'] and not self._keys['s']:
            scale = self.turbo_scale if self._keys['shift'] else 1.0
            linear = self.linear * scale
        elif self._keys['s'] and not self._keys['w']:
            linear = -self.linear
        if self._keys['a'] and not self._keys['d']:
            angular = self.angular
        elif self._keys['d'] and not self._keys['a']:
            angular = -self.angular
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self._twist = twist

    def _drop_if_stale(self, now):
        if self._owner is None or now - self._last_rx <= self.client_timeout:
            return False
        self._owner = None
        for key in self._keys:
            self._keys[key] = False
        self._twist = Twist()
        return True

    def _publish(self):
        if not self.context.ok():
            return
        now = time.monotonic()
        dropped = False
        with self._lock:
            dropped = self._drop_if_stale(now)
            twist = Twist()
            twist.linear.x = self._twist.linear.x
            twist.angular.z = self._twist.angular.z
        if dropped:
            self.get_logger().info('client heartbeat lost, motors stopped')
        self.pub.publish(twist)


class KeyHandler(BaseHTTPRequestHandler):
    node = None

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path.split('?', 1)[0] != '/api/status':
            self._reply(404, {'success': False, 'error': 'not found'})
            return
        client_id = self._client_id()
        if client_id is None:
            self._reply(400, {'success': False, 'error': 'bad client id'})
            return
        self._reply(200, self.node.status(client_id))

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        client_id = self._client_id()
        if client_id is None:
            self._reply(400, {'success': False, 'error': 'bad client id'})
            return
        payload = self._read_json()
        if payload is None:
            self._reply(400, {'success': False, 'error': 'bad request'})
            return
        if path == '/api/keyboard':
            key = str(payload.get('key', '')).lower()
            ok, _message = self.node.apply_key(client_id, key, bool(payload.get('state')))
        elif path == '/api/car/control':
            try:
                speed = float(payload['speed'])
                steering = float(payload['steering'])
            except (KeyError, TypeError, ValueError):
                self._reply(400, {'success': False, 'error': 'bad request'})
                return
            ok, _message = self.node.apply_axes(client_id, speed, steering)
        elif path == '/api/car/stop':
            ok = self.node.clear_motion(client_id)
        elif path == '/api/tracking':
            if bool(payload.get('tracking')):
                ok, _message = self.node.apply_key(client_id, '', False)
            else:
                ok = self.node.release(client_id)
        else:
            self._reply(404, {'success': False, 'error': 'not found'})
            return
        if not ok:
            self._reply(409, {'success': False, 'error': 'busy'})
            return
        self._reply(200, {'success': True})

    def _client_id(self):
        client_id = self.headers.get('X-Teleop-Client', '')
        if CLIENT_ID.fullmatch(client_id) is None:
            return None
        return client_id

    def _read_json(self):
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            return None
        if length == 0:
            return {}
        if length < 0 or length > 8192:
            return None
        try:
            data = json.loads(self.rfile.read(length).decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return None
        if not isinstance(data, dict):
            return None
        return data

    def _reply(self, status, payload):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    KeyHandler.node = node
    server = ThreadingHTTPServer((node.listen_host, node.listen_port), KeyHandler)
    server.daemon_threads = True
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    node.get_logger().info(
        f'one web client for /api/keyboard on :{node.listen_port}  '
        f'linear={node.linear:.2f} m/s  angular={node.angular:.2f} rad/s'
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
