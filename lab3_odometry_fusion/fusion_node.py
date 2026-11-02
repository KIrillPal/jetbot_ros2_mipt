#!/usr/bin/env python3
"""Sensor adapters + read-only comparison UI; publishes only Odometry measurements."""
from collections import deque
import copy
import csv
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import signal
import threading
import time
from urllib.parse import urlsplit, parse_qs

from fusion_math import History, alignment, aligned, paired, diagonal, valid, wrap, diagnostic_level


class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.sources = {key: History() for key in ('wheel', 'lidar', 'fused', 'raw_lidar')}
        self.alignment = {'ready': False, 'reason': 'Waiting for fresh stationary wheel/laser pair'}
        self.fault = None
        self.raw_lidar_at = None
        self.scan_at = None
        self.tf_edges = {}
        self.diagnostics = []
        self.publisher_counts = {}
        self.measurement_covariances = {}

    def snapshot(self, query=None, points=True):
        query = query or {}
        with self.lock:
            sources = {}
            for key, store in self.sources.items():
                after = int(query.get(key+'_after', ['0'])[0])
                session = query.get(key+'_session', [''])[0]
                sources[key] = store.state(after, session)
                if not points or key == 'raw_lidar':
                    sources[key].pop('points')
            now = time.monotonic()
            raw_age = None if self.raw_lidar_at is None else now-self.raw_lidar_at
            scan_age = None if self.scan_at is None else now-self.scan_at
            ages = [self.sources['wheel'].age(), self.sources['lidar'].age(), raw_age, scan_age]
            healthy = self.alignment['ready'] and self.fault is None and all(
                age is not None and age < 1.5 for age in ages)
            return dict(sources=sources, alignment=dict(self.alignment), fault=self.fault,
                        raw_lidar_age_s=raw_age, scan_age_s=scan_age,
                        tf_edges=list(self.tf_edges.values()), input_health=healthy,
                        diagnostics=list(self.diagnostics), read_only=True,
                        publisher_counts=dict(self.publisher_counts),
                        measurement_covariances=dict(self.measurement_covariances))


def handler_for(state):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def respond(self, code, kind, body):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            url = urlsplit(self.path)
            if url.path in ('/', '/app.js'):
                name = 'app.js' if url.path.endswith('.js') else 'index.html'
                kind = 'text/javascript' if name.endswith('.js') else 'text/html'
                self.respond(200, kind+'; charset=utf-8',
                             (Path(__file__).parent/'dist'/name).read_bytes())
            elif url.path in ('/api/state', '/healthz'):
                try:
                    result = state.snapshot(parse_qs(url.query), points=url.path != '/healthz')
                except ValueError:
                    self.respond(400, 'text/plain', b'Invalid cursor')
                    return
                self.respond(200, 'application/json', json.dumps(result, allow_nan=False).encode())
            elif url.path == '/api/trail.csv':
                stream = io.StringIO(newline='')
                fields = ['source', 'stamp_s', 'x', 'y', 'yaw', 'linear_x', 'linear_y', 'angular_z', 'seq']
                writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
                writer.writeheader()
                with state.lock:
                    for key, store in state.sources.items():
                        for row in store.rows:
                            writer.writerow(dict(row, source=key))
                self.respond(200, 'text/csv; charset=utf-8', stream.getvalue().encode())
            else:
                self.respond(404, 'text/plain', b'Not found')

        def reject(self):
            self.respond(405, 'text/plain', b'Read-only: no robot controls')
        do_POST = do_PUT = do_DELETE = do_PATCH = reject
    return Handler


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
    from diagnostic_msgs.msg import DiagnosticArray

    state = State()

    class Adapter(Node):
        def __init__(self):
            super().__init__('lab3_fusion_measurements')
            self.wheels = deque(maxlen=300)
            self.previous = {}
            self.publisher_ids = {}
            self.wheel_out = self.create_publisher(Odometry, '/lab3/ekf/wheel', 20)
            self.lidar_out = self.create_publisher(Odometry, '/lab3/ekf/lidar', 20)
            self.topics = {'wheel': '/diffbot_base_controller/odom', 'raw_lidar': '/odom_rf2o'}
            for key, topic in dict(self.topics, fused='/odometry/filtered').items():
                self.create_subscription(Odometry, topic, self.callback(key), qos_profile_sensor_data)
            self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
            self.create_subscription(TFMessage, '/tf', self.tf_callback(False), qos_profile_sensor_data)
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(TFMessage, '/tf_static', self.tf_callback(True), static_qos)
            self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics, 10)
            self.create_timer(1.0, self.graph)

        def fail(self, reason):
            if state.fault is None:
                state.fault = reason+'; restart ONLY the fusion container to begin a new session'
                self.get_logger().error(state.fault)

        def scan(self, msg):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            now = self.get_clock().now().nanoseconds*1e-9
            if 0 <= now-stamp < 1.5 and sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max
                                         for r in msg.ranges) >= 30:
                with state.lock:
                    state.scan_at = time.monotonic()

        def tf_callback(self, static):
            def accept(msg):
                with state.lock:
                    for t in msg.transforms:
                        state.tf_edges[(t.header.frame_id, t.child_frame_id)] = dict(
                            parent=t.header.frame_id, child=t.child_frame_id, static=static)
            return accept

        def diagnostics(self, msg):
            with state.lock:
                state.diagnostics = [dict(level=diagnostic_level(s.level), name=s.name, message=s.message)
                                     for s in msg.status if 'ekf' in s.name.lower()][:12]

        def graph(self):
            with state.lock:
                for key, topic in dict(self.topics, fused='/odometry/filtered').items():
                    ids = tuple(sorted(bytes(e.endpoint_gid).hex()
                                       for e in self.get_publishers_info_by_topic(topic)))
                    state.publisher_counts[topic] = len(ids)
                    if len(ids) > 1:
                        self.fail('Multiple publishers on '+topic)
                    if key != 'fused' and ids:
                        if self.publisher_ids.get(key) and ids != self.publisher_ids[key]:
                            self.fail('Source publisher changed: '+topic)
                        self.publisher_ids[key] = ids

        def callback(self, source):
            def accept(msg):
                with state.lock:
                    self.accept(source, msg)
            return accept

        def accept(self, source, msg):
            pose, twist = msg.pose.pose, msg.twist.twist
            q = pose.orientation
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if not math.isfinite(norm) or norm < 1e-9:
                return
            x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
            row = dict(stamp_s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       x=pose.position.x, y=pose.position.y,
                       yaw=math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),
                       linear_x=twist.linear.x, linear_y=twist.linear.y, angular_z=twist.angular.z)
            if not valid(row) or msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
                self.fail('Invalid values or unexpected frames on '+source)
                return
            now = self.get_clock().now().nanoseconds*1e-9
            if not -0.1 <= now-row['stamp_s'] <= 1.5:
                return
            previous = self.previous.get(source)
            if previous:
                dt = row['stamp_s']-previous['stamp_s']
                if dt < -0.5:
                    self.fail('Backward source clock on '+source)
                if dt <= 0:
                    return
                if source != 'fused' and dt < 0.5 and (
                    math.hypot(row['x']-previous['x'], row['y']-previous['y']) > 3*dt+0.1 or
                    abs(wrap(row['yaw']-previous['yaw'])) > 6*dt+0.2):
                    self.fail('Odometry discontinuity on '+source)
            self.previous[source] = row
            state.measurement_covariances[source] = dict(
                pose_diagonal=[float(msg.pose.covariance[7*i]) for i in range(6)],
                twist_diagonal=[float(msg.twist.covariance[7*i]) for i in range(6)])
            if source == 'fused':
                state.sources['fused'].add(row)
                return
            if source == 'wheel':
                self.wheels.append(row)
                state.sources['wheel'].add(row)
                if state.fault is None:
                    out = copy.deepcopy(msg)
                    # Documented initial uncertainty, matching installed wheel driver except vy.
                    out.twist.covariance = diagonal([0.001, 0.0025, 1e6, 1e6, 1e6, 0.01])
                    self.wheel_out.publish(out)
                return
            state.raw_lidar_at = time.monotonic()
            state.sources['raw_lidar'].add(row)
            if state.fault:
                return
            if not state.alignment['ready']:
                initial = paired(self.wheels, row['stamp_s'])
                if initial is None or len(self.wheels) < 10 or (
                    abs(initial['linear_x']) > 0.01 or abs(initial['angular_z']) > 0.03) or any(
                    abs(p['linear_x']) > 0.01 or abs(p['angular_z']) > 0.03
                    for p in list(self.wheels)[-10:]):
                    return
                state.alignment = dict(alignment(initial, row), ready=True,
                    reason='Fixed SE(2) alignment from first synchronized stationary pair; never continuously corrected',
                    wheel_pose=dict(initial), paired_stamp_s=row['stamp_s'],
                    initial_disagreement_m=math.hypot(initial['x']-row['x'], initial['y']-row['y']),
                    initial_disagreement_rad=wrap(initial['yaw']-row['yaw']))
                self.get_logger().info('Fixed initial wheel/laser alignment acquired')
            result = aligned(row, state.alignment)
            state.sources['lidar'].add(result)
            if state.scan_at is None or time.monotonic()-state.scan_at > 1.5:
                return
            out = copy.deepcopy(msg)
            out.pose.pose.position.x, out.pose.pose.position.y = result['x'], result['y']
            out.pose.pose.orientation.x = out.pose.pose.orientation.y = 0.0
            out.pose.pose.orientation.z = math.sin(result['yaw']/2)
            out.pose.pose.orientation.w = math.cos(result['yaw']/2)
            # Raw RF2O has zero covariance. Explicit tunable starting assumptions:
            # sigma_xy=5 cm; sigma_yaw=0.08 rad. XY isotropic: rotation invariant.
            out.pose.covariance = diagonal([0.0025, 0.0025, 1e6, 1e6, 1e6, 0.0064])
            out.twist.covariance = diagonal([0.01, 0.01, 1e6, 1e6, 1e6, 0.04])
            self.lidar_out.publish(out)

    rclpy.init(args=[])
    node = Adapter()
    server = ThreadingHTTPServer(('127.0.0.1', 8766), handler_for(state))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    stopping = False

    def stop(*_args):
        nonlocal stopping
        stopping = True

    # Do not destroy the old Humble executor context inside a signal handler.
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print('FUSION_VIEWER http://127.0.0.1:8766/ no motor commands, no TF publication', flush=True)
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
