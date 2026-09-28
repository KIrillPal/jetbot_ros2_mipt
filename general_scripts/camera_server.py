#!/usr/bin/env python3
"""Live camera server for the Jetson IMX219.

Capture, scale, and JPEG encode stay on the ISP/GPU. HTTP only forwards the
newest frame, so a slow browser cannot stall the camera.

    nvarguscamerasrc (NVMM NV12, 1280x720 @ 60)
        -> nvvidconv (NVMM I420, preview size)
        -> nvjpegenc
        -> appsink (1 buffer, drop old, no clock sync)
        -> latest-frame slot
        -> multipart MJPEG on /stream.mjpg
"""

import argparse
import json
import re
import signal
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from jinja2 import Environment, FileSystemLoader, select_autoescape

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst


BOUNDARY = b'frame'
TEMPLATE_DIR = Path(__file__).resolve().parent / 'web' / 'templates'
CLIENT_ID = re.compile(r'[A-Za-z0-9-]{8,64}')
DRIVE_PATHS = frozenset((
    '/api/keyboard',
    '/api/car/control',
    '/api/car/stop',
    '/api/tracking',
))


def render_teleop_page():
    env = Environment(
        loader=FileSystemLoader(str(TEMPLATE_DIR)),
        autoescape=select_autoescape(['html']),
    )

    def url_for(endpoint, **_kwargs):
        if endpoint == 'video_feed':
            return '/video_feed'
        return '/'

    page = env.get_template('teleop.html').render(active_tab='teleop', url_for=url_for)
    return page.encode('utf-8')


def empty_status(x, y):
    return {
        'success': True,
        'x': x,
        'y': y,
        'mode': 'teleoperation',
        'command': {
            'speed': 0.0,
            'steering': 0.0,
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


class FrameHub:
    """One slot. Publishers overwrite; readers always take the newest JPEG."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._stopped = False
        self._capture_count = 0
        self._publish_count = 0
        self._window_start = time.monotonic()
        self._window_capture = 0
        self._window_publish = 0
        self.capture_fps = 0.0
        self.stream_fps = 0.0
        self.jpeg_bytes = 0
        self.clients = 0

    def stop(self):
        with self._cond:
            self._stopped = True
            self._cond.notify_all()

    def note_capture(self):
        with self._cond:
            self._capture_count += 1
            self._window_capture += 1
            self._roll()

    def publish(self, jpeg):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self.jpeg_bytes = len(jpeg)
            self._publish_count += 1
            self._window_publish += 1
            self._roll()
            self._cond.notify_all()

    def wait_next(self, seq, timeout):
        with self._cond:
            if self._seq == seq and not self._stopped:
                self._cond.wait(timeout)
            return self._stopped, self._seq, self._jpeg

    def snapshot(self):
        with self._cond:
            return {
                'capture_fps': round(self.capture_fps, 1),
                'stream_fps': round(self.stream_fps, 1),
                'frames_captured': self._capture_count,
                'frames_published': self._publish_count,
                'jpeg_bytes': self.jpeg_bytes,
                'clients': self.clients,
                'seq': self._seq,
            }

    def _roll(self):
        now = time.monotonic()
        elapsed = now - self._window_start
        if elapsed < 1.0:
            return
        self.capture_fps = self._window_capture / elapsed
        self.stream_fps = self._window_publish / elapsed
        self._window_capture = 0
        self._window_publish = 0
        self._window_start = now


class CameraPipeline:
    def __init__(self, hub, sensor_id, width, height, fps, quality):
        self.hub = hub
        self.fps = fps
        self._stop = threading.Event()
        self._thread = None
        self._pipeline = None
        source_mode, source_w, source_h, source_fps = _source_mode(width, height)
        pipeline = (
            f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={source_mode} ! '
            f'video/x-raw(memory:NVMM),width={source_w},height={source_h},'
            f'format=NV12,framerate={source_fps}/1 ! '
            'nvvidconv ! '
            f'video/x-raw(memory:NVMM),width={width},height={height},format=I420 ! '
            f'nvjpegenc quality={quality} ! '
            'appsink name=sink max-buffers=1 drop=true sync=false'
        )
        self._description = pipeline

    def start(self):
        self._pipeline = Gst.parse_launch(self._description)
        self._sink = self._pipeline.get_by_name('sink')
        self._bus = self._pipeline.get_bus()
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('camera pipeline failed to start')
        self._thread = threading.Thread(target=self._run, name='camera', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._pipeline is not None:
            self._pipeline.set_state(Gst.State.NULL)
        self.hub.stop()

    def _run(self):
        period = 1.0 / self.fps
        next_due = 0.0
        while not self._stop.is_set():
            message = self._bus.pop_filtered(
                Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    print(f'camera error: {error}', file=sys.stderr)
                    if debug:
                        print(debug, file=sys.stderr)
                self._stop.set()
                self.hub.stop()
                break
            sample = self._sink.emit('try-pull-sample', 100 * Gst.MSECOND)
            if sample is None:
                continue
            self.hub.note_capture()
            now = time.monotonic()
            if now < next_due:
                continue
            next_due += period
            if next_due <= now:
                next_due = now + period
            buffer = sample.get_buffer()
            ok, info = buffer.map(Gst.MapFlags.READ)
            if not ok:
                continue
            try:
                jpeg = bytes(info.data)
            finally:
                buffer.unmap(info)
            self.hub.publish(jpeg)


def _source_mode(width, height):
    """Pick an IMX219 mode that is at least as large as the preview."""
    if width <= 1280 and height <= 720:
        return 4, 1280, 720, 60
    return 2, 1920, 1080, 30


class Handler(BaseHTTPRequestHandler):
    hub = None
    page = b''
    teleop_port = 8081
    head_lock = threading.Lock()
    heads = {}

    def log_message(self, fmt, *args):
        path = self.path.split('?', 1)[0]
        if path in ('/stream.mjpg', '/video_feed', '/api/status'):
            return
        super().log_message(fmt, *args)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        client, cookie = self._client()
        if path in ('/', '/teleop'):
            self._bytes(200, 'text/html; charset=utf-8', self.page, cookie)
        elif path in ('/stream.mjpg', '/video_feed'):
            self._stream()
        elif path == '/snapshot.jpg':
            _, _, jpeg = self.hub.wait_next(0, 2.0)
            if not jpeg:
                self._bytes(503, 'text/plain', b'no frame yet\n')
                return
            self._bytes(200, 'image/jpeg', jpeg)
        elif path == '/stats':
            body = json.dumps(self.hub.snapshot()).encode('utf-8')
            self._bytes(200, 'application/json', body)
        elif path == '/api/status':
            self._status(client, cookie)
        else:
            self._bytes(404, 'text/plain', b'not found\n')

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        client, cookie = self._client()
        body = self._raw_body()
        if body is None:
            self._json(400, {'success': False, 'error': 'bad request'}, cookie)
            return
        if path in DRIVE_PATHS:
            status, payload = self._proxy('POST', path, body, client)
            if path == '/api/tracking' and status == 200:
                try:
                    tracking = json.loads(body.decode('utf-8')).get('tracking')
                except (UnicodeDecodeError, json.JSONDecodeError, AttributeError):
                    tracking = None
                if tracking is False:
                    with self.head_lock:
                        self.heads.pop(client, None)
            self._json(status, payload, cookie)
            return
        if path == '/api/mode':
            self._json(200, {'success': True, 'mode': 'teleoperation'}, cookie)
            return
        if path == '/api/position':
            self._position(client, cookie, body)
            return
        if path == '/api/reset':
            with self.head_lock:
                self.heads[client] = {'x': 0.0, 'y': 0.0}
            self._json(200, {'success': True, 'x': 0, 'y': 0}, cookie)
            return
        if path in ('/api/capture', '/api/action'):
            self._json(200, {'success': True}, cookie)
            return
        self._bytes(404, 'text/plain', b'not found\n')

    def _client(self):
        header = self.headers.get('Cookie', '')
        for part in header.split(';'):
            part = part.strip()
            if part.startswith('teleop_client='):
                value = part.split('=', 1)[1]
                if CLIENT_ID.fullmatch(value):
                    return value, None
        client = str(uuid.uuid4())
        return client, client

    def _head(self, client):
        with self.head_lock:
            return self.heads.setdefault(client, {'x': 0.0, 'y': 0.0})

    def _position(self, client, cookie, body):
        try:
            data = json.loads(body.decode('utf-8'))
            dx = float(data['dx'])
            dy = float(data['dy'])
        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
            self._json(400, {'success': False, 'error': 'bad request'}, cookie)
            return
        head = self._head(client)
        with self.head_lock:
            head['x'] = max(-200.0, min(200.0, head['x'] + dx))
            head['y'] = max(-200.0, min(200.0, head['y'] + dy))
            x, y = head['x'], head['y']
        self._json(200, {'success': True, 'x': round(x), 'y': round(y)}, cookie)

    def _status(self, client, cookie):
        status, payload = self._proxy('GET', '/api/status', b'', client)
        head = self._head(client)
        if status != 200 or not isinstance(payload, dict):
            payload = empty_status(head['x'], head['y'])
        else:
            payload['x'] = head['x']
            payload['y'] = head['y']
        self._json(200, payload, cookie)

    def _proxy(self, method, path, body, client):
        request = urllib.request.Request(
            f'http://127.0.0.1:{self.teleop_port}{path}',
            data=body if method == 'POST' else None,
            method=method,
            headers={
                'Content-Type': 'application/json',
                'X-Teleop-Client': client,
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=0.5) as response:
                payload = json.loads(response.read().decode('utf-8'))
                return response.status, payload
        except urllib.error.HTTPError as error:
            try:
                payload = json.loads(error.read().decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError):
                payload = {'success': False, 'error': 'teleop error'}
            return error.code, payload
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError):
            return 503, {'success': False, 'error': 'teleop node is not running'}

    def _raw_body(self):
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            return None
        if length < 0 or length > 8192:
            return None
        if length == 0:
            return b''
        return self.rfile.read(length)

    def _json(self, status, payload, cookie=None):
        self._bytes(status, 'application/json', json.dumps(payload).encode('utf-8'), cookie)

    def _bytes(self, status, content_type, body, cookie=None):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        if cookie:
            self.send_header(
                'Set-Cookie',
                f'teleop_client={cookie}; Path=/; SameSite=Lax',
            )
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header(
            'Content-Type',
            'multipart/x-mixed-replace; boundary=%s' % BOUNDARY.decode('ascii'),
        )
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        seq = 0
        with self.hub._cond:
            self.hub.clients += 1
        try:
            while True:
                stopped, seq, jpeg = self.hub.wait_next(seq, 2.0)
                if stopped or not jpeg:
                    if stopped:
                        break
                    continue
                chunk = (
                    b'--' + BOUNDARY + b'\r\n'
                    b'Content-Type: image/jpeg\r\n'
                    b'Content-Length: ' + str(len(jpeg)).encode('ascii') + b'\r\n'
                    b'\r\n' + jpeg + b'\r\n'
                )
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        finally:
            with self.hub._cond:
                self.hub.clients -= 1


def main():
    parser = argparse.ArgumentParser(description='Serve the Jetson camera over HTTP.')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--quality', type=int, default=70)
    parser.add_argument('--sensor-id', type=int, default=0)
    parser.add_argument('--teleop-port', type=int, default=8081)
    args = parser.parse_args()

    if args.width % 2 or args.height % 2 or args.width < 16 or args.height < 16:
        raise SystemExit('width and height must be even and at least 16')
    if not 1 <= args.fps <= 60:
        raise SystemExit('fps must be between 1 and 60')
    if not 1 <= args.quality <= 100:
        raise SystemExit('quality must be between 1 and 100')

    Gst.init(None)
    hub = FrameHub()
    Handler.hub = hub
    Handler.page = render_teleop_page()
    Handler.teleop_port = args.teleop_port
    camera = CameraPipeline(
        hub, args.sensor_id, args.width, args.height, args.fps, args.quality)
    camera.start()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    server.daemon_threads = True
    print(
        f'teleop page http://{args.host}:{args.port}/teleop  '
        f'camera {args.width}x{args.height} @ {args.fps} fps',
        flush=True,
    )

    def shutdown(signum, _frame):
        print('stopping', flush=True)
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    try:
        server.serve_forever()
    finally:
        camera.stop()
        server.server_close()


if __name__ == '__main__':
    main()
