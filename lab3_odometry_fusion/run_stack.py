#!/usr/bin/env python3
"""Start only measurement adapters and EKF. Neither subprocess can command motion."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request


def main():
    root = Path(__file__).resolve().parent
    children = []
    stopping = False

    def stop(*_args):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        adapter = subprocess.Popen([sys.executable, '-B', str(root/'fusion_node.py')])
        children.append(adapter)
        ekf = None
        while not stopping:
            if adapter.poll() is not None or (ekf is not None and ekf.poll() is not None):
                raise RuntimeError('A fusion process exited; stopping this fusion stack only')
            try:
                with urllib.request.urlopen('http://127.0.0.1:8766/healthz', timeout=2) as response:
                    state = json.load(response)
            except (urllib.error.URLError, TimeoutError):
                time.sleep(0.5)
                continue
            if state['fault']:
                raise RuntimeError(state['fault'])
            if ekf is None and state['publisher_counts'].get('/odometry/filtered', 0) > 0:
                raise RuntimeError('An /odometry/filtered publisher already exists; refusing a duplicate EKF')
            if ekf is None and state['input_health'] and state['publisher_counts'].get('/odometry/filtered') == 0:
                pose = state['alignment']['wheel_pose']
                initial = [pose['x'], pose['y'], 0., 0., 0., pose['yaw']] + [0.]*9
                binary = root/'ws/install/robot_localization/lib/robot_localization/ekf_node'
                ekf = subprocess.Popen([str(binary), '--ros-args', '-r', '__node:=ekf_filter_node',
                    '--params-file', str(root/'params/ekf.yaml'), '-p', 'initial_state:='+json.dumps(initial)])
                children.append(ekf)
                print('EKF started from aligned wheel pose, output /odometry/filtered, publish_tf=false', flush=True)
            time.sleep(0.5)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.terminate()
        for child in reversed(children):
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    main()
