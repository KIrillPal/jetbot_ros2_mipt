"""Pure geometry and bounded histories. No ROS or robot controls."""
from collections import deque
import math
import time
import uuid


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def diagnostic_level(value):
    # Older generated Humble byte fields use a one-byte bytes object.
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


def alignment(wheel, laser):
    angle = wrap(wheel['yaw'] - laser['yaw'])
    c, s = math.cos(angle), math.sin(angle)
    return {'x': wheel['x'] - c*laser['x'] + s*laser['y'],
            'y': wheel['y'] - s*laser['x'] - c*laser['y'], 'yaw': angle}


def aligned(sample, transform):
    c, s = math.cos(transform['yaw']), math.sin(transform['yaw'])
    return dict(sample, x=transform['x'] + c*sample['x'] - s*sample['y'],
                y=transform['y'] + s*sample['x'] + c*sample['y'],
                yaw=wrap(transform['yaw'] + sample['yaw']))


def paired(history, stamp, tolerance=0.12):
    if not history:
        return None
    result = min(history, key=lambda row: abs(row['stamp_s']-stamp))
    return result if abs(result['stamp_s']-stamp) <= tolerance else None


def diagonal(values):
    result = [0.0]*36
    for i, value in enumerate(values):
        result[7*i] = float(value)
    return result


def valid(sample):
    return all(math.isfinite(sample[key]) for key in
               ('stamp_s', 'x', 'y', 'yaw', 'linear_x', 'linear_y', 'angular_z'))


class History:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.rows = deque(maxlen=6000)
        self.receipts = deque(maxlen=120)
        self.latest = None
        self.received = 0
        self.session = uuid.uuid4().hex
        self.epoch = 1
        self.last_point = -math.inf

    def add(self, sample):
        now = self.clock()
        self.received += 1
        self.latest = dict(sample, seq=self.received)
        self.receipts.append(now)
        if now-self.last_point >= 0.1:
            self.rows.append(dict(self.latest))
            self.last_point = now

    def age(self):
        return None if not self.receipts else max(0.0, self.clock()-self.receipts[-1])

    def state(self, after=0, session=''):
        age = self.age()
        rate = 0.0
        if len(self.receipts) > 1 and self.receipts[-1] > self.receipts[0]:
            rate = (len(self.receipts)-1)/(self.receipts[-1]-self.receipts[0])
        return dict(status='waiting' if age is None else ('live' if age < 1.5 else 'stale'),
                    age_s=age, rate_hz=rate if age is not None and age < 1.5 else 0.0,
                    received=self.received, sequence=self.received, session=self.session,
                    epoch=self.epoch, max_points=6000, latest=self.latest,
                    points=[dict(row) for row in self.rows
                            if session != self.session or row['seq'] > after])
