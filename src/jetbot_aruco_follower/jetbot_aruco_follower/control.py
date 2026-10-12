"""Pure target-following policy, independent of ROS and camera drivers."""

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class Target:
    """Metric target observation relative to the camera."""

    distance_m: float
    bearing_rad: float


@dataclass(frozen=True)
class Command:
    """Bounded differential-drive command and controller state."""

    linear_mps: float
    angular_radps: float
    state: str


@dataclass(frozen=True)
class FollowConfig:
    """Tunable target-following limits."""

    stop_distance_m: float = 0.5
    max_linear_mps: float = 0.08
    max_angular_radps: float = 0.6
    marker_to_front_offset_m: float = 0.0
    linear_kp: float = 0.45
    angular_kp: float = 1.8
    stop_tolerance_m: float = 0.02
    align_before_forward_rad: float = 0.35
    loss_grace_s: float = 0.8
    search_pause_s: float = 0.5
    search_min_angle_rad: float = 0.5
    search_max_angle_rad: float = 1.5

    def validate(self):
        """Reject unsafe or non-finite controller values."""
        positive = (
            'stop_distance_m', 'max_linear_mps', 'max_angular_radps',
            'linear_kp', 'angular_kp', 'align_before_forward_rad',
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        nonnegative = (
            'marker_to_front_offset_m', 'stop_tolerance_m', 'loss_grace_s',
            'search_pause_s',
        )
        for name in nonnegative:
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be nonnegative and finite')
        if (not math.isfinite(self.search_min_angle_rad)
                or self.search_min_angle_rad <= 0
                or not math.isfinite(self.search_max_angle_rad)
                or self.search_max_angle_rad < self.search_min_angle_rad):
            raise ValueError(
                'search angles must be positive and ordered')


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class FollowController:
    """Stop, turn, approach, and search with a camera-loss fail-safe."""

    def __init__(self, config, rng=None):
        config.validate()
        self.config = config
        self._rng = rng or random.Random()
        self.reset()

    def reset(self):
        """Forget the previous target and any active search motion."""
        self.last_seen_at = None
        self._search_phase = None
        self._search_until = None
        self._search_angular = 0.0

    def command(self, target, now, observed_at=None, camera_ok=True):
        """Return a bounded command for the latest target observation."""
        if not camera_ok:
            return Command(0.0, 0.0, 'CAMERA_STALE')

        if target is not None:
            if (not math.isfinite(target.distance_m)
                    or not math.isfinite(target.bearing_rad)
                    or target.distance_m <= 0):
                target = None

        if target is not None:
            self._search_phase = None
            self.last_seen_at = now if observed_at is None else observed_at
            distance_from_front = max(
                0.0,
                target.distance_m
                - self.config.marker_to_front_offset_m,
            )
            error = distance_from_front - self.config.stop_distance_m
            if error <= self.config.stop_tolerance_m:
                return Command(0.0, 0.0, 'ARRIVED')
            angular = _clamp(
                self.config.angular_kp * target.bearing_rad,
                self.config.max_angular_radps,
            )
            if abs(target.bearing_rad) >= (
                    self.config.align_before_forward_rad):
                return Command(0.0, angular, 'ALIGNING')
            linear = min(
                self.config.max_linear_mps,
                self.config.linear_kp * error,
            )
            return Command(linear, angular, 'APPROACHING')

        if (self.last_seen_at is not None
                and now - self.last_seen_at < self.config.loss_grace_s):
            self._search_phase = None
            return Command(0.0, 0.0, 'TARGET_LOST_GRACE')
        return self._search(now)

    def _search(self, now):
        if self._search_phase is None or now >= self._search_until:
            if self._search_phase == 'turn':
                self._search_phase = 'pause'
                self._search_until = now + self.config.search_pause_s
                self._search_angular = 0.0
            else:
                self._start_search_turn(now)
        if self._search_phase == 'pause':
            return Command(0.0, 0.0, 'SEARCH_PAUSE')
        return Command(0.0, self._search_angular, 'SEARCHING')

    def _start_search_turn(self, now):
        magnitude = self._rng.uniform(
            self.config.search_min_angle_rad,
            self.config.search_max_angle_rad,
        )
        sign = 1.0 if self._rng.random() < 0.5 else -1.0
        speed = self.config.max_angular_radps
        self._search_phase = 'turn'
        self._search_angular = sign * speed
        self._search_until = now + magnitude / speed
