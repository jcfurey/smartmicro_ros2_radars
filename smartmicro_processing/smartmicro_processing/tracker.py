# SPDX-License-Identifier: Apache-2.0
"""Planar moving-object tracker for radar Doppler movers (sensor or fixed frame)."""
from dataclasses import dataclass, field
import math

import numpy as np


@dataclass
class TrackerConfig:
    cluster_radius: float = 0.6  # m, movers closer than this form one measurement
    gate: float = 1.2  # m, association distance to the predicted position
    position_std: float = 0.15  # m, cluster-centroid measurement noise
    radial_speed_std: float = 0.1  # m/s, cluster radial-speed measurement noise
    accel_std: float = 2.0  # m/s^2, white-acceleration process noise
    confirm_hits: int = 8  # hits within the last confirm_window scans to confirm
    confirm_window: int = 10
    max_coast: float = 1.0  # s without any associated detection before deletion
    static_hold: float = 5.0  # s a confirmed track may live on zero-Doppler support only
    static_gate: float = 0.5  # m, zero-Doppler detections this close support a track
    ghost_range_gap: float = 1.5  # m, track-level ghost rule (see ghosts.py)
    ghost_speed_tolerance: float = 0.25  # m/s
    background_range_bin: float = 0.5  # m, polar background cell size
    background_azimuth_bin_deg: float = 3.0
    background_time_constant: float = 30.0  # s, occupancy memory of the static background
    background_threshold: float = 0.3  # occupancy fraction above which a cell is background
    background_warmup: float = 3.0  # s of observation before any cell can be background

    def __post_init__(self):
        values = (self.cluster_radius, self.gate, self.position_std, self.radial_speed_std,
                  self.accel_std, self.max_coast, self.static_hold, self.static_gate,
                  self.ghost_range_gap, self.ghost_speed_tolerance, self.background_range_bin,
                  self.background_azimuth_bin_deg, self.background_time_constant,
                  self.background_warmup)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Tracker scales must be finite and positive')
        if not 0 < self.background_threshold < 1:
            raise ValueError('background_threshold must be within (0, 1)')
        if not 1 <= self.confirm_hits <= self.confirm_window <= 64:
            raise ValueError('Need 1 <= confirm_hits <= confirm_window <= 64')


@dataclass
class Track:
    track_id: int
    x: np.ndarray  # [px, py, vx, vy]
    P: np.ndarray
    stamp: float
    last_moving: float
    last_support: float
    history: list = field(default_factory=list)  # recent hit flags
    confirmed: bool = False
    hits: int = 0
    first_stamp: float = 0.0
    radial_speed: float = 0.0  # last associated cluster speed, for the ghost rule
    ghost: bool = False

    @property
    def speed(self):
        return float(np.hypot(self.x[2], self.x[3]))


class BackgroundModel:
    """
    Exponentially forgetting polar occupancy of zero-Doppler detections.

    Occupancy is normalised by the weight accumulated since the last reset
    (a bias-corrected moving average): a cell hit in every scan is background
    once ``background_warmup`` seconds have been observed, instead of after
    ~0.7 time constants (21 s at 30 s). Until then ``ready`` is false and
    nothing is background. Valid only while the input frame is fixed relative
    to the scene (stationary sensor, or detections transformed into a fixed
    frame by the caller); ``reset()`` when the sensor moves.
    """

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.occupancy = {}
        self.weight = 0.0
        self.start = None
        self.stamp = None

    @property
    def ready(self):
        return (self.start is not None and self.weight > 0
                and self.stamp - self.start >= self.config.background_warmup)

    def keys(self, xy):
        c = self.config
        r = np.hypot(xy[:, 0], xy[:, 1])
        az = np.arctan2(xy[:, 1], xy[:, 0])
        return list(zip((r // c.background_range_bin).astype(int),
                        np.floor(az / math.radians(c.background_azimuth_bin_deg)).astype(int)))

    def update(self, stamp, static_xy):
        if self.start is None:
            self.start = stamp
        dt = 0.0 if self.stamp is None else max(0.0, stamp - self.stamp)
        self.stamp = stamp
        keep = math.exp(-dt / self.config.background_time_constant)
        alpha = 1 - keep
        self.weight = self.weight * keep + alpha
        occupied = set(self.keys(static_xy)) if len(static_xy) else set()
        prune = 1e-3 * self.weight
        for key in list(self.occupancy):
            self.occupancy[key] *= keep
            if self.occupancy[key] < prune and key not in occupied:
                del self.occupancy[key]
        for key in occupied:
            self.occupancy[key] = self.occupancy.get(key, 0.0) + alpha

    def is_background(self, xy):
        if not len(xy) or not self.ready:
            return np.zeros(len(xy), bool)
        limit = self.config.background_threshold * self.weight
        return np.array([self.occupancy.get(k, 0.0) >= limit for k in self.keys(xy)])


def cluster(xy, radius):
    """Single-linkage clusters; returns a label per point."""
    n = len(xy)
    labels = -np.ones(n, int)
    if not n:
        return labels
    near = np.linalg.norm(xy[:, None] - xy[None, :], axis=2) <= radius
    label = 0
    for i in range(n):
        if labels[i] >= 0:
            continue
        stack = [i]
        labels[i] = label
        while stack:
            j = stack.pop()
            for k in np.flatnonzero(near[j] & (labels < 0)):
                labels[k] = label
                stack.append(k)
        label += 1
    return labels


class MovingObjectTracker:
    """
    Constant-velocity EKF tracks from ghost-filtered Doppler movers.

    Measurements are mover-cluster centroids with their mean compensated
    radial speed (sensor at the origin of the input frame). Tracks confirm
    after M-of-N hits, coast briefly without detections, and may hold on
    nearby zero-Doppler detections for ``static_hold`` seconds (tangential
    motion or standing still). A tentative track farther than a confirmed one
    with a similar radial speed is treated as a multipath copy and dropped.
    """

    def __init__(self, config=TrackerConfig()):
        self.config = config
        self.tracks = []
        self.next_id = 1
        self.background = BackgroundModel(config)
        # Per static input of the last step: True when outside the learned background;
        # None when the background was not ready (or the static returns were unknown).
        self.static_novel = None
        self.last_step = None

    def measurements(self, mover_xyz, mover_speed):
        xy = np.asarray(mover_xyz, float).reshape(-1, 3)[:, :2]
        speed = np.asarray(mover_speed, float).reshape(-1)
        labels = cluster(xy, self.config.cluster_radius)
        if not len(xy):
            return []
        return [(xy[labels == k].mean(0), float(speed[labels == k].mean()),
                 int((labels == k).sum())) for k in range(labels.max() + 1)]

    def predict(self, track, stamp):
        dt = max(0.0, stamp - track.stamp)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.config.accel_std ** 2
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        track.x = F @ track.x
        track.P = F @ track.P @ F.T + q * G @ G.T
        track.stamp = stamp

    def update(self, track, position, radial_speed):
        px, py, vx, vy = track.x
        r = math.hypot(px, py)
        H = np.zeros((3, 4))
        H[0, 0] = H[1, 1] = 1
        z = np.array([position[0], position[1], radial_speed])
        if r > 1e-3:
            ux, uy = px / r, py / r
            predicted_vr = ux * vx + uy * vy
            H[2] = [(vx - ux * predicted_vr) / r, (vy - uy * predicted_vr) / r, ux, uy]
        else:
            predicted_vr = 0.0
        h = np.array([px, py, predicted_vr])
        R = np.diag([self.config.position_std ** 2] * 2 + [self.config.radial_speed_std ** 2])
        S = H @ track.P @ H.T + R
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ (z - h)
        track.P = (np.eye(4) - K @ H) @ track.P

    def update_position(self, track, position):
        H = np.zeros((2, 4))
        H[0, 0] = H[1, 1] = 1
        R = np.eye(2) * self.config.position_std ** 2
        S = H @ track.P @ H.T + R
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ (np.asarray(position) - track.x[:2])
        track.P = (np.eye(4) - K @ H) @ track.P
        # Zero-Doppler support: the object is at rest or moving tangentially; damp speed.
        track.x[2:] *= 0.8

    def coast(self, stamp):
        """Advance without a static/moving split (failed Doppler fit): record a miss."""
        return self.step(stamp, (), (), None)

    def step(self, stamp, mover_xyz, mover_speed, static_xyz=(), sensor_moving=False):
        """
        Advance to ``stamp`` (s); return the confirmed tracks.

        ``static_xyz=None`` means the static returns are unknown: the background
        is neither learned nor used. ``sensor_moving`` resets the background,
        which assumes a scene-fixed input frame.
        """
        c = self.config
        if self.last_step is not None and stamp - self.last_step > c.max_coast:
            self.tracks = []  # no coasting through a data gap longer than max_coast
        self.last_step = stamp
        for track in self.tracks:
            self.predict(track, stamp)
        measurements = self.measurements(mover_xyz, mover_speed)
        # Greedy nearest-neighbour association, confirmed tracks first.
        order = sorted(self.tracks, key=lambda t: (not t.confirmed, t.track_id))
        used = set()
        for track in order:
            best, best_distance = None, c.gate
            for i, (position, _, _) in enumerate(measurements):
                distance = np.linalg.norm(position - track.x[:2])
                if i not in used and distance < best_distance:
                    best, best_distance = i, distance
            hit = best is not None
            if hit:
                used.add(best)
                self.update(track, measurements[best][0], measurements[best][1])
                track.radial_speed = measurements[best][1]
                track.last_moving = track.last_support = stamp
            track.history = (track.history + [hit])[-c.confirm_window:]
            track.hits += hit
        if sensor_moving:
            self.background.reset()
        if static_xyz is None:
            static_xy = np.empty((0, 2))
            self.static_novel = None
        else:
            static_xy = np.asarray(static_xyz, float).reshape(-1, 3)[:, :2]
            # Judge novelty before learning this scan, so a new standing object stays novel.
            # Before warm-up every return is novel for holding tracks, but static_novel
            # is None: callers must not drop returns on an unlearned background.
            ready = self.background.ready
            novel = ~self.background.is_background(static_xy)
            self.static_novel = novel if ready else None
            self.background.update(stamp, static_xy)
            static_xy = static_xy[novel]
        for track in self.tracks:
            if (track.confirmed and not track.ghost and track.last_moving < stamp
                    and len(static_xy)):
                distances = np.linalg.norm(static_xy - track.x[:2], axis=1)
                close = distances < c.static_gate
                if close.any() and stamp - track.last_moving <= c.static_hold:
                    self.update_position(track, static_xy[close].mean(0))
                    track.last_support = stamp
            if not track.confirmed and sum(track.history) >= c.confirm_hits:
                track.confirmed = True
        for track in self.tracks:
            track.ghost = track.confirmed and self.is_ghost(track)
        for i, (position, speed, _) in enumerate(measurements):
            if i in used:
                continue
            r = max(np.linalg.norm(position), 1e-3)
            velocity = position / r * speed  # radial component only until more hits
            self.tracks.append(Track(
                self.next_id, np.r_[position, velocity],
                np.diag([c.position_std ** 2] * 2 + [1.0, 1.0]), stamp, stamp, stamp, [True],
                hits=1, radial_speed=speed, first_stamp=stamp))
            self.next_id += 1
        self.tracks = [t for t in self.tracks if stamp - t.last_support <= c.max_coast
                       and (t.confirmed or len(t.history) < c.confirm_window
                            or sum(t.history) >= c.confirm_hits - 1)]
        return [t for t in self.tracks if t.confirmed and not t.ghost]

    def is_ghost(self, track):
        """Farther than another confirmed track with a similar measured radial speed."""
        c = self.config
        r = np.linalg.norm(track.x[:2])
        for other in self.tracks:
            if other is track or not other.confirmed:
                continue
            if r - np.linalg.norm(other.x[:2]) <= c.ghost_range_gap:
                continue
            # First-order bounce repeats the speed; second-order roughly doubles it.
            for factor in (1.0, 2.0):
                if abs(abs(track.radial_speed) - factor * abs(other.radial_speed)) < (
                        factor * c.ghost_speed_tolerance):
                    return True
        return False
