# SPDX-License-Identifier: Apache-2.0
"""Planar moving-object tracker for radar Doppler movers (sensor or fixed frame)."""
from dataclasses import dataclass, field
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


@dataclass
class TrackerConfig:
    cluster_radius: float = 0.6  # m, movers closer than this form one measurement
    gate: float = 1.2  # m, association distance to the predicted position
    joint_association: bool = False  # global distance matching within each priority tier
    association_uncertainty: bool = False  # ablation: EKF innovation costs need calibration
    association_doppler: bool = True  # only used by joint_association + association_uncertainty
    position_std: float = 0.15  # m, cluster-centroid measurement noise
    radial_speed_std: float = 0.1  # m/s, cluster radial-speed measurement noise
    accel_std: float = 2.0  # m/s^2, white-acceleration process noise
    confirm_hits: int = 8  # hits within the last confirm_window scans to confirm
    confirm_window: int = 10
    evidence_confirmation: bool = False  # experimental early confirmation from consistent hits
    max_coast: float = 1.0  # s without any associated detection before deletion
    static_hold: float = 5.0  # s a confirmed track may live on zero-Doppler support only
    static_gate: float = 0.5  # m, zero-Doppler detections this close support a track
    standing_support: bool = False  # preserve pre-stop background and require current support
    standing_hold: float = 30.0  # absolute cap on static-only support in the experiment
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
                  self.background_warmup, self.standing_hold)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Tracker scales must be finite and positive')
        if not 0 < self.background_threshold < 1:
            raise ValueError('background_threshold must be within (0, 1)')
        if not 1 <= self.confirm_hits <= self.confirm_window <= 64:
            raise ValueError('Need 1 <= confirm_hits <= confirm_window <= 64')
        if not isinstance(self.evidence_confirmation, bool):
            raise ValueError('evidence_confirmation must be a boolean')
        if not isinstance(self.standing_support, bool):
            raise ValueError('standing_support must be a boolean')
        if self.standing_support and self.standing_hold < self.static_hold:
            raise ValueError('standing_hold must cover the legacy static_hold interval')
        if not all(isinstance(v, bool) for v in (
                self.joint_association, self.association_uncertainty, self.association_doppler)):
            raise ValueError('Association switches must be booleans')


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
    radial_speed: float = 0.0  # last associated cluster's ghost-rule speed (see step)
    ghost: bool = False
    confirmation_evidence: list = field(default_factory=list)
    standing_background: object = None  # cells known before a confirmed mover stopped
    standing_anchor: object = None
    standing_history: list = field(default_factory=list)
    standing_verified: bool = False

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

    Measurements are mover-cluster centroids in the input frame (sensor at
    its origin) with their mean radial speed *relative to the sensor*, so the
    state is position and velocity relative to the radar in its own axes; it
    equals ground velocity only while the radar is stationary. Tracks confirm
    after M-of-N hits, coast briefly without detections, and may hold on
    nearby novel zero-Doppler detections for ``static_hold`` seconds
    (tangential motion or standing still) while the sensor is still. A
    confirmed track farther than another one with a similar ghost-rule
    (ego-compensated) radial speed is treated as a multipath copy and hidden.
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

    def measurements(self, mover_xyz, mover_speed, ghost_speed=None):
        """
        Cluster movers into ``(centroid, mean speed, count)`` tuples.

        With ``ghost_speed`` (one value per mover), each tuple gains the
        cluster's mean ghost-rule speed as a fourth element.
        """
        xy = np.asarray(mover_xyz, float).reshape(-1, 3)[:, :2]
        speed = np.asarray(mover_speed, float).reshape(-1)
        labels = cluster(xy, self.config.cluster_radius)
        if not len(xy):
            return []
        clusters = [labels == k for k in range(labels.max() + 1)]
        result = [(xy[c].mean(0), float(speed[c].mean()), int(c.sum())) for c in clusters]
        if ghost_speed is None:
            return result
        ghost = np.asarray(ghost_speed, float).reshape(-1)
        if len(ghost) != len(xy):
            raise ValueError('ghost_speed needs one value per mover')
        return [m + (float(ghost[c].mean()),) for m, c in zip(result, clusters)]

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

    def confirmation_score(self, track, position, radial_speed):
        """Bounded per-scan consistency, not a calibrated probability."""
        c = self.config
        position_error = np.linalg.norm(position - track.x[:2]) / max(
            2 * c.position_std, c.cluster_radius / 2)
        radius = np.linalg.norm(track.x[:2])
        predicted = float(track.x[:2] @ track.x[2:] / radius) if radius > 1e-3 else 0.
        speed_error = (radial_speed - predicted) / max(
            3 * c.radial_speed_std, c.ghost_speed_tolerance)
        return math.exp(-.5 * (position_error ** 2 + speed_error ** 2))

    def ready_to_confirm(self, track):
        """Preserve M-of-N confirmation; allow an earlier, well-supported path."""
        c = self.config
        hits = sum(track.history)
        if hits >= c.confirm_hits:
            return True
        # Six good scans can replace eight ordinary hits. Newborn points get one
        # vote regardless of cluster population; a missed scan subtracts a vote.
        return (c.evidence_confirmation and hits >= 6
                and sum(track.confirmation_evidence) >= 4.5)

    def association_cost(self, track, position, radial_speed):
        """Squared innovation distance using the current EKF uncertainty model."""
        c = self.config
        if np.linalg.norm(position - track.x[:2]) >= c.gate:
            return math.inf
        H = np.zeros((3, 4))
        H[0, 0] = H[1, 1] = 1.
        radius = np.linalg.norm(track.x[:2])
        predicted = 0.
        if radius > 1e-3:
            unit = track.x[:2] / radius
            predicted = float(unit @ track.x[2:])
            H[2, :2] = (track.x[2:] - unit * predicted) / radius
            H[2, 2:] = unit
        delta = np.r_[position - track.x[:2], radial_speed - predicted]
        R = np.diag([c.position_std ** 2] * 2 + [c.radial_speed_std ** 2])
        S = H @ track.P @ H.T + R
        if not c.association_doppler:
            delta, S = delta[:2], S[:2, :2]
        S = (S + S.T) / 2
        try:
            value = float(delta @ np.linalg.solve(S, delta))
        except np.linalg.LinAlgError:
            return math.inf
        return value if math.isfinite(value) and value >= 0 else math.inf

    def associate(self, order, measurements):
        """One-to-one matching with explicit misses and confirmed-first priority."""
        c = self.config
        matches, used = {}, set()
        if not c.joint_association:
            for track in order:
                best, best_distance = None, c.gate
                for i, measurement in enumerate(measurements):
                    distance = np.linalg.norm(measurement[0] - track.x[:2])
                    if i not in used and distance < best_distance:
                        best, best_distance = i, distance
                if best is not None:
                    matches[track.track_id] = best
                    used.add(best)
            return matches
        # Nominal 99% chi-square gates for 3D or 2D innovations. The underlying
        # radar covariance is experimental, so these are not coverage guarantees.
        for confirmed in (True, False):
            tier = [t for t in order if t.confirmed == confirmed]
            available = [i for i in range(len(measurements)) if i not in used]
            if not tier or not available:
                continue
            miss = ((11.345 if c.association_doppler else 9.211)
                    if c.association_uncertainty else len(tier) + 1.)
            cost = np.full((len(tier), len(available) + len(tier)), miss)
            for row, track in enumerate(tier):
                for col, index in enumerate(available):
                    position, speed = measurements[index][:2]
                    if c.association_uncertainty:
                        value = self.association_cost(track, position, speed)
                    else:
                        distance = np.linalg.norm(position - track.x[:2])
                        value = distance / c.gate if distance < c.gate else math.inf
                    cost[row, col] = value if value < miss else miss * (len(tier) + 2)
            rows, cols = linear_sum_assignment(cost)
            for row, col in zip(rows, cols):
                if col < len(available) and cost[row, col] < miss:
                    matches[tier[row].track_id] = available[col]
                    used.add(available[col])
        return matches

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

    def reset_background(self):
        """Forget the static background and every standing track's saved copy of it."""
        self.background.reset()
        for track in self.tracks:
            track.standing_background = track.standing_anchor = None
            track.standing_history = []
            track.standing_verified = False

    def standing_observations(self, stamp, static_xy, sensor_moving):
        """Assign each current static point to at most one eligible anchored track."""
        c = self.config
        if not c.standing_support or sensor_moving:
            return {}
        candidates = [t for t in self.tracks if t.confirmed and not t.ghost
                      and t.standing_background is not None and t.last_moving < stamp
                      and stamp - t.last_moving <= c.standing_hold]
        result = {t.track_id: np.empty((0, 2)) for t in candidates}
        if not candidates or not len(static_xy):
            return result
        keys = self.background.keys(static_xy)
        cost = np.full((len(candidates), len(static_xy)), np.inf)
        for i, track in enumerate(candidates):
            distance = np.linalg.norm(static_xy - track.x[:2], axis=1)
            allowed = np.array([key not in track.standing_background for key in keys])
            allowed &= distance < c.static_gate
            if track.standing_anchor is not None:
                allowed &= (np.linalg.norm(static_xy - track.standing_anchor, axis=1)
                            < c.static_gate)
            cost[i, allowed] = distance[allowed]
        owner = np.argmin(cost, axis=0)
        finite = np.isfinite(cost.min(axis=0))
        for i, track in enumerate(candidates):
            result[track.track_id] = static_xy[finite & (owner == i)]
        return result

    def step(self, stamp, mover_xyz, mover_speed, static_xyz=(), sensor_moving=False,
             ghost_speed=None):
        """
        Advance to ``stamp`` (s); return the confirmed tracks.

        ``mover_speed`` is each mover's radial speed relative to the sensor
        (positive receding): it drives the EKF, confirmation and association
        in the input frame. ``ghost_speed`` (default: ``mover_speed``) is the
        speed the track-level ghost rule compares, e.g. the ego-compensated
        Doppler residual used by ``ghosts.py``; the two agree on a stationary
        sensor. ``static_xyz=None`` means the static returns are unknown: the
        background is neither learned nor used. ``sensor_moving`` resets the
        background, which assumes a scene-fixed input frame, and disables
        zero-Doppler support: on a moving sensor every static return is novel.
        """
        c = self.config
        if self.last_step is not None and stamp - self.last_step > c.max_coast:
            self.tracks = []  # no coasting through a data gap longer than max_coast
        self.last_step = stamp
        for track in self.tracks:
            self.predict(track, stamp)
        if ghost_speed is None:  # existing callers and overrides take two arguments
            measurements = self.measurements(mover_xyz, mover_speed)
            ghost_means = [m[1] for m in measurements]
        else:
            measurements = self.measurements(mover_xyz, mover_speed, ghost_speed)
            ghost_means = [m[3] for m in measurements]
        # Confirmed tracks retain priority in both association modes.
        order = sorted(self.tracks, key=lambda t: (not t.confirmed, t.track_id))
        matches = self.associate(order, measurements)
        used = set()
        for track in order:
            best = matches.get(track.track_id)
            hit = best is not None
            evidence = -1.
            if hit:
                used.add(best)
                if c.evidence_confirmation:
                    evidence = self.confirmation_score(
                        track, measurements[best][0], measurements[best][1])
                self.update(track, measurements[best][0], measurements[best][1])
                track.radial_speed = ghost_means[best]
                track.last_moving = track.last_support = stamp
                if c.standing_support:
                    track.standing_anchor = None
                    track.standing_history = []
                    track.standing_verified = False
            track.history = (track.history + [hit])[-c.confirm_window:]
            if c.evidence_confirmation:
                track.confirmation_evidence = (track.confirmation_evidence + [evidence])[
                    -c.confirm_window:]
            track.hits += hit
        if sensor_moving:
            self.reset_background()
        if static_xyz is None:
            static_xy = np.empty((0, 2))
            all_static_xy = static_xy
            self.static_novel = None
        else:
            static_xy = np.asarray(static_xyz, float).reshape(-1, 3)[:, :2]
            all_static_xy = static_xy
            # Judge novelty before learning this scan, so a new standing object stays novel.
            # Before warm-up every return is novel for holding tracks, but static_novel
            # is None: callers must not drop returns on an unlearned background.
            ready = self.background.ready
            novel = ~self.background.is_background(static_xy)
            self.static_novel = novel if ready else None
            self.background.update(stamp, static_xy)
            static_xy = static_xy[novel]
        standing = (self.standing_observations(stamp, all_static_xy, sensor_moving)
                    if static_xyz is not None else {})
        for track in self.tracks:
            support = standing.get(track.track_id, static_xy)
            extended = track.track_id in standing
            if (track.confirmed and not track.ghost and track.last_moving < stamp
                    and not sensor_moving):
                distances = np.linalg.norm(support - track.x[:2], axis=1)
                close = distances < c.static_gate
                if extended:
                    track.standing_history = (track.standing_history + [bool(close.any())])[-5:]
                    track.standing_verified |= sum(track.standing_history) >= 3
                recent = (stamp - track.last_moving <= c.static_hold
                          or extended and track.standing_verified)
                if close.any() and recent:
                    position = support[close].mean(0)
                    if extended and track.standing_anchor is None:
                        track.standing_anchor = position.copy()
                    self.update_position(track, position)
                    track.last_support = stamp
            if not track.confirmed and self.ready_to_confirm(track):
                track.confirmed = True
            if (c.standing_support and track.confirmed and track.standing_background is None
                    and track.last_moving == stamp and self.background.ready
                    and not sensor_moving):
                threshold = c.background_threshold * self.background.weight
                track.standing_background = frozenset(
                    key for key, weight in self.background.occupancy.items()
                    if weight >= threshold)
        for track in self.tracks:
            track.ghost = track.confirmed and self.is_ghost(track)
        for i, measurement in enumerate(measurements):
            if i in used:
                continue
            position, speed = measurement[:2]
            r = max(np.linalg.norm(position), 1e-3)
            velocity = position / r * speed  # radial component only until more hits
            self.tracks.append(Track(
                self.next_id, np.r_[position, velocity],
                np.diag([c.position_std ** 2] * 2 + [1.0, 1.0]), stamp, stamp, stamp, [True],
                hits=1, radial_speed=ghost_means[i], first_stamp=stamp,
                confirmation_evidence=[1.] if c.evidence_confirmation else []))
            self.next_id += 1
        self.tracks = [t for t in self.tracks if stamp - t.last_support <= c.max_coast
                       and (t.confirmed or len(t.history) < c.confirm_window
                            or sum(t.history) >= c.confirm_hits - 1)]
        return [t for t in self.tracks if t.confirmed and not t.ghost]

    def is_ghost(self, track):
        """Farther than another confirmed track with a similar ghost-rule radial speed."""
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
