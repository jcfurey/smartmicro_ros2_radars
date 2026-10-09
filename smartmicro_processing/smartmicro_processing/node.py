# SPDX-License-Identifier: Apache-2.0
"""Passive experimental Doppler node; the existing estimator retains all TF ownership."""
import copy
import dataclasses
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from umrr_ros2_msgs.msg import DetectionAudit
from visualization_msgs.msg import Marker, MarkerArray

from .classification import classified_cloud, classify, cleared_audit, drawable
from .cloud import (empty_cloud, empty_like, GateConfig, measurements, select_measurements,
                    subset_cloud)
from .doppler import fit_velocity, FitConfig
from .ghosts import ghost_reasons, ghost_rejection_mask, GhostConfig, GhostReason
from .obstacles import near_tracks, obstacle_points, ObstacleConfig, PersistenceFilter, to_frame
from .ros_support import declare, DiagnosticsRateLimiter, output_publisher
from .tracker import MovingObjectTracker, TrackerConfig

# Relative by default so a namespace moves the input with the node; remap it in launch.
DEFAULT_INPUT = 'smart_radar/port_targets_0'
# Subset and display clouds: built and published only while they have subscribers.
# obstacles, tracked_objects, tracks, detection_audit and experimental_velocity are
# published on every accepted scan.
CLOUD_OUTPUTS = ('quality_targets', 'doppler_inliers', 'doppler_outliers', 'unclassified_targets',
                 'moving_targets', 'moving_ghosts', 'tracked_targets', 'classified_targets')
FIT_PARAMETERS = {
    'doppler_sign': ('+1: input Doppler is positive receding; -1: positive approaching.',
                     -1, 1, 2),
    'residual_threshold': ('Radial-speed residual gate for Doppler inliers (m/s).', .001, 10),
    'min_inliers': ('Minimum number of consensus inliers for a valid fit.', 4, 4096, 1),
    'min_inlier_fraction': ('Minimum consensus fraction of quality targets, (0.5, 1].', .5, 1),
    'max_condition': ('Maximum condition number of the bearing matrix (> 1).', 1, 1e6),
    'ransac_trials': ('RANSAC minimal-sample trials per scan.', 1, 1024, 1),
    'noise_floor': ('Radial-speed noise floor for the covariance (m/s).', 1e-4, 10),
    'velocity_std_floor': ('Standard deviation added to every velocity axis (m/s).', 1e-4, 10),
    'max_velocity_std': ('Reject fits whose largest velocity std exceeds this (m/s).', 1e-4, 100),
    'max_speed': ('Reject fits faster than this sensor speed (m/s).', .01, 300),
}
GHOST_PARAMETERS = {
    'range_gap': ('Minimum extra range for speed-copy or behind-static hypotheses (m); '
                  'also the track-level ghost rule.', .1, 20),
    'speed_tolerance': ('Compensated-speed match for the same-speed ghost rule (m/s); '
                        'also the track-level ghost rule.', .01, 5),
    'wall_azimuth_deg': ('Bearing match for the behind-static-return ghost rule (deg).', .1, 30),
}
TRACKER_PARAMETERS = {
    'evidence_confirmation': ('Experimental early confirmation from consistent position '
                              'and signed-Doppler predictions; M-of-N remains the fallback.',),
    'standing_support': ('Experimental anchored support using the background before a '
                         'confirmed mover stopped; requires current static returns.',),
    'standing_hold': ('Absolute maximum static-only age under experimental standing support (s).',
                      .01, 600),
    'joint_association': ('Experimental one-to-one global assignment '
                          'within confirmed and tentative priority tiers.',),
    'association_uncertainty': ('Use EKF innovation costs and gates instead of distance '
                                'in joint assignment; requires covariance validation.',),
    'association_doppler': ('Include signed radial-speed innovation when joint_association '
                            'and association_uncertainty are both enabled.',),
    'cluster_radius': ('Moving targets closer than this form one measurement (m).', .05, 10),
    'gate': ('Association distance from a predicted track position (m).', .05, 20),
    'confirm_hits': ('Hits within confirm_window scans needed to confirm a track.', 1, 64, 1),
    'confirm_window': ('Scan window for track confirmation.', 1, 64, 1),
    'max_coast': ('Delete a track after this long without support (s).', .05, 30),
    'static_hold': ('A confirmed track may live on novel zero-Doppler support this long (s); '
                    'no such support while the sensor is moving.', .01, 600),
    'background_time_constant': ('Memory of the static background used to tell novel '
                                 'zero-Doppler support from walls (s). Valid only while '
                                 'the input frame is fixed in the scene.', 1, 3600),
    'background_threshold': ('Fraction of recent scans a polar cell must be hit in to be '
                             'background; lower keeps walls a person occludes as background.',
                             .01, .99),
    'background_warmup': ('Observation time before the background is used; it restarts '
                          'whenever the sensor is seen moving (s).', .1, 600),
}
# Consecutive valid fits faster than sensor_moving_speed before the sensor counts as moving;
# the stationary captures never exceeded 0.05 m/s in two consecutive scans.
MOVING_SCANS = 3
OBSTACLE_PARAMETERS = {
    'persistence_hits': ('Static returns pass when their polar cell was hit in this many of '
                         'the last persistence_window scans.', 1, 64, 1),
    'persistence_window': ('Scan window for static persistence.', 1, 64, 1),
    'safety_range': ('Non-ghost returns nearer than this pass immediately (m).', 0.01, 50),
    'track_radius': ('Moving returns this close to a confirmed track pass (m).', .05, 10),
    'obstacle_height': ('Output z in obstacle_frame (the input frame when empty) (m); '
                        'negative keeps the measured z, which is unreliable on this sensor.',
                        -1, 10),
    'shadow_gap': ('Novel static returns this far beyond a confirmed track, near its '
                   'bearing (shadow_half_angle_deg), are treated as its multipath (m); '
                   '<= 0 disables.', -1, 50),
    'shadow_half_angle_deg': ('Bearing half-width of the multipath shadow behind a track, '
                              '(0, 180] (deg). 180 restores the original any-bearing rule, '
                              'whose ghost suppression was measured; this default is not.',
                              0, 180),
}
OBSTACLE_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                   for i, n in enumerate('xyz')]
# track_id is UINT32: float32 cannot keep IDs above 2^24 apart (tentative clusters use IDs).
TRACK_FIELDS = [PointField(name=n, offset=4 * i, count=1, datatype=(
    PointField.UINT32 if n == 'track_id' else PointField.FLOAT32))
    for i, n in enumerate(('x', 'y', 'z', 'vx', 'vy', 'speed', 'track_id', 'age'))]
GATE_PARAMETERS = {
    'min_range': ('Minimum XYZ range of a quality target (m).', 0, 300),
    'max_range': ('Maximum XYZ range of a quality target (m).', 0, 300),
    'min_snr_db': ('Minimum SNR of a quality target (dB).', -40, 100),
}


def _declare_config(node, config_type, specs):
    values = {}
    for name, (description, *limits) in specs.items():
        default = config_type.__dataclass_fields__[name].default
        values[name] = declare(node, name, default, description, *limits)
    return config_type(**values)


class RadarProcessing(Node):
    """Gate radar targets, fit sensor-origin velocity and partition by Doppler residual."""

    def __init__(self):
        super().__init__('umrr96_processing')
        self.frame = declare(self, 'expected_frame_id', 'umrr96',
                             'TF frame required on input clouds; others are rejected.')
        if not self.frame or self.frame.startswith('/') or any(c.isspace() for c in self.frame):
            raise ValueError('expected_frame_id must be a nonempty TF frame without leading slash')
        self.hardware_id = declare(
            self, 'hardware_id', '',
            "Diagnostic hardware_id; set it to the driver's <model>@<ip> (e.g. "
            'umrr96_v1_2_2@192.168.11.11) so aggregators group this node with the radar. '
            'Empty uses expected_frame_id.') or self.frame
        self.max_age = declare(self, 'max_input_age', .5,
                               'Reject input stamps older than this ROS-clock age (s).', .05, 10)
        self.future_tolerance = declare(
            self, 'future_tolerance', .05,
            'Reject input stamps further than this ahead of the ROS clock (s).', 0, 1)
        self.stale_timeout = declare(
            self, 'stale_timeout', .5,
            'Wall-clock time without input before outputs clear and input is stale (s). '
            'Also the input-stamp gap after which static persistence (between accepted '
            'scans) and the background (between valid fits) restart.', .1, 10)
        diagnostics_period = declare(
            self, 'diagnostics_period', 1.0,
            'Minimum period between unchanged /diagnostics messages (s); state changes '
            'publish immediately.', .1, 10)
        freshness = (self.max_age, self.future_tolerance, self.stale_timeout)
        if not all(math.isfinite(v) for v in freshness):
            raise ValueError('Invalid freshness limits')
        self.fit_config = _declare_config(self, FitConfig, FIT_PARAMETERS)
        self.gate_config = _declare_config(self, GateConfig, GATE_PARAMETERS)
        self.ghost_config = _declare_config(self, GhostConfig, GHOST_PARAMETERS)
        self.reject_static_only = declare(
            self, 'reject_static_only', True,
            'Reject on a nearer static return alone; false keeps that reason advisory '
            'while preserving the same/double-speed rejection rules.')
        # One set of ghost gates: the track-level rule uses the point rule's values.
        self.tracker_config = dataclasses.replace(
            _declare_config(self, TrackerConfig, TRACKER_PARAMETERS),
            ghost_range_gap=self.ghost_config.range_gap,
            ghost_speed_tolerance=self.ghost_config.speed_tolerance)
        self.tracker = MovingObjectTracker(self.tracker_config)
        self.sensor_moving_speed = declare(
            self, 'sensor_moving_speed', .05,
            'Fitted sensor speed above which (for 3 consecutive scans) the sensor is moving: '
            'the sensor-frame background is reset and not used, and tracks get no '
            'zero-Doppler support (m/s).', .005, 10)
        self.fast_scans = 0
        self.last_fit_stamp = None  # input stamp of the last valid fit (ns)
        self.background_gap_resets = 0
        # Outputs are reliable KEEP_LAST with qos_overrides (output_publisher); the
        # input subscription stays sensor-data QoS.
        self.track_pub = output_publisher(self, PointCloud2, '~/tracked_objects')
        self.marker_pub = output_publisher(self, MarkerArray, '~/track_markers', 10)
        self.obstacle_config = _declare_config(self, ObstacleConfig, OBSTACLE_PARAMETERS)
        self.persistence = PersistenceFilter(self.obstacle_config)
        self.obstacle_pub = output_publisher(self, PointCloud2, '~/obstacles')
        self.obstacle_frame = declare(
            self, 'obstacle_frame', '',
            'Frame of ~/obstacles; empty keeps the input frame. Otherwise a frame rigidly '
            'attached to the radar (e.g. base_link): points use the latest TF (a static '
            'mount is the contract) and z = obstacle_height in that frame. Scans without '
            'that transform publish no obstacles.')
        if self.obstacle_frame and (self.obstacle_frame.startswith('/') or any(
                c.isspace() for c in self.obstacle_frame)):
            raise ValueError('obstacle_frame must be empty or a TF frame without leading slash')
        self.transform_obstacles = self.obstacle_frame not in ('', self.frame)
        self.obstacle_output_frame = self.obstacle_frame or self.frame
        self.tf_buffer = Buffer() if self.transform_obstacles else None
        self.tf_listener = (TransformListener(self.tf_buffer, self)
                            if self.transform_obstacles else None)
        self.obstacle_tf_failures = self.obstacle_tf_changes = 0
        self.obstacle_tf_error = None  # why the last scan's obstacles were withheld
        self.obstacle_mount = None
        self.cloud_publishers = {name: output_publisher(self, PointCloud2, '~/' + name)
                                 for name in CLOUD_OUTPUTS}
        self.subsets_published = set()  # CLOUD_OUTPUTS published since the last clear
        self.audit_pub = output_publisher(self, DetectionAudit, '~/detection_audit')
        self.velocity_pub = output_publisher(self, TwistWithCovarianceStamped,
                                             '~/experimental_velocity', 10)
        self.diagnostics_pub = output_publisher(self, DiagnosticArray, '/diagnostics', 10)
        self.diagnostics_limiter = DiagnosticsRateLimiter(diagnostics_period)
        topic = declare(self, 'input_topic', DEFAULT_INPUT,
                        'Input PointCloud2. Prefer remapping the relative default name; '
                        'kept for compatibility with existing parameter files.')
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.receive, qos_profile_sensor_data)
        self.last_stamp = None
        self.last_input = None  # last accepted cloud: its layout shapes subset clears
        self.last_now = None
        self.last_receipt_wall = None
        self.last_input_rejected = False
        self.last_fit_wall = None
        self.outputs_hold_data = False
        self.state = 'waiting_for_input'
        self.stats = {}
        self.received = self.valid_fits = self.rejected_inputs = 0
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog_timer = self.create_timer(.1, self.watchdog, clock=self.watchdog_clock)
        self.get_logger().info(
            f'Experimental 3D Doppler processing on {self.subscription.topic_name}; sign, '
            'timing, extrinsics and covariance need physical validation before fusion.')

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_now is not None and now < self.last_now:
            self.clear_clouds('clock_reset')
            self.last_stamp = None
            self.last_fit_stamp = None
            self.last_fit_wall = None
            self.last_receipt_wall = None
            self.state = 'clock_reset'
            self.stats = {}
            # Time went backwards (bag loop): tracks, background and persistence restart.
            self.tracker = MovingObjectTracker(self.tracker_config)
            self.persistence.reset()
            self.fast_scans = 0
        self.last_now = now
        return now

    def clear_clouds(self, reason='cleared'):
        """
        Publish one empty cloud per output after they carried data.

        Subset and display clouds are cleared only if they were published
        since the last clear (they are built only with subscribers).

        Each clear keeps its output's own schema: subset clouds the last
        accepted input's layout, ``tracked_objects`` and ``obstacles`` their
        fixed fields. The stamp is the last accepted input stamp, never a newer
        ``now()``, so downstream monotonic-stamp checks keep accepting the next
        real scan.
        Tracker and background state are kept: one rejected scan must not discard
        the learned background, and the tracker drops tracks itself after a gap
        longer than ``max_coast``.
        """
        if not self.outputs_hold_data:
            return
        self.outputs_hold_data = False
        stamp = (Time(nanoseconds=self.last_stamp).to_msg() if self.last_stamp is not None
                 else self.get_clock().now().to_msg())
        header = Header(stamp=stamp, frame_id=self.frame)
        subset = (empty_like(self.last_input, header) if self.last_input is not None
                  else empty_cloud(header))
        audit = cleared_audit(header, reason)
        for name in sorted(self.subsets_published):  # each output that carried data
            self.cloud_publishers[name].publish(
                classified_cloud(header, np.empty((0, 5)), audit)
                if name == 'classified_targets' else subset)
        self.subsets_published.clear()
        self.audit_pub.publish(audit)
        self.track_pub.publish(create_cloud(header, TRACK_FIELDS, []))
        self.obstacle_pub.publish(create_cloud(
            Header(stamp=stamp, frame_id=self.obstacle_output_frame), OBSTACLE_FIELDS, []))
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def reject(self, reason, detail=''):
        self.rejected_inputs += 1
        self.last_input_rejected = True
        self.state = reason
        self.stats = {}
        self.clear_clouds(reason)
        self.get_logger().warning(f'Rejected input ({reason}){detail}', throttle_duration_sec=5.0)
        self.publish_diagnostics()

    def receive(self, cloud):
        start = time.monotonic()
        now = self.now_ns()
        self.received += 1
        self.last_receipt_wall = start
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if cloud.header.frame_id != self.frame:
            self.reject('unexpected_frame',
                        f': frame {cloud.header.frame_id!r}, expected {self.frame!r}')
            return
        if not 0 <= cloud.header.stamp.nanosec < 1_000_000_000 or stamp <= 0:
            self.reject('invalid_stamp')
            return
        age = (now - stamp) * 1e-9
        if age > self.max_age or age < -self.future_tolerance:
            self.reject('stale_stamp' if age > self.max_age else 'future_stamp',
                        f': age {age:.3f} s')
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reject('nonmonotonic_stamp')
            return
        if self.last_stamp is not None and (stamp - self.last_stamp) * 1e-9 > self.stale_timeout:
            self.persistence.reset()  # k-of-n counts scans; do not span a data gap
        try:
            values = measurements(cloud)
            indices, stats = select_measurements(values, self.gate_config)
        except (ValueError, TypeError, BufferError) as error:
            self.reject('invalid_cloud', f': {error}')
            return
        self.last_stamp = stamp
        self.last_input = cloud
        self.last_input_rejected = False
        selected = values[indices]
        result = fit_velocity(selected[:, :3], selected[:, 3], self.fit_config)
        self.publish_subset('quality_targets', lambda: subset_cloud(cloud, indices))
        if result.valid:
            movers = ~result.inliers
            mover_reasons = ghost_reasons(selected[movers, :3], result.residuals[movers],
                                          selected[result.inliers, :3], self.ghost_config)
            ghosts = ghost_rejection_mask(mover_reasons,
                                          reject_static_only=self.reject_static_only)
            moving = indices[movers][~ghosts]
            local = np.flatnonzero(movers)[~ghosts]
            speed = float(np.linalg.norm(result.velocity))
            self.fast_scans = self.fast_scans + 1 if speed > self.sensor_moving_speed else 0
            if (self.last_fit_stamp is not None
                    and (stamp - self.last_fit_stamp) * 1e-9 > self.stale_timeout):
                # Sensor motion is unknown across a data gap or a run of failed fits:
                # the old scene may no longer be background, so warm-up restarts.
                self.tracker.reset_background()
                self.background_gap_resets += 1
            self.last_fit_stamp = stamp
            static = selected[result.inliers, :3]
            # The EKF runs in the sensor frame, where positions move at the velocity
            # relative to the radar: feed it the sign-adapted measured Doppler. The
            # ego-compensated residual is kept for the ghost rule, as in ghosts.py.
            relative = self.fit_config.doppler_sign * selected[local, 3]
            tracks = self.tracker.step(stamp * 1e-9, selected[local, :3], relative, static,
                                       sensor_moving=self.sensor_moving,
                                       ghost_speed=result.residuals[local])
            self.publish_tracks(cloud.header, tracks, stamp * 1e-9)
            # None until the background is learned: never drop returns as track multipath
            # against an unlearned (or, on a moving sensor, meaningless) background.
            obstacles = obstacle_points(
                static, self.persistence.step(static), selected[movers, :3], ghosts,
                [t.x[:2] for t in tracks], self.obstacle_config, self.tracker.static_novel,
                flatten=not self.transform_obstacles)
            self.publish_obstacles(cloud.header, obstacles)
            track_xy = [t.x[:2] for t in tracks]
            on_track = near_tracks(selected[local, :3], track_xy,
                                   self.obstacle_config.track_radius)
            partitions = {'doppler_inliers': indices[result.inliers],
                          'doppler_outliers': indices[movers], 'unclassified_targets': [],
                          'moving_targets': moving,
                          'tracked_targets': moving[on_track],
                          'moving_ghosts': indices[movers][ghosts]}
            twist = TwistWithCovarianceStamped(header=cloud.header)
            linear = twist.twist.twist.linear
            linear.x, linear.y, linear.z = (float(v) for v in result.velocity)
            covariance = np.zeros((6, 6))
            covariance[:3, :3] = result.covariance
            covariance[3:, 3:] = np.eye(3) * 1e6  # No angular velocity observation.
            twist.twist.covariance = covariance.reshape(-1).tolist()
            self.velocity_pub.publish(twist)
            self.valid_fits += 1
            self.last_fit_wall = time.monotonic()
            stats.update(inliers=int(result.inliers.sum()),
                         outliers=int((~result.inliers).sum()),
                         moving=int((~ghosts).sum()), ghosts=int(ghosts.sum()),
                         tracks=len(tracks), obstacles=len(obstacles),
                         unclassified=0, condition=result.condition, residual_rmse=result.rmse,
                         vx=float(result.velocity[0]), vy=float(result.velocity[1]),
                         vz=float(result.velocity[2]),
                         max_velocity_std=float(np.sqrt(np.max(
                             np.linalg.eigvalsh(result.covariance)))))
            for name, flag in (('ghost_same_speed', GhostReason.SAME_SPEED),
                               ('ghost_double_speed', GhostReason.DOUBLE_SPEED),
                               ('ghost_behind_static', GhostReason.BEHIND_STATIC)):
                stats[name] = int(((mover_reasons & flag) != 0).sum())
            stats['static_only_advisory'] = int(((mover_reasons != 0) & ~ghosts).sum())
        else:
            # No valid static/moving split: tracks coast (a recorded miss) and stay marked;
            # fail conservative and pass every quality target through persistence alone
            # (ghosts cannot be told apart here).
            tracks = self.tracker.coast(stamp * 1e-9)
            self.publish_tracks(cloud.header, tracks, stamp * 1e-9)
            points = selected[:, :3]
            obstacles = obstacle_points(points, self.persistence.step(points), np.empty((0, 3)),
                                        np.empty(0, bool), [t.x[:2] for t in tracks],
                                        self.obstacle_config,
                                        flatten=not self.transform_obstacles)
            self.publish_obstacles(cloud.header, obstacles)
            partitions = {'doppler_inliers': [], 'doppler_outliers': [],
                          'moving_targets': [], 'moving_ghosts': [], 'tracked_targets': [],
                          'unclassified_targets': indices}
            stats.update(inliers=0, outliers=0, moving=0, ghosts=0, tracks=len(tracks),
                         obstacles=len(obstacles), unclassified=len(indices),
                         ghost_same_speed=0, ghost_double_speed=0, ghost_behind_static=0,
                         static_only_advisory=0)
            mover_reasons = np.empty(0, dtype=np.uint8)
        for name, subset in partitions.items():
            self.publish_subset(name, lambda subset=subset: subset_cloud(cloud, subset))
        audit = classify(cloud.header, values, self.gate_config, result, indices,
                         mover_reasons, partitions['tracked_targets'],
                         ghost_rejected=ghosts if result.valid else None)
        self.audit_pub.publish(audit)
        self.publish_subset('classified_targets',
                            lambda: classified_cloud(cloud.header, values, audit))
        stats.update(audited=len(audit.source_index), displayed=len(drawable(values)[1]))
        self.outputs_hold_data = True
        self.state = result.reason
        stats.update(obstacles_published=self.obstacle_tf_error is None)
        stats.update(stamp_ns=stamp, receive_age_seconds=age, sensor_moving=self.sensor_moving,
                     reject_static_only=self.reject_static_only,
                     background_ready=self.tracker.background.ready,
                     processing_ms=1000 * (time.monotonic() - start))
        self.stats = stats
        self.publish_diagnostics()

    @property
    def sensor_moving(self):
        return self.fast_scans >= MOVING_SCANS

    def publish_subset(self, name, build):
        """Build and publish a subset/display cloud only while it has subscribers."""
        publisher = self.cloud_publishers[name]
        if not publisher.get_subscription_count():
            return
        publisher.publish(build())
        self.subsets_published.add(name)

    def publish_obstacles(self, header, points):
        """
        Publish obstacle evidence at the input stamp in ``obstacle_frame``.

        The mount is static, so the latest transform is the contract, not a
        fallback. Without it, nothing is published for this scan: Nav2 must
        not receive sensor-frame points labelled with another frame.
        """
        if not self.transform_obstacles:
            self.obstacle_pub.publish(create_cloud(header, OBSTACLE_FIELDS, points))
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.obstacle_frame, self.frame, Time()).transform
            mount = (np.array([transform.translation.x, transform.translation.y,
                               transform.translation.z]),
                     np.array([transform.rotation.x, transform.rotation.y,
                               transform.rotation.z, transform.rotation.w]))
            points = to_frame(points, *mount, self.obstacle_config.obstacle_height)
        except (TransformException, ValueError) as error:
            self.obstacle_tf_failures += 1
            self.obstacle_tf_error = f'{type(error).__name__}: {error}'
            self.get_logger().warning(
                f'Obstacles withheld: no valid transform {self.frame} -> '
                f'{self.obstacle_frame} ({error})', throttle_duration_sec=5.0)
            return
        if self.obstacle_mount is not None and (
                np.linalg.norm(mount[0] - self.obstacle_mount[0]) > 1e-3
                or abs(abs(mount[1] @ self.obstacle_mount[1]) - 1) > 1e-6):
            self.obstacle_tf_changes += 1
            self.get_logger().warning(
                f'Transform {self.frame} -> {self.obstacle_frame} changed; obstacle_frame '
                'must be rigidly attached to the radar', throttle_duration_sec=5.0)
        self.obstacle_mount = mount
        self.obstacle_tf_error = None
        self.obstacle_pub.publish(create_cloud(
            Header(stamp=header.stamp, frame_id=self.obstacle_frame), OBSTACLE_FIELDS, points))

    def publish_tracks(self, header, tracks, now):
        rows = [(t.x[0], t.x[1], 0.0, t.x[2], t.x[3], t.speed, t.track_id, now - t.first_stamp)
                for t in tracks]
        self.track_pub.publish(create_cloud(header, TRACK_FIELDS, rows))
        if not self.marker_pub.get_subscription_count():
            return
        markers = [Marker(action=Marker.DELETEALL)]
        for t in tracks:
            body = Marker(header=header, ns='track', id=t.track_id, type=Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y = float(t.x[0]), float(t.x[1])
            body.pose.position.z = 0.8
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = 0.5
            body.scale.z = 1.6
            body.color.r, body.color.g, body.color.b, body.color.a = 1.0, 0.55, 0.1, 0.6
            label = Marker(header=header, ns='label', id=t.track_id, type=Marker.TEXT_VIEW_FACING)
            label.pose = copy.deepcopy(body.pose)
            label.pose.position.z = 1.9
            label.scale.z = 0.3
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'#{t.track_id} {t.speed:.1f} m/s'
            arrow = Marker(header=header, ns='velocity', id=t.track_id, type=Marker.ARROW)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.08, 0.16, 0.2
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.9, 0.2, 1.0
            start = body.pose.position
            tip = type(start)(x=start.x + float(t.x[2]), y=start.y + float(t.x[3]), z=0.1)
            arrow.points = [type(start)(x=start.x, y=start.y, z=0.1), tip]
            markers += [body, label, arrow]
        self.marker_pub.publish(MarkerArray(markers=markers))

    def watchdog(self):
        now = self.now_ns()
        wall = time.monotonic()
        stale_receipt = (self.last_receipt_wall is None
                         or wall - self.last_receipt_wall > self.stale_timeout)
        stale_measurement = (self.last_stamp is not None
                             and (now - self.last_stamp) * 1e-9 > self.max_age)
        # While scans keep arriving but are rejected, their reason stays the state
        # (outputs were cleared on rejection); alternating states would flood
        # /diagnostics, which publishes every state change at once.
        if stale_receipt or (stale_measurement and not self.last_input_rejected):
            if self.state != 'input_stale':
                self.state = 'input_stale'
                self.stats = {}
                self.clear_clouds('input_stale')
        self.publish_diagnostics()

    def publish_diagnostics(self):
        # OK means a numerical fit passed; calibration remains separately false.
        level = DiagnosticStatus.OK if self.state == 'valid' else DiagnosticStatus.WARN
        if self.state in ('input_stale', 'waiting_for_input', 'clock_reset'):
            level = DiagnosticStatus.STALE
        # The last accepted scan's obstacles were withheld for want of a transform.
        withheld = self.obstacle_tf_error is not None and self.outputs_hold_data
        if withheld:
            level = DiagnosticStatus.WARN
        if not self.diagnostics_limiter.due((self.state, level, withheld)):
            return
        age = None if self.last_fit_wall is None else time.monotonic() - self.last_fit_wall
        values = dict(state=self.state, fit_valid=self.state == 'valid', calibrated=False,
                      frame=self.frame, input_topic=self.subscription.topic_name,
                      velocity_reference='radar_measurement_origin',
                      time_basis='input_header_receive_time',
                      doppler_sign=self.fit_config.doppler_sign,
                      evidence_confirmation=self.tracker.config.evidence_confirmation,
                      standing_support=self.tracker.config.standing_support,
                      joint_association=self.tracker.config.joint_association,
                      association_uncertainty=self.tracker.config.association_uncertainty,
                      association_doppler=self.tracker.config.association_doppler,
                      received=self.received, valid_fits=self.valid_fits,
                      rejected_inputs=self.rejected_inputs, last_velocity_age_seconds=age,
                      background_gap_resets=self.background_gap_resets,
                      obstacle_frame=self.obstacle_output_frame,
                      obstacle_tf_failures=self.obstacle_tf_failures,
                      obstacle_tf_changes=self.obstacle_tf_changes,
                      obstacle_tf_error=self.obstacle_tf_error,
                      **self.stats)
        if withheld:
            message = f'{self.state}; obstacles withheld: no transform to {self.obstacle_frame}'
        elif self.state == 'valid':
            message = 'Experimental fit valid; calibration pending'
        else:
            message = self.state
        status = DiagnosticStatus(
            level=level, name=self.get_fully_qualified_name() + '/doppler',
            hardware_id=self.hardware_id,
            message=message,
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        self.diagnostics_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadarProcessing()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
