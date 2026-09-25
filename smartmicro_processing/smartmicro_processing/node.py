# SPDX-License-Identifier: Apache-2.0
"""Passive experimental Doppler node; the existing estimator retains all TF ownership."""
import copy
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
from visualization_msgs.msg import Marker, MarkerArray

from .cloud import empty_cloud, GateConfig, measurements, select_measurements, subset_cloud
from .doppler import fit_velocity, FitConfig
from .ghosts import ghost_mask, GhostConfig
from .obstacles import obstacle_points, ObstacleConfig, PersistenceFilter
from .ros_support import declare, DiagnosticsRateLimiter
from .tracker import MovingObjectTracker, TrackerConfig

# Relative by default so a namespace moves the input with the node; remap it in launch.
DEFAULT_INPUT = 'smart_radar/port_targets_0'
CLOUD_OUTPUTS = ('quality_targets', 'doppler_inliers', 'doppler_outliers', 'unclassified_targets',
                 'moving_targets', 'moving_ghosts')
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
    'range_gap': ('A mover this much farther than a nearer same-speed mover or a static '
                  'return at its bearing is a multipath ghost (m).', .1, 20),
    'speed_tolerance': ('Compensated-speed match for the same-speed ghost rule (m/s).', .01, 5),
    'wall_azimuth_deg': ('Bearing match for the behind-static-return ghost rule (deg).', .1, 30),
}
TRACKER_PARAMETERS = {
    'cluster_radius': ('Moving targets closer than this form one measurement (m).', .05, 10),
    'gate': ('Association distance from a predicted track position (m).', .05, 20),
    'confirm_hits': ('Hits within confirm_window scans needed to confirm a track.', 1, 64, 1),
    'confirm_window': ('Scan window for track confirmation.', 1, 64, 1),
    'max_coast': ('Delete a track after this long without support (s).', .05, 30),
    'static_hold': ('A confirmed track may live on novel zero-Doppler support this long (s).',
                    .01, 600),
    'background_time_constant': ('Memory of the static background used to tell novel '
                                 'zero-Doppler support from walls (s). Valid only while '
                                 'the input frame is fixed in the scene.', 1, 3600),
}
OBSTACLE_PARAMETERS = {
    'persistence_hits': ('Static returns pass when their polar cell was hit in this many of '
                         'the last persistence_window scans.', 1, 64, 1),
    'persistence_window': ('Scan window for static persistence.', 1, 64, 1),
    'safety_range': ('Non-ghost returns nearer than this pass immediately (m).', 0.01, 50),
    'track_radius': ('Moving returns this close to a confirmed track pass (m).', .05, 10),
    'obstacle_height': ('Output z in the sensor frame (m); negative keeps the measured z, '
                        'which is unreliable on this sensor.', -1, 10),
    'shadow_gap': ('Novel static returns this far beyond the nearest confirmed track are '
                   'treated as its multipath (m); <= 0 disables.', -1, 50),
}
OBSTACLE_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                   for i, n in enumerate('xyz')]
TRACK_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
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
        self.max_age = declare(self, 'max_input_age', .5,
                               'Reject input stamps older than this ROS-clock age (s).', .05, 10)
        self.future_tolerance = declare(
            self, 'future_tolerance', .05,
            'Reject input stamps further than this ahead of the ROS clock (s).', 0, 1)
        self.stale_timeout = declare(
            self, 'stale_timeout', .5,
            'Wall-clock time without input before outputs clear and input is stale (s).', .1, 10)
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
        self.tracker_config = _declare_config(self, TrackerConfig, TRACKER_PARAMETERS)
        self.tracker = MovingObjectTracker(self.tracker_config)
        self.track_pub = self.create_publisher(PointCloud2, '~/tracked_objects',
                                               qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(MarkerArray, '~/track_markers', 10)
        self.obstacle_config = _declare_config(self, ObstacleConfig, OBSTACLE_PARAMETERS)
        self.persistence = PersistenceFilter(self.obstacle_config)
        self.obstacle_pub = self.create_publisher(PointCloud2, '~/obstacles',
                                                  qos_profile_sensor_data)
        self.cloud_publishers = {
            name: self.create_publisher(PointCloud2, '~/' + name, qos_profile_sensor_data)
            for name in CLOUD_OUTPUTS}
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped,
                                                  '~/experimental_velocity', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.diagnostics_limiter = DiagnosticsRateLimiter(diagnostics_period)
        topic = declare(self, 'input_topic', DEFAULT_INPUT,
                        'Input PointCloud2. Prefer remapping the relative default name; '
                        'kept for compatibility with existing parameter files.')
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.receive, qos_profile_sensor_data)
        self.last_stamp = None
        self.last_now = None
        self.last_receipt_wall = None
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
            self.last_stamp = None
            self.last_fit_wall = None
            self.last_receipt_wall = None
            self.state = 'clock_reset'
            self.stats = {}
            self.clear_clouds()
        self.last_now = now
        return now

    def clear_clouds(self):
        """
        Publish one empty cloud per output after they carried data.

        The stamp is the last accepted input stamp, never a newer ``now()``, so
        downstream monotonic-stamp checks keep accepting the next real scan.
        """
        if not self.outputs_hold_data:
            return
        self.outputs_hold_data = False
        stamp = (Time(nanoseconds=self.last_stamp).to_msg() if self.last_stamp is not None
                 else self.get_clock().now().to_msg())
        message = empty_cloud(Header(stamp=stamp, frame_id=self.frame))
        for publisher in self.cloud_publishers.values():
            publisher.publish(message)
        self.track_pub.publish(message)
        self.obstacle_pub.publish(message)
        self.persistence.reset()
        self.tracker = MovingObjectTracker(self.tracker_config)
        self.marker_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def reject(self, reason, detail=''):
        self.rejected_inputs += 1
        self.state = reason
        self.stats = {}
        self.clear_clouds()
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
        try:
            values = measurements(cloud)
            indices, stats = select_measurements(values, self.gate_config)
        except (ValueError, TypeError, BufferError) as error:
            self.reject('invalid_cloud', f': {error}')
            return
        self.last_stamp = stamp
        selected = values[indices]
        result = fit_velocity(selected[:, :3], selected[:, 3], self.fit_config)
        self.cloud_publishers['quality_targets'].publish(subset_cloud(cloud, indices))
        if result.valid:
            movers = ~result.inliers
            ghosts = ghost_mask(selected[movers, :3], result.residuals[movers],
                                selected[result.inliers, :3], self.ghost_config)
            moving = indices[movers][~ghosts]
            local = np.flatnonzero(movers)[~ghosts]
            tracks = self.tracker.step(stamp * 1e-9, selected[local, :3],
                                       result.residuals[local], selected[result.inliers, :3])
            self.publish_tracks(cloud.header, tracks, stamp * 1e-9)
            static = selected[result.inliers, :3]
            novel = ~self.tracker.background.is_background(static[:, :2])
            obstacles = obstacle_points(
                static, self.persistence.step(static), selected[movers, :3], ghosts,
                [t.x[:2] for t in tracks], self.obstacle_config, novel)
            self.obstacle_pub.publish(create_cloud(cloud.header, OBSTACLE_FIELDS, obstacles))
            partitions = {'doppler_inliers': indices[result.inliers],
                          'doppler_outliers': indices[movers], 'unclassified_targets': [],
                          'moving_targets': moving,
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
        else:
            # No valid static/moving split: fail conservative for marking and pass every
            # quality target through persistence alone (ghosts cannot be told apart here).
            points = selected[:, :3]
            obstacles = obstacle_points(points, self.persistence.step(points), np.empty((0, 3)),
                                        np.empty(0, bool), [], self.obstacle_config)
            self.obstacle_pub.publish(create_cloud(cloud.header, OBSTACLE_FIELDS, obstacles))
            partitions = {'doppler_inliers': [], 'doppler_outliers': [],
                          'moving_targets': [], 'moving_ghosts': [],
                          'unclassified_targets': indices}
            stats.update(inliers=0, outliers=0, moving=0, ghosts=0, unclassified=len(indices))
        for name, subset in partitions.items():
            self.cloud_publishers[name].publish(subset_cloud(cloud, subset))
        self.outputs_hold_data = True
        self.state = result.reason
        stats.update(stamp_ns=stamp, receive_age_seconds=age,
                     processing_ms=1000 * (time.monotonic() - start))
        self.stats = stats
        self.publish_diagnostics()

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
        if stale_receipt or stale_measurement:
            if self.state != 'input_stale':
                self.state = 'input_stale'
                self.stats = {}
                self.clear_clouds()
        self.publish_diagnostics()

    def publish_diagnostics(self):
        # OK means a numerical fit passed; calibration remains separately false.
        level = DiagnosticStatus.OK if self.state == 'valid' else DiagnosticStatus.WARN
        if self.state in ('input_stale', 'waiting_for_input', 'clock_reset'):
            level = DiagnosticStatus.STALE
        if not self.diagnostics_limiter.due((self.state, level)):
            return
        age = None if self.last_fit_wall is None else time.monotonic() - self.last_fit_wall
        values = dict(state=self.state, fit_valid=self.state == 'valid', calibrated=False,
                      frame=self.frame, input_topic=self.subscription.topic_name,
                      velocity_reference='radar_measurement_origin',
                      time_basis='input_header_receive_time',
                      doppler_sign=self.fit_config.doppler_sign,
                      received=self.received, valid_fits=self.valid_fits,
                      rejected_inputs=self.rejected_inputs, last_velocity_age_seconds=age,
                      **self.stats)
        status = DiagnosticStatus(
            level=level, name=self.get_fully_qualified_name() + '/doppler', hardware_id=self.frame,
            message=('Experimental fit valid; calibration pending' if self.state == 'valid'
                     else self.state),
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
