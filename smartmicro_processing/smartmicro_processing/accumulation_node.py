# SPDX-License-Identifier: Apache-2.0
"""
Short-window evidence using per-scan TF, with a separately named still-sensor preview.

``source_point_index`` in the output clouds indexes the point, in row-major
order, within the *accumulation input* cloud carrying ``source_stamp_*``
(default ``umrr96_processing/doppler_inliers``). That cloud is itself a
reordered subset of the driver's target cloud, so the index is **not** a
driver ``port_targets_0`` index; join on the stamp and the inlier cloud (whose
point records keep the driver's original bytes) instead.
"""
from collections import Counter, deque
import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .accumulation import (AccumulationConfig, pose_step, pose_step_limits, TemporalEvidence,
                           transform_measurements)
from .cloud import measurements
from .evidence_cloud import evidence_cloud
from .ros_support import declare, DiagnosticsRateLimiter

# Relative by default so a namespace moves the input with the node; remap it in launch.
DEFAULT_INPUT = 'umrr96_processing/doppler_inliers'
CONFIG_PARAMETERS = {
    'window_seconds': ('Evidence lifetime by ROS timestamp (s).', .1, 5),
    'voxel_size': ('Edge of the cubic cells that each hold one representative (m).', .05, 5),
    'min_support_scans': ('Distinct scans needed for the confirmed cloud.', 1, 256, 1),
    'max_scans': ('Maximum retained scans; oldest are evicted first.', 1, 256, 1),
    'max_observations': ('Maximum retained cell representatives over all scans.',
                         1, 131072, 1),
}


class RadarAccumulation(Node):
    """Accumulate Doppler-inlier scans into bounded, per-cell temporal evidence."""

    def __init__(self):
        super().__init__('umrr96_accumulation')

        self.mode = declare(self, 'mode', 'pose_compensated',
                            'pose_compensated (TF to fixed_frame) or stationary_preview.')
        if self.mode not in ('pose_compensated', 'stationary_preview'):
            raise ValueError('mode must be pose_compensated or stationary_preview')
        self.frame = declare(self, 'expected_frame_id', 'umrr96',
                             'Sensor TF frame required on input clouds.')
        self.fixed_frame = declare(self, 'fixed_frame', 'odom',
                                   'Output frame for pose compensation; must be fixed.')
        for frame in (self.frame, self.fixed_frame):
            if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
                raise ValueError('Frame names must be nonempty without whitespace/leading slash')
        if self.mode == 'pose_compensated' and self.fixed_frame == self.frame:
            raise ValueError('Pose compensation needs a distinct, genuinely fixed reference frame')
        self.config = AccumulationConfig(**{
            name: declare(self, name, AccumulationConfig.__dataclass_fields__[name].default,
                          description, *limits)
            for name, (description, *limits) in CONFIG_PARAMETERS.items()})
        self.evidence = TemporalEvidence(self.config)
        self.tf_wait = declare(self, 'tf_wait_seconds', .2,
                               'Wall time a scan waits for its exact-stamp TF (s).', .01, 1)
        self.max_pending = declare(self, 'max_pending_scans', 32,
                                   'Scans that may wait for TF at once.', 1, 128, 1)
        self.max_age = declare(self, 'max_input_age', .5,
                               'Reject input stamps older than this ROS-clock age (s).', .05, 5)
        self.future_tolerance = declare(
            self, 'future_tolerance', .05,
            'Reject input stamps further than this ahead of the ROS clock (s).', 0, .2)
        self.stale_timeout = declare(
            self, 'stale_timeout', .5,
            'Wall-clock time without a non-empty scan before history clears (s).', .1, 5)
        self.translation_step = declare(
            self, 'max_pose_translation_step', 1.0,
            'Pose jump allowance between consecutive transformed scans, on top of '
            'max_platform_speed * dt (m).', .01, 100)
        self.rotation_step = declare(
            self, 'max_pose_rotation_step', .5,
            'Rotation jump allowance between consecutive transformed scans, on top of '
            'max_platform_angular_speed * dt (rad).', .01, math.pi)
        self.max_speed = declare(
            self, 'max_platform_speed', 15.0,
            'Plausible sensor speed used to scale the translation step with scan spacing (m/s).',
            0, 100)
        self.max_angular_speed = declare(
            self, 'max_platform_angular_speed', 2.0,
            'Plausible sensor turn rate used to scale the rotation step with scan spacing '
            '(rad/s).', 0, 20)
        rate = declare(self, 'publish_hz', 10.0, 'Output cloud rate (Hz).', 1, 50)
        diagnostics_period = declare(
            self, 'diagnostics_period', 1.0,
            'Minimum period between unchanged /diagnostics messages (s); state changes '
            'publish immediately.', .1, 10)
        if type(self.max_pending) is not int or not 1 <= self.max_pending <= 128:
            raise ValueError('max_pending_scans must be within 1..128')
        self.output_frame = self.fixed_frame if self.mode == 'pose_compensated' else self.frame
        prefix = '~/' if self.mode == 'pose_compensated' else '~/stationary_preview/'
        self.evidence_pub = self.create_publisher(PointCloud2, prefix + 'accumulated_targets',
                                                  qos_profile_sensor_data)
        self.confirmed_pub = self.create_publisher(PointCloud2, prefix + 'confirmed_targets',
                                                   qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.diagnostics_limiter = DiagnosticsRateLimiter(diagnostics_period)
        self.empty_heartbeat = {publisher: DiagnosticsRateLimiter(diagnostics_period)
                                for publisher in (self.evidence_pub, self.confirmed_pub)}
        topic = declare(self, 'input_topic', DEFAULT_INPUT,
                        'Input PointCloud2. Prefer remapping the relative default name; '
                        'kept for compatibility with existing parameter files.')
        self.input_sub = self.create_subscription(PointCloud2, topic, self.receive,
                                                  qos_profile_sensor_data)
        self.input_topic = self.input_sub.topic_name
        self.buffer = (Buffer(cache_time=Duration(seconds=self.config.window_seconds + 2))
                       if self.mode == 'pose_compensated' else None)
        self.listener = TransformListener(self.buffer, self) if self.buffer is not None else None
        self.pending = deque()
        self.last_stamp = self.last_clock = self.last_receipt = None
        self.last_pose = self.last_transformed_stamp = None
        self.last_processed_stamp = None
        self.last_scan_voxels = 0
        self.received = self.processed = self.clock_resets = self.pose_resets = 0
        self.manual_resets = self.empty_inputs = 0
        self.published_data = {self.evidence_pub: None, self.confirmed_pub: None}
        self.dropped = Counter()
        self.state = 'waiting_for_input'
        self.reset_service = self.create_service(Empty, '~/reset', self.reset)
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0/rate, self.publish, clock=self.timer_clock)
        self.get_logger().info(
            f'Radar temporal evidence: {self.mode}, {self.config.window_seconds:g} s window, '
            f'{self.config.voxel_size:g} m cells, frame {self.output_frame}, '
            f'input {self.input_topic}')

    def clear_history(self, reset_input=False, reset_tf=False):
        self.evidence.clear()
        self.pending.clear()
        self.last_pose = self.last_transformed_stamp = None
        self.last_scan_voxels = 0
        if reset_input:
            self.last_stamp = self.last_receipt = self.last_processed_stamp = None
        if reset_tf and self.buffer is not None:
            self.buffer.clear()

    def now_ns(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock is not None and now < self.last_clock:
            self.clock_resets += 1
            self.clear_history(reset_input=True, reset_tf=True)
            self.state = 'clock_reset'
        self.last_clock = now
        return now

    def reject(self, reason, detail=''):
        self.dropped[reason] += 1
        self.state = reason
        if reason == 'unexpected_frame':
            self.get_logger().warning(f'Ignoring input{detail}', throttle_duration_sec=5.0)

    def receive(self, cloud):
        now = self.now_ns()
        self.received += 1
        if cloud.width * cloud.height == 0:
            # Empty clouds (upstream clears or scans without inliers) carry no
            # evidence: they neither advance the stamp epoch nor refresh the
            # receipt watchdog, so upstream staleness still clears history.
            self.empty_inputs += 1
            return
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if cloud.header.frame_id != self.frame:
            self.reject('unexpected_frame',
                        f' with frame {cloud.header.frame_id!r}; expected {self.frame!r}')
            return
        if stamp <= 0 or not 0 <= cloud.header.stamp.nanosec < 1_000_000_000:
            self.reject('invalid_stamp')
            return
        age = (now - stamp) * 1e-9
        if age > self.max_age or age < -self.future_tolerance:
            self.reject('stale_stamp' if age > self.max_age else 'future_stamp')
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reject('nonmonotonic_stamp')
            return
        try:
            values = measurements(cloud)
        except (ValueError, TypeError, BufferError):
            self.reject('invalid_cloud')
            return
        indices = np.flatnonzero(
            np.isfinite(values).all(axis=1)
            & (np.max(np.abs(values[:, :3]), axis=1, initial=0) > 1e-6)
            & (np.abs(values[:, :3]) <= np.finfo(np.float32).max).all(axis=1))
        self.dropped['invalid_points'] += len(values) - len(indices)
        self.last_stamp, self.last_receipt = stamp, time.monotonic()
        if not len(indices):
            self.state = 'no_detections'
            return
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.dropped['pending_capacity'] += 1
        self.pending.append((stamp, values[indices], indices, self.last_receipt))
        self.process_pending(now)

    def check_pose_step(self, pose, stamp):
        """Clear history on a pose jump larger than plausible motion since the last scan."""
        if self.last_pose is None:
            return
        dt = (stamp - self.last_transformed_stamp) * 1e-9
        translation_limit, rotation_limit = pose_step_limits(
            self.translation_step, self.rotation_step, dt, self.max_speed,
            self.max_angular_speed)
        distance, angle = pose_step(self.last_pose, pose)
        if distance > translation_limit or angle > rotation_limit:
            self.evidence.clear()
            self.pose_resets += 1
            self.get_logger().warning(
                f'Pose discontinuity: {distance:.3f} m / {angle:.3f} rad in {dt:.3f} s exceeds '
                f'{translation_limit:.3f} m / {rotation_limit:.3f} rad; evidence cleared',
                throttle_duration_sec=5.0)

    def process_pending(self, now):
        while self.pending:
            stamp, values, indices, queued = self.pending[0]
            age = (now - stamp) * 1e-9
            if age >= self.config.window_seconds or age > self.max_age:
                self.pending.popleft()
                self.reject('expired_pending')
                continue
            if time.monotonic() - queued > self.tf_wait:
                self.pending.popleft()
                self.reject('tf_timeout')
                continue
            if self.buffer is not None:
                try:
                    transform = self.buffer.lookup_transform(
                        self.fixed_frame, self.frame,
                        Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
                except TransformException:
                    self.state = 'waiting_for_tf'
                    break
                t, q = transform.transform.translation, transform.transform.rotation
                pose = ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                try:
                    transformed = transform_measurements(values, *pose)
                except ValueError:
                    self.pending.popleft()
                    self.reject('invalid_transform')
                    continue
                self.check_pose_step(pose, stamp)
                self.last_pose, self.last_transformed_stamp = pose, stamp
            else:
                transformed = values
            self.pending.popleft()
            if self.evidence.add(transformed, indices, stamp, now):
                self.processed += 1
                self.last_processed_stamp = stamp
                self.last_scan_voxels = (len(self.evidence.frames[-1][1])
                                         if self.evidence.frames else 0)
                self.state = 'accumulating' if self.buffer is not None else 'stationary_preview'

    def publish_cloud(self, publisher, samples, header):
        """Publish data every cycle; an empty cloud once on the transition, then ~1 Hz."""
        if not publisher.get_subscription_count():
            self.published_data[publisher] = None  # Rebuild state for a new subscriber.
            return
        has_data = bool(len(samples))
        heartbeat = self.empty_heartbeat[publisher]
        if has_data or self.published_data[publisher] is not False:
            if not has_data:
                heartbeat.due(None, force=True)
        elif not heartbeat.due(None):
            return
        publisher.publish(evidence_cloud(samples, header))
        self.published_data[publisher] = has_data

    def output_stamp(self, samples, now):
        """Newest contributing scan; TF lookups at that stamp succeed downstream."""
        if len(samples):
            return int(samples['source_stamp_ns'].max())
        if self.last_processed_stamp is not None:
            return self.last_processed_stamp
        return now

    def publish(self):
        now = self.now_ns()
        if (self.last_receipt is not None
                and time.monotonic() - self.last_receipt > self.stale_timeout):
            self.clear_history()  # Also expires evidence while the bag clock is paused.
            self.state = 'input_stale'
        self.process_pending(now)
        level = DiagnosticStatus.OK if self.state == 'accumulating' else DiagnosticStatus.WARN
        if self.state in ('input_stale', 'waiting_for_input', 'clock_reset'):
            level = DiagnosticStatus.STALE
        subscribed = (self.evidence_pub.get_subscription_count()
                      or self.confirmed_pub.get_subscription_count())
        diagnostics_due = self.diagnostics_limiter.due((self.state, level))
        if not (subscribed or diagnostics_due):
            self.evidence.expire(now)
            return
        samples = self.evidence.snapshot_array(now)
        confirmed = samples[samples['support_scans'] >= self.config.min_support_scans]
        header = Header(frame_id=self.output_frame,
                        stamp=Time(nanoseconds=self.output_stamp(samples, now)).to_msg())
        self.publish_cloud(self.evidence_pub, samples, header)
        self.publish_cloud(self.confirmed_pub, confirmed, header)
        if diagnostics_due:
            self.publish_diagnostics(level, len(samples), len(confirmed), now)

    def publish_diagnostics(self, level, accumulated, confirmed, now):
        values = dict(state=self.state, mode=self.mode, input_topic=self.input_topic,
                      output_frame=self.output_frame, calibrated=False,
                      motion_compensated=self.buffer is not None and bool(accumulated),
                      window_seconds=self.config.window_seconds,
                      voxel_size=self.config.voxel_size,
                      received=self.received, processed=self.processed,
                      pending=len(self.pending), empty_inputs=self.empty_inputs,
                      retained_scans=len(self.evidence.frames),
                      observations=self.evidence.observations,
                      accumulated_voxels=accumulated, confirmed_voxels=confirmed,
                      last_scan_voxels=self.last_scan_voxels,
                      last_transformed_stamp_ns=self.last_transformed_stamp,
                      capacity_drops=self.evidence.capacity_drops,
                      clock_resets=self.clock_resets, pose_resets=self.pose_resets,
                      manual_resets=self.manual_resets,
                      **{'dropped_' + key: value for key, value in self.dropped.items()})
        header = Header(stamp=Time(nanoseconds=now).to_msg())
        self.diagnostics_pub.publish(DiagnosticArray(header=header, status=[DiagnosticStatus(
            name=self.get_fully_qualified_name() + '/evidence', hardware_id=self.frame,
            level=level, message=self.state,
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]))

    def reset(self, request, response):
        self.manual_resets += 1
        self.clear_history(reset_input=True, reset_tf=True)
        self.state = 'reset'
        self.diagnostics_limiter.last_key = None  # Report the reset immediately.
        self.publish()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadarAccumulation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
