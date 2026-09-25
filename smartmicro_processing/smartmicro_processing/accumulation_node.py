# SPDX-License-Identifier: Apache-2.0
"""Short-window evidence using per-scan TF, with a separately named still-sensor preview."""
from collections import Counter, deque
from dataclasses import fields
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
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

from .accumulation import AccumulationConfig, TemporalEvidence, pose_discontinuity, transform_measurements
from .cloud import measurements
from .evidence_cloud import evidence_cloud


class RadarAccumulation(Node):
    def __init__(self):
        super().__init__('umrr96_accumulation')

        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.mode = param('mode', 'pose_compensated')
        if self.mode not in ('pose_compensated', 'stationary_preview'):
            raise ValueError('mode must be pose_compensated or stationary_preview')
        self.frame = param('expected_frame_id', 'umrr96')
        self.fixed_frame = param('fixed_frame', 'odom')
        for frame in (self.frame, self.fixed_frame):
            if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
                raise ValueError('Frame names must be nonempty without whitespace/leading slash')
        if self.mode == 'pose_compensated' and self.fixed_frame == self.frame:
            raise ValueError('Pose compensation needs a distinct, genuinely fixed reference frame')
        self.config = AccumulationConfig(**{f.name: param(f.name, f.default)
                                            for f in fields(AccumulationConfig)})
        self.evidence = TemporalEvidence(self.config)
        self.tf_wait = param('tf_wait_seconds', .2)
        self.max_pending = param('max_pending_scans', 32)
        self.max_age = param('max_input_age', .5)
        self.future_tolerance = param('future_tolerance', .05)
        self.stale_timeout = param('stale_timeout', .5)
        self.translation_step = param('max_pose_translation_step', 1.0)
        self.rotation_step = param('max_pose_rotation_step', .5)
        rate = param('publish_hz', 10.0)
        bounded = [(self.tf_wait, .01, 1), (self.max_age, .05, 5),
                   (self.future_tolerance, 0, .2), (self.stale_timeout, .1, 5),
                   (self.translation_step, .01, 100), (self.rotation_step, .01, math.pi),
                   (rate, 1, 50)]
        if any(not math.isfinite(v) or not low <= v <= high for v, low, high in bounded):
            raise ValueError('Invalid timing, pose-step or publish-rate limit')
        if type(self.max_pending) is not int or not 1 <= self.max_pending <= 128:
            raise ValueError('max_pending_scans must be within 1..128')
        self.output_frame = self.fixed_frame if self.mode == 'pose_compensated' else self.frame
        prefix = '~/' if self.mode == 'pose_compensated' else '~/stationary_preview/'
        self.evidence_pub = self.create_publisher(PointCloud2, prefix + 'accumulated_targets',
                                                  qos_profile_sensor_data)
        self.confirmed_pub = self.create_publisher(PointCloud2, prefix + 'confirmed_targets',
                                                   qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.input_topic = param('input_topic', '/umrr96_processing/doppler_inliers')
        self.input_sub = self.create_subscription(PointCloud2, self.input_topic, self.receive,
                                                  qos_profile_sensor_data)
        self.buffer = (Buffer(cache_time=Duration(seconds=self.config.window_seconds + 2))
                       if self.mode == 'pose_compensated' else None)
        self.listener = TransformListener(self.buffer, self) if self.buffer is not None else None
        self.pending = deque()
        self.last_stamp = self.last_clock = self.last_receipt = None
        self.last_pose = self.last_transformed_stamp = None
        self.last_scan_voxels = 0
        self.received = self.processed = self.clock_resets = self.pose_resets = self.manual_resets = 0
        self.dropped = Counter()
        self.state = 'waiting_for_input'
        self.reset_service = self.create_service(Empty, '~/reset', self.reset)
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0/rate, self.publish, clock=self.timer_clock)
        self.get_logger().info(
            f'Radar temporal evidence: {self.mode}, {self.config.window_seconds:g} s window, '
            f'{self.config.voxel_size:g} m cells, frame {self.output_frame}')

    def clear_history(self, reset_input=False, reset_tf=False):
        self.evidence.clear()
        self.pending.clear()
        self.last_pose = self.last_transformed_stamp = None
        self.last_scan_voxels = 0
        if reset_input:
            self.last_stamp = self.last_receipt = None
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

    def reject(self, reason):
        self.dropped[reason] += 1
        self.state = reason

    def receive(self, cloud):
        now = self.now_ns()
        self.received += 1
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if cloud.header.frame_id != self.frame:
            self.reject('unexpected_frame')
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
        indices = np.flatnonzero(np.isfinite(values).all(axis=1)
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
                if self.last_pose is not None and pose_discontinuity(
                        self.last_pose, pose, self.translation_step, self.rotation_step):
                    self.evidence.clear()
                    self.pose_resets += 1
                self.last_pose, self.last_transformed_stamp = pose, stamp
            else:
                transformed = values
            self.pending.popleft()
            if self.evidence.add(transformed, indices, stamp, now):
                self.processed += 1
                self.last_scan_voxels = len(self.evidence.frames[-1][1]) if self.evidence.frames else 0
                self.state = 'accumulating' if self.buffer is not None else 'stationary_preview'

    def publish(self):
        now = self.now_ns()
        if self.last_receipt is not None and time.monotonic() - self.last_receipt > self.stale_timeout:
            self.clear_history()  # Also expires evidence while the bag clock is paused.
            self.state = 'input_stale'
        self.process_pending(now)
        samples = self.evidence.snapshot(now)
        confirmed = [s for s in samples if s['support_scans'] >= self.config.min_support_scans]
        header = Header(frame_id=self.output_frame, stamp=Time(nanoseconds=now).to_msg())
        self.evidence_pub.publish(evidence_cloud(samples, header))
        self.confirmed_pub.publish(evidence_cloud(confirmed, header))
        level = DiagnosticStatus.OK if self.state == 'accumulating' else DiagnosticStatus.WARN
        if self.state in ('input_stale', 'waiting_for_input', 'clock_reset'):
            level = DiagnosticStatus.STALE
        values = dict(state=self.state, mode=self.mode, input_topic=self.input_topic,
                      output_frame=self.output_frame, calibrated=False,
                      motion_compensated=self.buffer is not None and bool(samples),
                      window_seconds=self.config.window_seconds, voxel_size=self.config.voxel_size,
                      received=self.received, processed=self.processed, pending=len(self.pending),
                      retained_scans=len(self.evidence.frames), observations=self.evidence.observations,
                      accumulated_voxels=len(samples), confirmed_voxels=len(confirmed),
                      last_scan_voxels=self.last_scan_voxels,
                      last_transformed_stamp_ns=self.last_transformed_stamp,
                      capacity_drops=self.evidence.capacity_drops, clock_resets=self.clock_resets,
                      pose_resets=self.pose_resets, manual_resets=self.manual_resets,
                      **{'dropped_' + key: value for key, value in self.dropped.items()})
        self.diagnostics_pub.publish(DiagnosticArray(header=header, status=[DiagnosticStatus(
            name=self.get_fully_qualified_name() + '/evidence', hardware_id=self.frame, level=level,
            message=self.state, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]))

    def reset(self, request, response):
        self.manual_resets += 1
        self.clear_history(reset_input=True, reset_tf=True)
        self.state = 'reset'
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
