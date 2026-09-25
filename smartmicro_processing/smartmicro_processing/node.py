# SPDX-License-Identifier: Apache-2.0
"""Passive experimental Doppler node; the existing estimator retains all TF ownership."""
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
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from .cloud import empty_cloud, GateConfig, measurements, select_measurements, subset_cloud
from .doppler import fit_velocity, FitConfig
from .ros_support import declare, DiagnosticsRateLimiter

# Relative by default so a namespace moves the input with the node; remap it in launch.
DEFAULT_INPUT = 'smart_radar/port_targets_0'
CLOUD_OUTPUTS = ('quality_targets', 'doppler_inliers', 'doppler_outliers', 'unclassified_targets')
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
            partitions = {'doppler_inliers': indices[result.inliers],
                          'doppler_outliers': indices[~result.inliers], 'unclassified_targets': []}
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
                         unclassified=0, condition=result.condition, residual_rmse=result.rmse,
                         vx=float(result.velocity[0]), vy=float(result.velocity[1]),
                         vz=float(result.velocity[2]),
                         max_velocity_std=float(np.sqrt(np.max(
                             np.linalg.eigvalsh(result.covariance)))))
        else:
            partitions = {'doppler_inliers': [], 'doppler_outliers': [],
                          'unclassified_targets': indices}
            stats.update(inliers=0, outliers=0, unclassified=len(indices))
        for name, subset in partitions.items():
            self.cloud_publishers[name].publish(subset_cloud(cloud, subset))
        self.outputs_hold_data = True
        self.state = result.reason
        stats.update(stamp_ns=stamp, receive_age_seconds=age,
                     processing_ms=1000 * (time.monotonic() - start))
        self.stats = stats
        self.publish_diagnostics()

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
