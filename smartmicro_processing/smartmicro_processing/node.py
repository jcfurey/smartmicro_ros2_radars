# SPDX-License-Identifier: Apache-2.0
"""Passive experimental Doppler node; the existing estimator retains all TF ownership."""
from dataclasses import fields
import math
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from .cloud import GateConfig, empty_cloud, measurements, select_measurements, subset_cloud
from .doppler import FitConfig, fit_velocity


class RadarProcessing(Node):
    def __init__(self):
        super().__init__('umrr96_processing')

        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.frame = param('expected_frame_id', 'umrr96')
        if not self.frame or self.frame.startswith('/') or any(c.isspace() for c in self.frame):
            raise ValueError('expected_frame_id must be a nonempty TF frame without leading slash')
        self.max_age = param('max_input_age', .5)
        self.future_tolerance = param('future_tolerance', .05)
        self.stale_timeout = param('stale_timeout', .5)
        if (not all(math.isfinite(v) for v in (self.max_age, self.future_tolerance, self.stale_timeout))
                or not .05 <= self.max_age <= 10 or not 0 <= self.future_tolerance <= 1
                or not .1 <= self.stale_timeout <= 10):
            raise ValueError('Invalid freshness limits')
        self.fit_config = FitConfig(**{f.name: param(f.name, f.default) for f in fields(FitConfig)})
        self.gate_config = GateConfig(**{f.name: param(f.name, f.default) for f in fields(GateConfig)})
        self.cloud_publishers = {name: self.create_publisher(PointCloud2, '~/' + name,
                                                            qos_profile_sensor_data)
                                 for name in ('quality_targets', 'doppler_inliers',
                                              'doppler_outliers', 'unclassified_targets')}
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped,
                                                  '~/experimental_velocity', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.subscription = self.create_subscription(
            PointCloud2, param('input_topic', '/smart_radar/port_targets_0'),
            self.receive, qos_profile_sensor_data)
        self.last_stamp = None
        self.last_now = None
        self.last_receipt_wall = None
        self.last_fit_wall = None
        self.state = 'waiting_for_input'
        self.stats = {}
        self.received = self.valid_fits = self.rejected_inputs = 0
        self.watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog_timer = self.create_timer(.1, self.watchdog, clock=self.watchdog_clock)
        self.get_logger().info(
            'Experimental 3D Doppler processing; sign, timing, extrinsics and covariance '
            'need physical validation before fusion.')

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
        message = empty_cloud(Header(stamp=self.get_clock().now().to_msg(), frame_id=self.frame))
        for publisher in self.cloud_publishers.values():
            publisher.publish(message)

    def reject(self, reason):
        self.rejected_inputs += 1
        self.state = reason
        self.stats = {}
        self.clear_clouds()
        self.publish_diagnostics()

    def receive(self, cloud):
        start = time.monotonic()
        now = self.now_ns()
        self.received += 1
        self.last_receipt_wall = start
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if cloud.header.frame_id != self.frame:
            self.reject('unexpected_frame')
            return
        if not 0 <= cloud.header.stamp.nanosec < 1_000_000_000 or stamp <= 0:
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
            indices, stats = select_measurements(values, self.gate_config)
        except (ValueError, TypeError, BufferError) as error:
            self.reject('invalid_cloud')
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self.last_stamp = stamp
        selected = values[indices]
        result = fit_velocity(selected[:, :3], selected[:, 3], self.fit_config)
        self.cloud_publishers['quality_targets'].publish(subset_cloud(cloud, indices))
        if result.valid:
            partitions = dict(doppler_inliers=indices[result.inliers],
                              doppler_outliers=indices[~result.inliers], unclassified_targets=[])
            twist = TwistWithCovarianceStamped(header=cloud.header)
            twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z = (
                float(v) for v in result.velocity)
            covariance = np.zeros((6, 6))
            covariance[:3, :3] = result.covariance
            covariance[3:, 3:] = np.eye(3) * 1e6  # No angular velocity observation.
            twist.twist.covariance = covariance.reshape(-1).tolist()
            self.velocity_pub.publish(twist)
            self.valid_fits += 1
            self.last_fit_wall = time.monotonic()
            stats.update(inliers=int(result.inliers.sum()), outliers=int((~result.inliers).sum()),
                         unclassified=0, condition=result.condition, residual_rmse=result.rmse,
                         vx=float(result.velocity[0]), vy=float(result.velocity[1]),
                         vz=float(result.velocity[2]),
                         max_velocity_std=float(np.sqrt(np.max(np.linalg.eigvalsh(result.covariance)))))
        else:
            partitions = dict(doppler_inliers=[], doppler_outliers=[], unclassified_targets=indices)
            stats.update(inliers=0, outliers=0, unclassified=len(indices))
        for name, subset in partitions.items():
            self.cloud_publishers[name].publish(subset_cloud(cloud, subset))
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
        age = None if self.last_fit_wall is None else time.monotonic() - self.last_fit_wall
        # OK means a numerical fit passed; calibration remains separately false.
        level = DiagnosticStatus.OK if self.state == 'valid' else DiagnosticStatus.WARN
        if self.state in ('input_stale', 'waiting_for_input', 'clock_reset'):
            level = DiagnosticStatus.STALE
        values = dict(state=self.state, fit_valid=self.state == 'valid', calibrated=False,
                      frame=self.frame, velocity_reference='radar_measurement_origin',
                      time_basis='input_header_receive_time', doppler_sign=self.fit_config.doppler_sign,
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
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
