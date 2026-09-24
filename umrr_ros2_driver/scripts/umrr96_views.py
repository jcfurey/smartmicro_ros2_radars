#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Detection density in a selected TF frame and sensor-local polar fan displays."""

import copy
from collections import deque
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import transform_points

from umrr96_fan_image import FanImage
from umrr96_filter import DetectionFilter


class DensityGrid:
    """Count hits in Cartesian cells with exponential decay, never ray clearing."""

    def __init__(self, resolution=.25, decay_seconds=2.0):
        if not all(math.isfinite(v) and v > 0 for v in (resolution, decay_seconds)):
            raise ValueError('Resolution and decay time must be finite and positive')
        self.resolution = resolution
        self.decay_seconds = decay_seconds
        self.cells = {}
        self.time = None

    def clear(self):
        self.cells.clear()
        self.time = None

    def decay(self, now):
        if not math.isfinite(now):
            raise ValueError('Non-finite clock value')
        if self.time is not None:
            if now < self.time:
                self.clear()
            else:
                factor = math.exp(-(now - self.time) / self.decay_seconds)
                self.cells = {key: weight * factor for key, weight in self.cells.items()
                              if weight * factor >= .25}
        self.time = now

    def add(self, points, now, weight=1.0):
        self.decay(now)
        for x, y in points:
            if math.isfinite(x) and math.isfinite(y):
                cell = (math.floor(x / self.resolution), math.floor(y / self.resolution))
                self.cells[cell] = self.cells.get(cell, 0.0) + weight

    def set_decay(self, seconds, now):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('Decay time must be finite and positive')
        self.decay(now)  # Apply the old rate up to the change, without clearing hits.
        self.decay_seconds = seconds

    def samples(self):
        return [((i + .5) * self.resolution, (j + .5) * self.resolution, 0.0, hits)
                for (i, j), hits in sorted(self.cells.items())]


def fields(channel):
    return [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, name in enumerate(('x', 'y', 'z', channel))]


def density_color(hits, full_scale):
    fraction = min(1.0, max(0.0, hits / full_scale))
    if fraction < .5:
        t = 2 * fraction
        rgb = (.1 * (1 - t), .35 + .55 * t, .95 - .15 * t)
    else:
        t = 2 * fraction - 1
        rgb = (t, .9 + .05 * t, .8 - .7 * t)
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=.15 + .85 * math.sqrt(fraction))


class RadarViews(Node):
    """Publish RViz markers and flat clouds without accessing sensor controls."""

    def __init__(self):
        super().__init__('umrr96_views')

        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        self.frame = param('frame_id', 'umrr96')
        self.grid_frame = param('grid_frame_id', '') or self.frame
        self.tf_wait = param('tf_wait_seconds', .25)
        if not math.isfinite(self.tf_wait) or not 0 <= self.tf_wait <= 2:
            raise ValueError('tf_wait_seconds must be within 0..2 seconds')
        self.tf_buffer = Buffer(node=self) if self.grid_frame != self.frame else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.tf_buffer else None
        self.pending_grid = deque()
        self.tf_dropped = 0
        self.grid_clock = None
        self.range = param('max_range', 20.0)
        angle = param('half_angle_degrees', 90.0)
        resolution = param('cell_size', .25)
        decay = self.declare_parameter('decay_seconds', 2.0, ParameterDescriptor(
            description='Density exponential decay time in seconds; adjustable at runtime.',
            floating_point_range=[FloatingPointRange(from_value=.1, to_value=30.0)])).value
        self.full_scale = param('density_full_scale', 20.0)
        rate = param('publish_hz', 10.0)
        topic = param('input_topic', '/smart_radar/port_targets_0')
        if not all(math.isfinite(v) for v in (
                self.range, angle, resolution, decay, self.full_scale, rate)):
            raise ValueError('Display parameters must be finite')
        if not (0 < self.range <= 200 and 0 < angle <= 180 and .05 <= resolution <= 5
                and .1 <= decay <= 30 and 1 <= self.full_scale <= 10000 and 1 <= rate <= 50):
            raise ValueError('Display parameter out of range')
        if (2 * self.range / resolution) ** 2 > 250000:
            raise ValueError('Grid exceeds 250000 cells; increase cell_size')
        self.angle = math.radians(angle)
        self.image_renderer = FanImage(
            self.range, self.angle, resolution, param('image_angle_bin_degrees', 2.0),
            param('image_width', 960), param('image_height', 640))
        self.grid = DensityGrid(resolution, decay)
        self.last_input = None
        self.image_targets = []
        self.input_header = None
        self.fan_stale = False
        self.filter = DetectionFilter(
            self.declare_parameter('filter_mode', 'off').value,
            self.declare_parameter('filter_min_snr_db', 6.0).value,
            self.declare_parameter('filter_min_abs_speed', .25).value)
        self.add_on_set_parameters_callback(self.validate_filter)
        self.add_post_set_parameters_callback(self.update_filter)
        self.grid_pub = self.create_publisher(Marker, '/smart_radar/density_grid', 1)
        self.cells_pub = self.create_publisher(PointCloud2, '/smart_radar/density_cells', 1)
        self.fan_pub = self.create_publisher(PointCloud2, '/smart_radar/fan_targets', 1)
        self.image_pub = self.create_publisher(Image, '/smart_radar/fan_image', 1)
        self.filtered_pub = self.create_publisher(
            PointCloud2, '/smart_radar/filtered_targets_0', qos_profile_sensor_data)
        self.filter_status_pub = self.create_publisher(
            String, '/smart_radar/filter_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.guides_pub = self.create_publisher(
            MarkerArray, '/smart_radar/fan_guides',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.receive, qos_profile_sensor_data)
        self.reset_service = self.create_service(Empty, '/smart_radar/reset_density', self.reset)
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.guides_pub.publish(self.guides())
        self.publish_filter_status([])
        self.get_logger().info(
            f'Radar views: {resolution:g} m cells, decay {decay:g} s, range {self.range:g} m')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def header(self, frame=None):
        return Header(stamp=self.get_clock().now().to_msg(), frame_id=frame or self.frame)

    def filter_options(self, parameters):
        options = {name: self.get_parameter(name).value for name in (
            'filter_mode', 'filter_min_snr_db', 'filter_min_abs_speed')}
        options.update({p.name: p.value for p in parameters if p.name in options})
        return options

    def validate_filter(self, parameters):
        try:
            decay = next((p.value for p in parameters if p.name == 'decay_seconds'),
                         self.get_parameter('decay_seconds').value)
            if not math.isfinite(decay) or not .1 <= decay <= 30:
                raise ValueError('decay_seconds must be finite and within 0.1..30 seconds')
            options = self.filter_options(parameters)
            DetectionFilter(options['filter_mode'], options['filter_min_snr_db'],
                            options['filter_min_abs_speed'])
            return SetParametersResult(successful=True)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

    def update_filter(self, parameters):
        decay = self.get_parameter('decay_seconds').value
        decay_changed = decay != self.grid.decay_seconds
        if decay_changed:
            now = self.now_seconds()
            self.transform_grid(now)
            old_decay = self.grid.decay_seconds
            self.grid.set_decay(decay, now)
            # Pending TF scans also retain decay accrued before the rate change.
            self.pending_grid = deque((header, points, queued, now,
                weight * math.exp(-max(0, now - received) / old_decay))
                for header, points, queued, received, weight in self.pending_grid)
        options = self.filter_options(parameters)
        if (options['filter_mode'], options['filter_min_snr_db'], options['filter_min_abs_speed']) == (
                self.filter.mode, self.filter.min_snr_db, self.filter.min_abs_speed):
            if decay_changed:
                self.publish_view_status()
                self.publish()
            return
        self.filter = DetectionFilter(options['filter_mode'], options['filter_min_snr_db'],
                                      options['filter_min_abs_speed'])
        self.grid.clear()
        self.pending_grid.clear()
        self.last_input, self.input_header = None, None
        self.image_targets = []
        self.fan_pub.publish(point_cloud2.create_cloud(self.header(), fields('snr'), []))
        # Clear the last filtered frame too; full-layout data resumes on the next input.
        if hasattr(self, 'last_cloud'):
            empty = copy.deepcopy(self.last_cloud)
            empty.height, empty.width, empty.row_step, empty.data = 1, 0, 0, []
            self.filtered_pub.publish(empty)
        self.publish_filter_status([])
        self.publish()

    def publish_filter_status(self, points, stamp=0.0):
        selected, self.filter_stats = self.filter.select(points, stamp)
        self.publish_view_status()
        return selected

    def publish_view_status(self):
        stats = dict(self.filter_stats)
        stats.update(grid_frame_id=self.grid_frame, grid_tf_dropped=self.tf_dropped,
                     grid_tf_pending=len(self.pending_grid),
                     grid_timestamp_source='ros_receive_time', ego_motion_compensated=False,
                     density_decay_seconds=self.grid.decay_seconds)
        self.filter_status_pub.publish(String(data=json.dumps(stats)))

    def receive(self, cloud):
        if cloud.header.frame_id != self.frame:
            self.get_logger().warning(
                f'Ignoring frame {cloud.header.frame_id!r}; expected {self.frame!r}',
                throttle_duration_sec=5)
            return
        try:
            if cloud.row_step != cloud.width * cloud.point_step:
                raise ValueError('Expected a packed radar target cloud')
            points = point_cloud2.read_points(cloud)
            stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
            selected = self.publish_filter_status(points, stamp or self.now_seconds())
            filtered = copy.deepcopy(cloud)
            filtered.height, filtered.width = 1, len(selected)
            filtered.row_step = filtered.width * cloud.point_step
            # Preserve every field and its original bytes, including sentinel/flag values.
            filtered.data = b''.join(cloud.data[i * cloud.point_step:(i + 1) * cloud.point_step]
                                     for i in selected)
            self.filtered_pub.publish(filtered)
            self.last_cloud = cloud
            cartesian, fan, image_targets = [], [], []
            for index in selected:
                p = points[index]
                x, y, radius, azimuth, snr = (float(p[name]) for name in (
                    'x', 'y', 'range', 'azimuth_angle', 'snr'))
                if not all(math.isfinite(v) for v in (x, y, radius, azimuth)):
                    continue
                if not (0 <= radius <= self.range and abs(azimuth) <= self.angle):
                    continue
                if math.hypot(x, y) > self.range:
                    continue
                # The grid uses XYZ before projection: pitch/roll matter in a
                # world frame. Legacy planar inputs still work in sensor-local mode.
                z = float(p['z']) if 'z' in points.dtype.names else 0.0
                if math.isfinite(z):
                    cartesian.append((x, y, z))
                snr = snr if math.isfinite(snr) else 0.0
                fan.append((radius * math.cos(azimuth), radius * math.sin(azimuth),
                            0.0, snr))
                image_targets.append((radius, azimuth, snr))
        except (AssertionError, ValueError, IndexError, TypeError) as error:
            self.get_logger().warning(f'Ignoring malformed target cloud: {error}', throttle_duration_sec=5)
            return
        now = self.now_seconds()
        if self.tf_buffer is None:
            self.grid.add(((x, y) for x, y, _ in cartesian), now)
        elif not (cloud.header.stamp.sec or cloud.header.stamp.nanosec):
            # Time(0) means latest TF; never use that silently for accumulation.
            self.tf_dropped += 1
            self.get_logger().warning('Skipping density input with zero timestamp', throttle_duration_sec=5)
        else:
            if len(self.pending_grid) >= 40:
                self.pending_grid.popleft()
                self.tf_dropped += 1
            self.pending_grid.append((cloud.header, cartesian, time.monotonic(), now, 1.0))
            self.transform_grid(now)
        self.last_input = now
        self.input_header = cloud.header
        self.image_targets = image_targets
        self.fan_stale = False
        self.fan_pub.publish(point_cloud2.create_cloud(cloud.header, fields('snr'), fan))

    def transform_grid(self, now):
        if self.grid_clock is not None and now < self.grid_clock:
            self.pending_grid.clear()
            self.grid.clear()
        self.grid_clock = now
        while self.pending_grid:
            header, points, queued, received, weight = self.pending_grid[0]
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.grid_frame, header.frame_id, Time.from_msg(header.stamp))
            except TransformException as error:
                if time.monotonic() - queued < self.tf_wait:
                    break
                self.pending_grid.popleft()
                self.tf_dropped += 1
                self.get_logger().warning(
                    f'Skipping density input without timestamped TF: {error}', throttle_duration_sec=5)
                continue
            self.pending_grid.popleft()
            if points:
                transformed = transform_points(np.asarray(points), transform.transform)
                # Account for TF waiting when decaying the original hit weight.
                weight *= math.exp(-max(0, now - received) / self.grid.decay_seconds)
                self.grid.add(((x, y) for x, y, _ in transformed), now, weight)

    def publish(self):
        now = self.now_seconds()
        self.transform_grid(now)
        self.grid.decay(now)
        samples = self.grid.samples()
        header = self.header()
        grid_header = self.header(self.grid_frame)
        marker = Marker(header=grid_header, ns='detection_density', id=0, type=Marker.CUBE_LIST)
        marker.action = Marker.ADD if samples else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = self.grid.resolution
        marker.scale.z = .025
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        for x, y, _, hits in samples:
            # Keep flat fan points and guide lines visible when overlaid.
            marker.points.append(Point(x=x, y=y, z=-.05))
            marker.colors.append(density_color(hits, self.full_scale))
        self.grid_pub.publish(marker)
        self.cells_pub.publish(point_cloud2.create_cloud(grid_header, fields('density'), samples))
        # Instantaneous fan targets disappear after one second without input.
        if self.last_input is not None and (now < self.last_input or now - self.last_input > 1):
            if not self.fan_stale:
                self.fan_pub.publish(point_cloud2.create_cloud(header, fields('snr'), []))
                self.fan_stale = True
        if self.image_pub.get_subscription_count():
            state = ('WAITING FOR DATA' if self.last_input is None else
                     'NO RECENT SCAN' if self.fan_stale else 'LIVE')
            targets = self.image_targets if state == 'LIVE' else []
            pixels = self.image_renderer.render(targets, state, self.filter.mode)
            self.image_pub.publish(Image(
                header=self.input_header if state == 'LIVE' else header,
                height=pixels.shape[0], width=pixels.shape[1], encoding='rgb8',
                is_bigendian=0, step=pixels.shape[1] * 3, data=pixels.tobytes()))

    def reset(self, request, response):
        self.grid.clear()
        self.pending_grid.clear()
        self.publish()
        return response

    def guides(self):
        header = self.header()
        lines = Marker(header=header, ns='fan_guides', id=0, type=Marker.LINE_LIST)
        lines.pose.orientation.w = 1.0
        lines.scale.x = .025
        lines.color = ColorRGBA(r=.25, g=.5, b=.6, a=.65)
        markers = [lines]

        def label(text, x, y):
            marker = Marker(header=header, ns='fan_guides', id=len(markers),
                            type=Marker.TEXT_VIEW_FACING, text=text)
            marker.pose.orientation.w = 1.0
            marker.pose.position = Point(x=float(x), y=float(y), z=.1)
            marker.scale.z = .35
            marker.color = ColorRGBA(r=.65, g=.8, b=.85, a=1.0)
            markers.append(marker)

        for ring in range(1, 5):
            radius = self.range * ring / 4
            for i in range(120):
                a = -self.angle + 2 * self.angle * i / 120
                b = -self.angle + 2 * self.angle * (i + 1) / 120
                lines.points.extend([
                    Point(x=radius * math.cos(a), y=radius * math.sin(a), z=-.03),
                    Point(x=radius * math.cos(b), y=radius * math.sin(b), z=-.03)])
            label(f'{radius:g} m', radius, .7)
        for i in range(7):
            angle = -self.angle + 2 * self.angle * i / 6
            lines.points.extend([Point(z=-.03), Point(
                x=self.range * math.cos(angle), y=self.range * math.sin(angle), z=-.03)])
            label(f'{math.degrees(angle):+.0f}°', (self.range + 1) * math.cos(angle),
                  (self.range + 1) * math.sin(angle))
        label('RADAR', -.8, 0)
        return MarkerArray(markers=markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RadarViews()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
