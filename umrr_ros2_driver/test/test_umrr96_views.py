#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline density math and synthetic ROS topic checks; no radar required."""

from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import unittest

from geometry_msgs.msg import TransformStamped
import numpy as np
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray

# Test the source tree's package even when an older build is sourced.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from umrr_ros2_driver_py import detection_filter, views  # noqa: E402, I100


class DensityTests(unittest.TestCase):
    """Density grid binning and decay."""

    def test_cell_boundaries_and_hit_count(self):
        grid = views.DensityGrid(.25, 2)
        grid.add([(1.01, -.01), (1.24, -.24), (1.25, 0), (float('nan'), 0)], 0)
        self.assertEqual(grid.samples(), [(1.125, -.125, 0, 2), (1.375, .125, 0, 1)])

    def test_decay_and_stale_removal(self):
        grid = views.DensityGrid(.25, 2)
        grid.add([(1, 0)], 0)
        grid.decay(2)
        self.assertAlmostEqual(grid.samples()[0][3], math.exp(-1))
        grid.decay(3)
        self.assertEqual(grid.samples(), [])

    def test_new_hits_do_not_refresh_old_hits(self):
        grid = views.DensityGrid(.25, 2)
        grid.add([(1, 0)], 0)
        grid.add([(1, 0)], 2)
        self.assertAlmostEqual(grid.samples()[0][3], 1 + math.exp(-1))

    def test_decay_change_preserves_hits_and_elapsed_old_rate(self):
        grid = views.DensityGrid(.25, 2)
        grid.add([(1, 0)], 0)
        grid.set_decay(.5, 1)
        self.assertAlmostEqual(grid.samples()[0][3], math.exp(-.5))
        grid.decay(1.25)
        self.assertAlmostEqual(grid.samples()[0][3], math.exp(-1))
        for invalid in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                grid.set_decay(invalid, 2)
            self.assertEqual(grid.decay_seconds, .5)
            self.assertAlmostEqual(grid.samples()[0][3], math.exp(-1))
        grid.decay(1.5)
        self.assertEqual(grid.samples(), [])

    def test_clock_reversal_clears_old_frame(self):
        grid = views.DensityGrid()
        grid.add([(1, 0)], 5)
        grid.add([(2, 0)], 1)
        self.assertEqual(grid.samples(), [(2.125, .125, 0, 1)])

    def test_invalid_dimensions(self):
        for resolution, decay in ((0, 2), (.25, -1), (float('nan'), 2)):
            with self.assertRaises(ValueError):
                views.DensityGrid(resolution, decay)


class FanImageTests(unittest.TestCase):
    """Fan image projection and binning."""

    def test_forward_left_projection_and_visible_pixels(self):
        renderer = views.FanImage(20, math.pi / 2)
        radius, angle = 8.125, math.radians(23)
        u, v = renderer.pixel(radius, angle)
        self.assertLess(u, renderer.origin[0])
        self.assertLess(v, renderer.origin[1])
        rgb = renderer.render([(radius, angle, 40)])
        self.assertEqual(rgb.shape, (640, 960, 3))
        self.assertEqual(rgb.dtype, np.uint8)
        np.testing.assert_array_equal(rgb[v, u], renderer.palette[204])
        right_u, right_v = renderer.pixel(radius, -angle)
        np.testing.assert_array_equal(rgb[right_v, right_u], renderer.background[right_v, right_u])
        cleared = renderer.render([], 'NO RECENT SCAN')
        np.testing.assert_array_equal(cleared[v, u], renderer.background[v, u])

    def test_strongest_snr_boundaries_and_invalid_returns(self):
        renderer = views.FanImage(20, math.pi / 2)
        bins = renderer.bin_targets([
            (8.1, .4, 12), (8.2, .4, 40), (20, math.pi / 2, 25),
            (20, -math.pi / 2, -5), (21, 0, 30), (-1, 0, 30),
            (5, math.pi, 30), (float('nan'), 0, 30), (5, 0, float('nan'))])
        self.assertEqual(np.count_nonzero(np.isfinite(bins)), 3)
        self.assertEqual(float(bins.max()), 40)
        self.assertEqual(float(bins[-1, -1]), 25)
        self.assertEqual(float(bins[-1, 0]), -5)

    def test_rear_quadrants_and_image_limits(self):
        renderer = views.FanImage(20, math.pi)
        u, v = renderer.pixel(10, math.radians(140))
        self.assertGreater(v, renderer.origin[1])
        self.assertTrue(0 <= u < renderer.width and 0 <= v < renderer.height)
        for kwargs in ({'width': 0}, {'height': 2000}, {'angle_bin_degrees': 0},
                       {'angle_bin_degrees': float('nan')}):
            with self.assertRaises(ValueError):
                views.FanImage(20, math.pi / 2, **kwargs)


class FilterTests(unittest.TestCase):
    """Detection filter modes."""

    @staticmethod
    def point(radius=3.1, angle=.2, snr=20, speed=0):
        return {'x': radius * math.cos(angle), 'y': radius * math.sin(angle), 'z': 0,
                'range': radius, 'azimuth_angle': angle, 'snr': snr, 'radial_speed': speed}

    def test_off_preserves_every_point(self):
        points = [self.point(snr=-50), self.point(radius=float('nan'))]
        selected, stats = views.DetectionFilter().select(points, 1)
        self.assertEqual(selected, [0, 1])
        self.assertEqual(stats['accepted'], 2)

    def test_quality_mode_has_no_stationary_radar_or_speed_assumption(self):
        filtering = views.DetectionFilter('quality', 6, .25)
        selected, stats = filtering.select([
            self.point(speed=0), self.point(radius=9, speed=-5), self.point(snr=5)], 1)
        self.assertEqual(selected, [0, 1])
        self.assertEqual(stats['rejected_quality'], 1)
        self.assertEqual(stats['rejected_temporal'], 0)
        self.assertEqual(stats['rejected_motion'], 0)

    def test_mapping_requires_distinct_scans_and_preserves_stationary_returns(self):
        filtering = views.DetectionFilter('mapping')
        points = [self.point(), self.point(radius=3.2)]
        self.assertEqual(filtering.select(points, 1)[0], [])
        self.assertEqual(filtering.select(points, 1.1)[0], [0, 1])
        self.assertEqual(filtering.select([], 1.2)[0], [])
        self.assertEqual(filtering.select(points, 1.3)[0], [0, 1])
        self.assertEqual(filtering.select([], 1.4)[0], [])
        self.assertEqual(filtering.select([], 1.5)[0], [])
        self.assertEqual(filtering.select(points, 1.6)[0], [])

    def test_mapping_rejects_unrelated_and_stale_history(self):
        filtering = views.DetectionFilter('mapping')
        filtering.select([self.point()], 1)
        self.assertEqual(filtering.select([self.point(radius=9)], 1.1)[0], [])
        self.assertEqual(filtering.select([self.point(radius=9)], 2)[0], [])
        self.assertEqual(filtering.select([self.point(radius=9)], 2)[0], [])
        self.assertEqual(filtering.select([self.point(radius=9)], 1.9)[0], [])

    def test_moving_keeps_both_directions_and_counts_rejections(self):
        filtering = views.DetectionFilter('moving', 6, .25)
        selected, stats = filtering.select([
            self.point(speed=0), self.point(speed=.25), self.point(speed=-2),
            self.point(snr=5, speed=1), self.point(radius=float('nan'), speed=1)], 1)
        self.assertEqual(selected, [1, 2])
        self.assertEqual(stats['rejected_motion'], 1)
        self.assertEqual(stats['rejected_quality'], 2)
        self.assertEqual(stats['input'], stats['accepted'] + stats['rejected_motion']
                         + stats['rejected_quality'] + stats['rejected_temporal'])


class RosViewTests(unittest.TestCase):
    """Synthetic ROS topic checks."""

    def test_decay_change_preserves_pending_transform_hit_age(self):
        rclpy.init(args=['--ros-args', '-p', 'grid_frame_id:=odom', '-p', 'tf_wait_seconds:=2.0'])
        radar_views = views.RadarViews()
        now = 10.0
        radar_views.now_seconds = lambda: now
        header = Header(frame_id='umrr96')
        header.stamp.sec = 10
        layout = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'range', 'azimuth_angle', 'snr'))]
        try:
            radar_views.receive(point_cloud2.create_cloud(header, layout, [(1, 0, 0, 1, 0, 20)]))
            self.assertEqual(len(radar_views.pending_grid), 1)
            now = 11.0
            changed = radar_views.set_parameters_atomically([Parameter('decay_seconds', value=.5)])
            self.assertTrue(changed.successful)
            self.assertEqual(len(radar_views.pending_grid), 1)
            self.assertEqual(radar_views.grid.samples(), [])
            tf = TransformStamped(header=Header(frame_id='odom'), child_frame_id='umrr96')
            tf.header.stamp = header.stamp
            tf.transform.rotation.w = 1.0
            radar_views.tf_buffer.set_transform(tf, 'fixture')
            now = 11.25
            radar_views.publish()
            self.assertEqual(len(radar_views.pending_grid), 0)
            self.assertAlmostEqual(radar_views.grid.samples()[0][3], math.exp(-1))
            now = 11.5
            radar_views.publish()
            self.assertEqual(radar_views.grid.samples(), [])
        finally:
            radar_views.destroy_node()
            rclpy.shutdown()

    def test_world_grid_uses_each_scan_transform_without_changing_sensor_fan(self):
        rclpy.init(args=['--ros-args', '-p', 'grid_frame_id:=odom', '-p', 'tf_wait_seconds:=0.0'])
        radar_views = views.RadarViews()
        peer = rclpy.create_node('umrr96_tf_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(radar_views)
        executor.add_node(peer)
        cells, fan = [], []
        peer.create_subscription(PointCloud2, '/smart_radar/density_cells', cells.append, 1)
        peer.create_subscription(PointCloud2, '/smart_radar/fan_targets', fan.append, 1)
        layout = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x', 'y', 'z', 'range', 'azimuth_angle', 'snr'))]

        def cloud(stamp, x, z=0):
            header = Header(frame_id='umrr96')
            header.stamp.sec = stamp
            return point_cloud2.create_cloud(header, layout, [(x, 0, z, math.hypot(x, z), 0, 20)])

        def transform(stamp, x, y=0, yaw=0, pitch=0):
            tf = TransformStamped(header=Header(frame_id='odom'), child_frame_id='umrr96')
            tf.header.stamp.sec = stamp
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.rotation.w = math.cos(yaw / 2) * math.cos(pitch / 2)
            tf.transform.rotation.x = -math.sin(yaw / 2) * math.sin(pitch / 2)
            tf.transform.rotation.y = math.cos(yaw / 2) * math.sin(pitch / 2)
            tf.transform.rotation.z = math.sin(yaw / 2) * math.cos(pitch / 2)
            radar_views.tf_buffer.set_transform(tf, 'fixture')

        try:
            deadline = time.monotonic() + 3
            while (radar_views.fan_pub.get_subscription_count() == 0
                   and time.monotonic() < deadline):
                executor.spin_once(timeout_sec=.05)
            # The same world target as the sensor translates, yaws, and pitches.
            transform(10, 0)
            radar_views.receive(cloud(10, 5.125))
            transform(11, 1)
            radar_views.receive(cloud(11, 4.125))
            transform(12, 5.125, -4, yaw=math.pi / 2)
            radar_views.receive(cloud(12, 4))
            transform(13, 3.125, pitch=math.pi / 2)
            radar_views.receive(cloud(13, 1, z=2))
            self.assertEqual(len(radar_views.grid.samples()), 1)
            self.assertGreater(radar_views.grid.samples()[0][3], 3.5)
            self.assertEqual(radar_views.grid.samples()[0][0], 5.125)
            radar_views.publish()
            deadline = time.monotonic() + 3
            while (not cells or not fan) and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertEqual(cells[-1].header.frame_id, 'odom')
            self.assertEqual(fan[-1].header.frame_id, 'umrr96')
            # Neither latest TF nor identity TF is a fallback for missing timestamps.
            radar_views.receive(cloud(20, 2))
            radar_views.receive(cloud(0, 2))
            self.assertEqual(radar_views.tf_dropped, 2)
            self.assertEqual(len(radar_views.grid.samples()), 1)
            self.assertEqual(len(radar_views.image_targets), 1)  # Fan still usable without TF.
            radar_views.grid.clear()
            radar_views.receive(cloud(12, 4))  # Still uses t=12, not the latest (t=13).
            self.assertEqual(radar_views.grid.samples()[0][0], 5.125)
        finally:
            executor.remove_node(peer)
            executor.remove_node(radar_views)
            peer.destroy_node()
            radar_views.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

    def test_filter_selection_atomic_parameters_and_original_point_bytes(self):
        rclpy.init()
        radar_views = views.RadarViews()
        peer = rclpy.create_node('umrr96_filter_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(radar_views)
        executor.add_node(peer)
        filtered, statuses, cells = [], [], []
        peer.create_subscription(PointCloud2, '/smart_radar/filtered_targets_0',
                                 filtered.append, qos_profile_sensor_data)
        peer.create_subscription(PointCloud2, '/smart_radar/density_cells', cells.append, 1)
        peer.create_subscription(String, '/smart_radar/filter_status',
                                 lambda msg: statuses.append(json.loads(msg.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher = peer.create_publisher(PointCloud2, '/smart_radar/port_targets_0', 1)
        client = peer.create_client(SetParametersAtomically,
                                    '/umrr96_views/set_parameters_atomically')

        def wait(predicate):
            deadline = time.monotonic() + 5
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(predicate())

        def parameters(**values):
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
            future = client.call_async(request)
            wait(future.done)
            return future.result().result

        names = ('x', 'y', 'z', 'range', 'azimuth_angle', 'snr', 'radial_speed')
        layout = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(names)]
        layout.append(PointField(name='flags', offset=28, datatype=PointField.UINT32, count=1))
        points = [(3.1, 0, 0, 3.1, 0, 20, 0, 0x12345678),
                  (6.1, 0, 0, 6.1, 0, 25, 1, 0xFEDCBA98),
                  (9.1, 0, 0, 9.1, 0, 3, 1, 17),
                  (float('nan'), 0, 0, 2, 0, 30, 1, 29)]

        def send():
            cloud = point_cloud2.create_cloud(
                Header(frame_id='umrr96', stamp=peer.get_clock().now().to_msg()), layout, points)
            publisher.publish(cloud)
            wait(lambda: filtered and filtered[-1].header.stamp == cloud.header.stamp)
            return cloud

        try:
            wait(lambda: publisher.get_subscription_count() and statuses
                 and client.service_is_ready())
            cloud = send()
            self.assertEqual(filtered[-1].data, cloud.data)
            self.assertEqual(statuses[-1]['mode'], 'off')
            self.assertTrue(parameters(filter_mode='mapping').successful)
            wait(lambda: cells and cells[-1].width == 0)
            send()
            self.assertEqual(filtered[-1].width, 0)
            cloud = send()
            self.assertEqual(filtered[-1].width, 2)
            self.assertEqual(filtered[-1].fields, cloud.fields)
            self.assertEqual(filtered[-1].data, cloud.data[:64])
            wait(lambda: statuses[-1]['accepted'] == 2)
            self.assertEqual(statuses[-1]['rejected_quality'], 2)
            # RViz resends unchanged filters with decay. Keep both density and
            # persistence history, and publish the new value even without scans.
            history = list(radar_views.filter.history)
            cells_before = dict(radar_views.grid.cells)
            accepted_before = statuses[-1]['accepted']
            self.assertTrue(parameters(filter_mode='mapping', filter_min_snr_db=6.0,
                                       filter_min_abs_speed=.25, decay_seconds=1.0).successful)
            self.assertEqual(list(radar_views.filter.history), history)
            self.assertEqual(radar_views.grid.cells.keys(), cells_before.keys())
            self.assertTrue(all(0 < value <= cells_before[key]
                                for key, value in radar_views.grid.cells.items()))
            wait(lambda: statuses[-1]['density_decay_seconds'] == 1.0)
            self.assertEqual(statuses[-1]['accepted'], accepted_before)
            for invalid in (.09, 30.1, -1.0, float('nan'), float('inf')):
                rejected = parameters(filter_mode='off', decay_seconds=invalid)
                self.assertFalse(rejected.successful)
                self.assertEqual(radar_views.get_parameter('decay_seconds').value, 1.0)
                self.assertEqual(radar_views.get_parameter('filter_mode').value, 'mapping')
                self.assertEqual(list(radar_views.filter.history), history)
            rejected = parameters(filter_mode='invalid', filter_min_snr_db=50.0)
            self.assertFalse(rejected.successful)
            self.assertEqual(radar_views.get_parameter('filter_mode').value, 'mapping')
            self.assertEqual(radar_views.get_parameter('filter_min_snr_db').value, 6.0)
            self.assertTrue(parameters(filter_mode='moving').successful)
            cloud = send()
            self.assertEqual(filtered[-1].width, 1)
            self.assertEqual(filtered[-1].data, cloud.data[32:64])
            self.assertTrue(parameters(filter_mode='off').successful)
            cloud = send()
            self.assertEqual(filtered[-1].data, cloud.data)
        finally:
            executor.remove_node(peer)
            executor.remove_node(radar_views)
            peer.destroy_node()
            radar_views.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

    def test_projection_density_decay_reset_and_late_guides(self):
        rclpy.init()
        radar_views = views.RadarViews()
        peer = rclpy.create_node('umrr96_views_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(radar_views)
        executor.add_node(peer)
        cells, fan, markers, guides, images = [], [], [], [], []
        peer.create_subscription(PointCloud2, '/smart_radar/density_cells', cells.append, 1)
        peer.create_subscription(PointCloud2, '/smart_radar/fan_targets', fan.append, 1)
        peer.create_subscription(Marker, '/smart_radar/density_grid', markers.append, 1)
        peer.create_subscription(Image, '/smart_radar/fan_image', images.append, 1)
        publisher = peer.create_publisher(PointCloud2, '/smart_radar/port_targets_0', 1)

        def wait(predicate, timeout=5):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(predicate(), 'Synthetic topic check timed out')

        try:
            wait(lambda: publisher.get_subscription_count() > 0 and cells and markers and images)
            self.assertEqual(images[-1].encoding, 'rgb8')
            self.assertEqual((images[-1].width, images[-1].height), (960, 640))
            self.assertEqual(images[-1].step, 960 * 3)
            self.assertEqual(len(images[-1].data), 960 * 640 * 3)
            fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'range', 'azimuth_angle', 'snr'))]
            points = [(x, y, math.hypot(x, y), math.atan2(y, x), 30)
                      for x, y in ((1.11, -.01), (1.12, -.02), (25, 0), (-1, 0))]
            points.append((float('nan'), 0, 1, 0, 30))
            cloud = point_cloud2.create_cloud(
                Header(frame_id='umrr96', stamp=peer.get_clock().now().to_msg()), fields, points)
            publisher.publish(cloud)
            wait(lambda: fan and fan[-1].width == 2 and cells[-1].width == 1)
            output = point_cloud2.read_points(fan[-1])
            self.assertAlmostEqual(float(output[0]['x']), 1.11, places=5)
            self.assertAlmostEqual(float(output[0]['y']), -.01, places=5)
            self.assertEqual(float(output[0]['z']), 0)
            cell = point_cloud2.read_points(cells[-1])[0]
            self.assertEqual(float(cell['x']), 1.125)
            self.assertEqual(float(cell['y']), -.125)
            self.assertGreater(float(cell['density']), 1.5)
            self.assertEqual(markers[-1].type, Marker.CUBE_LIST)
            wait(lambda: images[-1].header.stamp == cloud.header.stamp)
            self.assertEqual(images[-1].header.frame_id, 'umrr96')

            # A subscriber joining after startup must still receive the fan guides.
            peer.create_subscription(MarkerArray, '/smart_radar/fan_guides', guides.append,
                                     QoSProfile(depth=1,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
            wait(lambda: guides)
            self.assertTrue({'5 m', '10 m', '15 m', '20 m'}.issubset(
                {marker.text for marker in guides[-1].markers}))
            wait(lambda: fan[-1].width == 0)
            wait(lambda: images[-1].header.stamp != cloud.header.stamp)
            np.testing.assert_array_equal(
                np.frombuffer(images[-1].data, dtype=np.uint8).reshape(640, 960, 3),
                radar_views.image_renderer.render([], 'NO RECENT SCAN'))
            wait(lambda: 0 < float(point_cloud2.read_points(cells[-1])[0]['density']) < 1.5)
            reset = peer.create_client(Empty, '/smart_radar/reset_density')
            self.assertTrue(reset.wait_for_service(timeout_sec=2))
            future = reset.call_async(Empty.Request())
            wait(lambda: future.done() and cells[-1].width == 0
                 and markers[-1].action == Marker.DELETE)
            cloud.header.frame_id = 'wrong_frame'
            publisher.publish(cloud)
            until = time.monotonic() + .3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.05)
            self.assertEqual(cells[-1].width, 0)

            # A target with elevation has different XY and range/azimuth projections.
            elevated = point_cloud2.create_cloud(Header(frame_id='umrr96'), fields,
                                                 [(1.1, 0, 5, 0, 25)])
            publisher.publish(elevated)
            wait(lambda: fan[-1].width == 1 and cells[-1].width == 1)
            self.assertEqual(float(point_cloud2.read_points(fan[-1])[0]['x']), 5)
            self.assertEqual(float(point_cloud2.read_points(cells[-1])[0]['x']), 1.125)
            publisher.publish(point_cloud2.create_cloud(Header(frame_id='umrr96'), fields, []))
            wait(lambda: fan[-1].width == 0)
            self.assertEqual(cells[-1].width, 1)  # Empty scans do not mark space as free.
        finally:
            executor.remove_node(peer)
            executor.remove_node(radar_views)
            peer.destroy_node()
            radar_views.destroy_node()
            executor.shutdown()
            rclpy.shutdown()


class ReferenceFilter:
    """The original per-point DetectionFilter loop (before P6), for equivalence."""

    def __init__(self, mode, min_snr_db=6.0, min_abs_speed=.25):
        self.mode, self.min_snr_db, self.min_abs_speed = mode, min_snr_db, min_abs_speed
        self.history = deque(maxlen=2)
        self.last_stamp = None

    def select(self, points, stamp):
        stats = {'mode': self.mode, 'input': len(points), 'accepted': 0,
                 'rejected_quality': 0, 'rejected_motion': 0, 'rejected_temporal': 0,
                 'min_snr_db': self.min_snr_db, 'min_abs_speed': self.min_abs_speed}
        if self.mode == 'off':
            stats['accepted'] = len(points)
            return list(range(len(points))), stats
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.history.clear()
        self.last_stamp = stamp
        while self.history and stamp - self.history[0][0] > .5:
            self.history.popleft()
        candidates, accepted = [], []
        for index, point in enumerate(points):
            x, y, z, radius, angle, snr, speed = (float(point[name]) for name in (
                'x', 'y', 'z', 'range', 'azimuth_angle', 'snr', 'radial_speed'))
            if (not all(math.isfinite(v) for v in (x, y, z, radius, angle, snr, speed))
                    or radius < 0 or abs(angle) > math.pi or snr < self.min_snr_db):
                stats['rejected_quality'] += 1
                continue
            if self.mode == 'moving' and abs(speed) < self.min_abs_speed:
                stats['rejected_motion'] += 1
                continue
            candidates.append((radius, angle))
            if self.mode == 'mapping' and not any(
                abs(radius - old_range) <= .5
                and abs(math.remainder(angle - old_angle, 2 * math.pi)) <= math.radians(3)
                for _, prior in self.history for old_range, old_angle in prior
            ):
                stats['rejected_temporal'] += 1
                continue
            accepted.append(index)
        self.history.append((stamp, candidates))
        stats['accepted'] = len(accepted)
        return accepted, stats


class VectorizedFilterTests(unittest.TestCase):
    NAMES = ('x', 'y', 'z', 'range', 'azimuth_angle', 'snr', 'radial_speed')

    def scan(self, rng, count, anchors):
        """Clustered float32 targets near earlier ranges/azimuths, gate edges and +/-pi."""
        dtype = np.dtype([(name, '<f4') for name in self.NAMES])
        points = np.zeros(count, dtype=dtype)
        base = anchors[rng.integers(0, len(anchors), count)]
        radius = base[:, 0] + rng.choice([0, .5, -.5, .49, .51, 1.0], count) * (
            rng.random(count) < .5) + rng.normal(0, .2, count)
        angle = base[:, 1] + rng.choice([0, 1, -1, .99, 1.01, 2], count) * math.radians(3) * (
            rng.random(count) < .5) + rng.normal(0, .02, count)
        angle[rng.random(count) < .05] = math.pi
        angle[rng.random(count) < .05] = -math.pi
        angle = np.where(np.abs(angle) > math.pi * 1.01, -angle, angle)
        points['range'], points['azimuth_angle'] = radius, angle
        points['x'], points['y'] = radius * np.cos(angle), radius * np.sin(angle)
        points['snr'] = rng.uniform(0, 40, count)
        points['radial_speed'] = rng.normal(0, 1, count)
        for name in ('x', 'range', 'snr', 'radial_speed'):
            points[name][rng.random(count) < .02] = np.nan
        points['range'][rng.random(count) < .02] *= -1
        return points

    def test_matches_reference_loop_in_every_mode(self):
        for seed, count in ((0, 40), (1, 300), (2, 1500), (3, 4096)):
            rng = np.random.default_rng(seed)
            anchors = np.column_stack((rng.uniform(0, 60, 64), rng.uniform(-3.2, 3.2, 64)))
            for mode in ('quality', 'mapping', 'moving', 'off'):
                reference = ReferenceFilter(mode, 12.0, .5)
                actual = views.DetectionFilter(mode, 12.0, .5)
                stamp, accepted, temporal = 100.0, 0, 0
                for step in range(8):
                    stamp += (.1, .1, .3, .1, -.2, .1, .6, .1)[step]
                    points = self.scan(rng, count if step % 3 else count // 4, anchors)
                    expected = reference.select(points, stamp)
                    self.assertEqual(actual.select(points, stamp), expected, (seed, mode, step))
                    self.assertEqual([h[0] for h in actual.history],
                                     [h[0] for h in reference.history])
                    accepted += expected[1]['accepted']
                    temporal += expected[1]['rejected_temporal']
                if mode == 'mapping':  # Both gate outcomes were exercised.
                    self.assertGreater(accepted, 0)
                    self.assertGreater(temporal, 0)

    def test_hash_and_pairwise_search_agree_on_gate_boundaries(self):
        prior_range = np.array([10.0, 10.0, 3.0, 3.0, 7.25])
        prior_angle = np.array([math.pi, -math.pi, 0.0, math.radians(3), -3.1])
        query_range = np.array([10.5, 9.5, 3.5000001, 3.0, 7.75, 10.0])
        query_angle = np.array([-math.pi + math.radians(2.9), math.pi, 0.0,
                                math.radians(6), 3.1, -math.pi + math.radians(3.1)])
        expected = [any(abs(r - pr) <= .5 and abs(math.remainder(a - pa, 2 * math.pi))
                        <= math.radians(3) for pr, pa in zip(prior_range, prior_angle))
                    for r, a in zip(query_range, query_angle)]
        tiled = detection_filter.BRUTE_FORCE_PAIRS
        many_range = np.tile(query_range, tiled // len(query_range) + 1)
        many_angle = np.tile(query_angle, tiled // len(query_angle) + 1)
        hashed = detection_filter.confirmed(many_range, many_angle, prior_range, prior_angle)
        self.assertEqual(hashed[:len(expected)].tolist(), expected)
        self.assertEqual(detection_filter.confirmed(
            query_range, query_angle, prior_range, prior_angle).tolist(), expected)


def make_views(*overrides):
    args = ['--ros-args']
    for override in overrides:
        args += ['-p', override]
    rclpy.init(args=args)
    return views.RadarViews()


class Recorder:
    """Publisher stand-in; ``subscribers`` controls the reported subscription count."""

    def __init__(self, subscribers=1):
        self.messages, self.subscribers = [], subscribers

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


class ViewsNodeTests(unittest.TestCase):
    LAYOUT = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'range', 'azimuth_angle', 'snr',
                                        'radial_speed'))]

    def tearDown(self):
        rclpy.try_shutdown()

    def test_integer_overrides_and_runtime_integer_sets(self):
        radar_views = make_views('max_range:=20', 'publish_hz:=5', 'decay_seconds:=3',
                                 'filter_min_snr_db:=8', 'cell_size:=1', 'tf_wait_seconds:=1')
        try:
            self.assertEqual((radar_views.range, radar_views.grid.decay_seconds), (20.0, 3.0))
            self.assertEqual(radar_views.grid.resolution, 1.0)
            self.assertEqual(radar_views.filter.min_snr_db, 8.0)
            self.assertEqual(radar_views.timer.timer_period_ns, 200_000_000)
            self.assertTrue(radar_views.describe_parameter('max_range').description)
            self.assertTrue(radar_views.set_parameters_atomically(
                [Parameter('decay_seconds', value=1)]).successful)
            self.assertEqual(radar_views.grid.decay_seconds, 1.0)
            self.assertTrue(radar_views.set_parameters_atomically(
                [Parameter('filter_min_abs_speed', value=1)]).successful)
            self.assertEqual(radar_views.filter.min_abs_speed, 1.0)
            for invalid in ('high', 31, True):
                self.assertFalse(radar_views.set_parameters_atomically(
                    [Parameter('decay_seconds', value=invalid)]).successful)
            self.assertEqual(radar_views.grid.decay_seconds, 1.0)
        finally:
            radar_views.destroy_node()

    def test_filter_change_clears_filtered_cloud_with_a_current_stamp(self):
        radar_views = make_views()
        filtered = Recorder()
        try:
            radar_views.filtered_pub = filtered
            header = Header(frame_id='umrr96')
            header.stamp.sec = 10  # An old receive stamp.
            radar_views.receive(point_cloud2.create_cloud(
                header, self.LAYOUT, [(1, 0, 0, 1, 0, 20, 0)]))
            self.assertEqual(filtered.messages[-1].header.stamp.sec, 10)
            before = radar_views.get_clock().now().nanoseconds
            self.assertTrue(radar_views.set_parameters_atomically(
                [Parameter('filter_mode', value='quality')]).successful)
            clear = filtered.messages[-1]
            self.assertEqual(clear.width, 0)
            self.assertEqual(clear.fields, filtered.messages[0].fields)
            self.assertGreaterEqual(Time.from_msg(clear.header.stamp).nanoseconds, before)
            self.assertEqual(clear.header.frame_id, 'umrr96')
        finally:
            radar_views.destroy_node()

    def test_headless_builds_no_display_messages_or_image(self):
        radar_views = make_views()
        try:
            for name in ('grid_pub', 'cells_pub', 'fan_pub', 'image_pub'):
                setattr(radar_views, name, Recorder(subscribers=0))
            filtered = radar_views.filtered_pub = Recorder(subscribers=0)
            header = Header(frame_id='umrr96')
            header.stamp = radar_views.get_clock().now().to_msg()
            radar_views.receive(point_cloud2.create_cloud(
                header, self.LAYOUT, [(1, 0, 0, 1, 0, 20, 0)]))
            radar_views.publish()
            self.assertEqual(len(filtered.messages), 1)  # The navigation output still flows.
            self.assertEqual(len(radar_views.grid.samples()), 1)  # Density history too.
            self.assertTrue(all(not getattr(radar_views, name).messages for name in (
                'grid_pub', 'cells_pub', 'fan_pub', 'image_pub')))
            self.assertIsNone(radar_views._image_renderer)
            radar_views.cells_pub.subscribers = 1
            radar_views.publish()
            self.assertEqual(radar_views.cells_pub.messages[-1].width, 1)
            self.assertFalse(radar_views.grid_pub.messages)
        finally:
            radar_views.destroy_node()


if __name__ == '__main__':
    unittest.main()
