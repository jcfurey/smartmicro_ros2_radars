#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline density math and synthetic ROS topic checks; no radar required."""

import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import unittest

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.srv import SetParametersAtomically
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


source = Path(__file__).resolve().parents[1] / 'scripts/umrr96_views.py'
sys.path.insert(0, str(source.parent))
spec = importlib.util.spec_from_file_location('umrr96_views', source)
views = importlib.util.module_from_spec(spec)
spec.loader.exec_module(views)


class DensityTests(unittest.TestCase):
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
    @staticmethod
    def point(radius=3.1, angle=.2, snr=20, speed=0):
        return dict(x=radius * math.cos(angle), y=radius * math.sin(angle), z=0,
                    range=radius, azimuth_angle=angle, snr=snr, radial_speed=speed)

    def test_off_preserves_every_point(self):
        points = [self.point(snr=-50), self.point(radius=float('nan'))]
        selected, stats = views.DetectionFilter().select(points, 1)
        self.assertEqual(selected, [0, 1])
        self.assertEqual(stats['accepted'], 2)

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
        client = peer.create_client(SetParametersAtomically, '/umrr96_views/set_parameters_atomically')

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
            wait(lambda: publisher.get_subscription_count() and statuses and client.service_is_ready())
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
            self.assertEqual(statuses[-1]['rejected_quality'], 2)
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
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
            wait(lambda: future.done() and cells[-1].width == 0 and markers[-1].action == Marker.DELETE)
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


if __name__ == '__main__':
    unittest.main()
