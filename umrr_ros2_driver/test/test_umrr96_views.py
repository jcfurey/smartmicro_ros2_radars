#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline density math and synthetic ROS topic checks; no radar required."""

import importlib.util
import math
from pathlib import Path
import time
import unittest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


source = Path(__file__).resolve().parents[1] / 'scripts/umrr96_views.py'
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


class RosViewTests(unittest.TestCase):
    def test_projection_density_decay_reset_and_late_guides(self):
        rclpy.init()
        radar_views = views.RadarViews()
        peer = rclpy.create_node('umrr96_views_fixture')
        executor = SingleThreadedExecutor()
        executor.add_node(radar_views)
        executor.add_node(peer)
        cells, fan, markers, guides = [], [], [], []
        peer.create_subscription(PointCloud2, '/smart_radar/density_cells', cells.append, 1)
        peer.create_subscription(PointCloud2, '/smart_radar/fan_targets', fan.append, 1)
        peer.create_subscription(Marker, '/smart_radar/density_grid', markers.append, 1)
        publisher = peer.create_publisher(PointCloud2, '/smart_radar/port_targets_0', 1)

        def wait(predicate, timeout=5):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.05)
            self.assertTrue(predicate(), 'Synthetic topic check timed out')

        try:
            wait(lambda: publisher.get_subscription_count() > 0 and cells and markers)
            fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, name in enumerate(('x', 'y', 'range', 'azimuth_angle', 'snr'))]
            points = [(x, y, math.hypot(x, y), math.atan2(y, x), 30)
                      for x, y in ((1.11, -.01), (1.12, -.02), (25, 0), (-1, 0))]
            points.append((float('nan'), 0, 1, 0, 30))
            cloud = point_cloud2.create_cloud(Header(frame_id='umrr96'), fields, points)
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

            # A subscriber joining after startup must still receive the fan guides.
            peer.create_subscription(MarkerArray, '/smart_radar/fan_guides', guides.append,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            wait(lambda: guides)
            self.assertTrue({'5 m', '10 m', '15 m', '20 m'}.issubset(
                {marker.text for marker in guides[-1].markers}))
            wait(lambda: fan[-1].width == 0)
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
