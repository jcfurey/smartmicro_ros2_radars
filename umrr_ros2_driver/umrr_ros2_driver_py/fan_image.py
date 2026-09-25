# SPDX-License-Identifier: Apache-2.0
"""
Rasterize radar target bins into an annotated RGB fan image.

OpenCV is imported only when an image is built, so a headless views node
without image subscribers does not need it.
"""

import math

import numpy as np


def _cv2():
    import cv2
    return cv2


class FanImage:
    """A pixel image of the latest target list, with no invented radar returns."""

    @staticmethod
    def check(max_range, half_angle, range_bin=.25, angle_bin_degrees=2.0,
              width=960, height=640):
        """Validate the settings without building the image; return the bin counts."""
        if not (640 <= width <= 1920 and 480 <= height <= 1080):
            raise ValueError('Image size must be 640..1920 by 480..1080')
        if not all(math.isfinite(v) and v > 0 for v in (
                max_range, half_angle, range_bin, angle_bin_degrees)):
            raise ValueError('Image range, angle, and bin sizes must be finite and positive')
        if half_angle > math.pi or not .25 <= angle_bin_degrees <= 10:
            raise ValueError('Image half angle must be <= pi; angle bins must be .25..10 degrees')
        range_bins = math.ceil(max_range / range_bin)
        angle_bins = math.ceil(2 * half_angle / math.radians(angle_bin_degrees))
        if range_bins * angle_bins > 1000000:
            raise ValueError('Image exceeds one million polar bins')
        return range_bins, angle_bins

    def __init__(self, max_range, half_angle, range_bin=.25, angle_bin_degrees=2.0,
                 width=960, height=640):
        self.range_bins, self.angle_bins = self.check(
            max_range, half_angle, range_bin, angle_bin_degrees, width, height)
        self.width, self.height = width, height
        self.max_range, self.half_angle = max_range, half_angle
        self.range_bin = range_bin
        cv2 = _cv2()
        self.angle_bin = 2 * half_angle / self.angle_bins
        # Fit the entire selected sector, including rear quadrants above +/-90 degrees.
        lateral = max_range * math.sin(min(half_angle, math.pi / 2))
        rear = max_range * min(0, math.cos(half_angle))
        self.scale = min((width - 140) / (2 * lateral),
                         (height - 244) / (max_range - rear))
        self.origin = (width / 2, 104 + max_range * self.scale)
        rows, cols = np.indices((height, width))
        x = (self.origin[1] - rows) / self.scale
        y = (self.origin[0] - cols) / self.scale
        radius, angle = np.hypot(x, y), np.arctan2(y, x)
        self.mask = (radius <= max_range) & (np.abs(angle) <= half_angle)
        self.pixel_bins = (
            np.minimum((radius[self.mask] / range_bin).astype(int), self.range_bins - 1),
            np.minimum(((angle[self.mask] + half_angle) / self.angle_bin).astype(int),
                       self.angle_bins - 1))
        ramp = np.arange(256, dtype=np.uint8).reshape(256, 1)
        self.palette = cv2.cvtColor(cv2.applyColorMap(ramp, cv2.COLORMAP_TURBO),
                                    cv2.COLOR_BGR2RGB).reshape(256, 3)
        self.background = np.full((height, width, 3), (10, 17, 25), dtype=np.uint8)
        self.background[self.mask] = (17, 32, 42)
        self.overlay = np.zeros_like(self.background)
        self._draw_guides()
        self.ink = np.any(self.overlay != 0, axis=2)

    def pixel(self, radius, angle):
        """Map forward range to image up, and positive azimuth to image left."""
        return (round(self.origin[0] - radius * math.sin(angle) * self.scale),
                round(self.origin[1] - radius * math.cos(angle) * self.scale))

    @staticmethod
    def text(image, text, position, scale=.45, color=(145, 175, 193), centered=False):
        cv2 = _cv2()
        u, v = position
        if centered:
            (width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            u -= width // 2
        cv2.putText(image, text, (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

    def _draw_guides(self):
        cv2 = _cv2()
        canvas = self.overlay
        line = (45, 69, 83)
        for i in range(1, 5):
            radius = self.max_range * i / 4
            arc = np.array([self.pixel(radius, a) for a in np.linspace(
                -self.half_angle, self.half_angle, 361)], dtype=np.int32)
            cv2.polylines(canvas, [arc], False, line, 1, cv2.LINE_AA)
            u, v = self.pixel(radius, 0)
            self.text(canvas, f'{radius:g} m', (u + 8, v - 4))
        for angle in np.linspace(-self.half_angle, self.half_angle, 7):
            cv2.line(canvas, self.pixel(0, 0), self.pixel(self.max_range, angle),
                     line, 1, cv2.LINE_AA)
            u, v = self.pixel(self.max_range + 20 / self.scale, angle)
            self.text(canvas, f'{math.degrees(angle):+.0f} deg', (u, v + 5),
                      scale=.38, centered=True)
        u, v = self.pixel(0, 0)
        cv2.drawMarker(canvas, (u, v), (175, 213, 223), cv2.MARKER_TRIANGLE_UP, 9, 1)
        self.text(canvas, 'RADAR', (u, v + 23), centered=True)
        self.text(canvas, 'UMRR-96 / FAN IMAGE', (28, 32), .72, (220, 234, 241))
        self.text(canvas, 'Latest target list | forward up | positive azimuth left', (28, 56))
        self.text(canvas, 'SNR (dB)', (60, self.height - 79))
        gradient = self.palette[np.linspace(0, 255, self.width - 120).astype(int)]
        canvas[self.height - 69:self.height - 55, 60:self.width - 60] = gradient
        for i in range(6):
            self.text(canvas, str(i * 10), (60 + i * (self.width - 120) / 5, self.height - 35),
                      centered=True)
        self.text(canvas, f'Display bins: {self.range_bin:g} m x '
                  f'{math.degrees(self.angle_bin):g} deg | strongest SNR per bin',
                  (28, self.height - 10), .4)

    def bin_targets(self, targets):
        """Return max SNR per range/azimuth bin; -inf means no detection."""
        bins = np.full((self.range_bins, self.angle_bins), -np.inf, dtype=np.float32)
        for radius, angle, snr in targets:
            if not all(math.isfinite(v) for v in (radius, angle, snr)):
                continue
            if not (0 <= radius <= self.max_range and abs(angle) <= self.half_angle):
                continue
            row = min(int(radius / self.range_bin), self.range_bins - 1)
            col = min(int((angle + self.half_angle) / self.angle_bin), self.angle_bins - 1)
            bins[row, col] = max(bins[row, col], snr)
        return bins

    def render(self, targets, state='LIVE', filter_mode='off'):
        """Return contiguous uint8 RGB pixels, including a waiting/stale indicator."""
        image = self.background.copy()
        if targets:
            values = self.bin_targets(targets)[self.pixel_bins]
            observed = np.isfinite(values)
            colors = image[self.mask]
            indices = np.rint(np.clip(values[observed], 0, 50) * 255 / 50).astype(int)
            colors[observed] = self.palette[indices]
            image[self.mask] = colors
        image[self.ink] = self.overlay[self.ink]
        status = f'{state} | {len(targets)} targets | filter: {filter_mode}'
        color = (115, 211, 170) if state == 'LIVE' else (246, 183, 88)
        self.text(image, status, (28, 81), .5, color)
        return image
