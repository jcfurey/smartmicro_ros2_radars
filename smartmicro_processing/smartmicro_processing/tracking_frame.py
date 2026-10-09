# SPDX-License-Identifier: Apache-2.0
"""Sensor pose in a fixed tracking frame (e.g. ``odom``) at one scan's stamp."""
import numpy as np


class SensorPose:
    """
    ``tracking_frame <- sensor`` transform: translation and XYZW rotation.

    Raises ValueError for a nonfinite translation or a rotation that is not a
    unit quaternion (within 1e-3).
    """

    def __init__(self, translation, quaternion):
        translation = np.asarray(translation, float).reshape(3)
        quaternion = np.asarray(quaternion, float).reshape(4)
        norm = np.linalg.norm(quaternion)
        if (not np.isfinite(translation).all() or not np.isfinite(quaternion).all()
                or abs(norm - 1) > 1e-3):
            raise ValueError('Tracking transform needs a finite translation and unit rotation')
        x, y, z, w = quaternion / norm
        self.translation = translation
        self.rotation = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    @classmethod
    def from_transform(cls, transform):
        """Build from a ``geometry_msgs/Transform``."""
        t, q = transform.translation, transform.rotation
        return cls([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

    @property
    def origin(self):
        """Sensor position (x, y) in the tracking frame."""
        return self.translation[:2].copy()

    def to_tracking(self, xyz):
        """Map sensor-frame points (N, 3) into the tracking frame."""
        return np.asarray(xyz, float).reshape(-1, 3) @ self.rotation.T + self.translation

    def to_sensor(self, xyz):
        """Map tracking-frame points (N, 3) into the sensor frame."""
        return (np.asarray(xyz, float).reshape(-1, 3) - self.translation) @ self.rotation
