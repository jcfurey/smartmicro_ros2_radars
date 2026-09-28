# SPDX-License-Identifier: Apache-2.0
"""Explain existing decisions and render current measurements without scan history."""
from copy import deepcopy

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from umrr_ros2_msgs.msg import DetectionAudit

from .cloud import quality_rejections
from .ghosts import GhostReason

DISPLAY_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
    ('source_index', '<u4'), ('classification', 'u1'), ('track_associated', 'u1'),
    ('reason_flags', '<u2')])
DISPLAY_FIELDS = [PointField(name=name, offset=DISPLAY_DTYPE.fields[name][1],
                             datatype=datatype, count=1)
                  for name, datatype in zip(DISPLAY_DTYPE.names, (
                      PointField.FLOAT32, PointField.FLOAT32, PointField.FLOAT32,
                      PointField.UINT32, PointField.UINT32, PointField.UINT8,
                      PointField.UINT8, PointField.UINT16))]
# Rejected gray, failed-fit amber, static white, moving salmon, suspected ghost blue.
COLORS = np.array([0x888888, 0xFFC94A, 0xEBEBEB, 0xFFAA96, 0x5A8CFF], dtype=np.uint32)
TRACK_COLOR = 0xFF3C28


def classify(header, values, gate_config, fit, indices, mover_reasons, tracked_indices,
             ghost_rejected=None):
    """Classify every original row, keeping independent ghost triggers and raw indices."""
    labels = np.full(len(values), DetectionAudit.REJECTED, dtype=np.uint8)
    reasons = np.zeros(len(values), dtype=np.uint16)
    tracked = np.zeros(len(values), dtype=bool)
    for rejected, code in zip(quality_rejections(values, gate_config), (
            DetectionAudit.NONFINITE, DetectionAudit.OUT_OF_RANGE, DetectionAudit.LOW_SNR)):
        reasons[rejected] = code
    if fit.valid:
        labels[indices[fit.inliers]] = DetectionAudit.STATIC
        movers = indices[~fit.inliers]
        rejected = (mover_reasons != 0 if ghost_rejected is None
                    else np.asarray(ghost_rejected, dtype=bool))
        if rejected.shape != (len(movers),):
            raise ValueError('Ghost rejection decisions must match the moving detections')
        labels[movers] = np.where(rejected, DetectionAudit.SUSPECTED_GHOST,
                                  DetectionAudit.MOVING)
        for flag, code in ((GhostReason.SAME_SPEED, DetectionAudit.GHOST_SAME_SPEED),
                           (GhostReason.DOUBLE_SPEED, DetectionAudit.GHOST_DOUBLE_SPEED),
                           (GhostReason.BEHIND_STATIC, DetectionAudit.GHOST_BEHIND_STATIC)):
            reasons[movers[(mover_reasons & flag) != 0]] |= code
        tracked[tracked_indices] = True
    else:
        labels[indices] = DetectionAudit.UNCLASSIFIED
        reasons[indices] = DetectionAudit.FIT_REJECTED
    return DetectionAudit(
        header=deepcopy(header), event=DetectionAudit.SCAN, status=fit.reason,
        fit_valid=fit.valid, source_index=list(range(len(values))),
        classification=labels.tolist(), reason_flags=reasons.tolist(),
        track_associated=tracked.tolist())


def classified_cloud(header, values, audit):
    """Render finite XYZ from this scan only; the audit still includes nonfinite positions."""
    # This display adapter is deliberately separate from byte-preserving raw/subset clouds.
    with np.errstate(over='ignore', invalid='ignore'):
        xyz = np.asarray(values[:, :3], dtype=np.float32)
    indices = np.flatnonzero(np.isfinite(xyz).all(axis=1))
    points = np.zeros(len(indices), dtype=DISPLAY_DTYPE)
    for column, name in enumerate('xyz'):
        points[name] = xyz[indices, column]
    labels = np.asarray(audit.classification, dtype=np.uint8)[indices]
    tracked = np.asarray(audit.track_associated, dtype=bool)[indices]
    points['rgb'] = COLORS[labels]
    points['rgb'][tracked] = TRACK_COLOR
    points['source_index'] = indices
    points['classification'] = labels
    points['reason_flags'] = np.asarray(audit.reason_flags, dtype=np.uint16)[indices]
    points['track_associated'] = tracked
    return create_cloud(deepcopy(header), DISPLAY_FIELDS, points)


def cleared_audit(header, reason):
    """Distinguish a cleared display from a real empty sensor scan."""
    return DetectionAudit(header=deepcopy(header), event=DetectionAudit.CLEAR, status=reason)
