# SPDX-License-Identifier: Apache-2.0
"""
Confirmed tracks as ``radar_msgs/RadarTracks`` (radar_msgs is optional).

The EKF state is planar ``[px, py, vx, vy]``; z, acceleration and size are not
estimated and carry ``UNOBSERVED_VARIANCE``. Covariances are upper triangles in
the order xx, xy, xz, yy, yz, zz.
"""
import uuid

try:
    from radar_msgs.msg import RadarTrack, RadarTracks
    from unique_identifier_msgs.msg import UUID
except ImportError:  # radar_msgs not installed: the node does not publish ~/tracks
    RadarTrack = RadarTracks = UUID = None

UNOBSERVED_VARIANCE = 1e6  # variance of z, acceleration and size (not estimated)


def track_uuid(namespace, track_id):
    """
    Return a deterministic UUID for a track of the node named ``namespace``.

    UUIDv5 (URL namespace) of ``ros:<namespace>/track/<id>``: stable for a
    track within a run and distinct between nodes; it repeats after the node
    restarts, like ``track_id``.
    """
    return uuid.uuid5(uuid.NAMESPACE_URL, f'ros:{namespace}/track/{int(track_id)}')


def upper_triangle(planar, z_variance=UNOBSERVED_VARIANCE):
    """Return (xx, xy, xz, yy, yz, zz) of a planar 2x2 covariance and a z variance."""
    return [float(planar[0][0]), float(planar[0][1]), 0., float(planar[1][1]), 0.,
            float(z_variance)]


def radar_tracks(header, tracks, namespace, z=0.):
    """
    Build ``RadarTracks`` from confirmed tracks; ``z`` is the published height.

    Position and velocity are the EKF state as in ``tracked_objects``;
    acceleration is zero and size a zero placeholder, both with
    ``UNOBSERVED_VARIANCE``; classification is DYNAMIC (Doppler movers).
    """
    unknown = [UNOBSERVED_VARIANCE, 0., 0., UNOBSERVED_VARIANCE, 0., UNOBSERVED_VARIANCE]
    message = RadarTracks(header=header)
    for track in tracks:
        item = RadarTrack(uuid=UUID(uuid=list(track_uuid(namespace, track.track_id).bytes)),
                          classification=RadarTrack.DYNAMIC)
        item.position.x, item.position.y = float(track.x[0]), float(track.x[1])
        item.position.z = float(z)
        item.velocity.x, item.velocity.y = float(track.x[2]), float(track.x[3])
        item.position_covariance = upper_triangle(track.P[:2, :2])
        item.velocity_covariance = upper_triangle(track.P[2:, 2:])
        item.acceleration_covariance = unknown
        item.size_covariance = unknown
        message.tracks.append(item)
    return message
