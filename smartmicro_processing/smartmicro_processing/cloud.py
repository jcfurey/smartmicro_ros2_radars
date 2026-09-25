# SPDX-License-Identifier: Apache-2.0
"""Read only required measurements, preserving original point bytes in subsets."""
from copy import deepcopy
from dataclasses import dataclass
import math

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

REQUIRED = ('x', 'y', 'z', 'radial_speed', 'snr')


@dataclass(frozen=True)
class GateConfig:
    min_range: float = .15
    max_range: float = 120.0
    min_snr_db: float = 6.0

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.min_range, self.max_range, self.min_snr_db)):
            raise ValueError('Quality gates must be finite')
        if not 0 < self.min_range < self.max_range <= 300 or not -40 <= self.min_snr_db <= 100:
            raise ValueError('Invalid range/SNR gates')


def measurements(cloud):
    """Handle row padding and endian order without mutating the input buffer."""
    if cloud.width * cloud.height > 4096:
        raise ValueError('Cloud exceeds 4096 detections')
    if (cloud.height < 1 or cloud.point_step < 1 or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) != cloud.height * cloud.row_step):
        raise ValueError('Malformed PointCloud2 layout')
    fields = {f.name: f for f in cloud.fields}
    if len(fields) != len(cloud.fields) or not set(REQUIRED) <= fields.keys():
        raise ValueError('Cloud needs unique x, y, z, radial_speed, snr fields')
    formats, offsets = [], []
    for name in REQUIRED:
        field = fields[name]
        if field.datatype not in (PointField.FLOAT32, PointField.FLOAT64) or field.count != 1:
            raise ValueError(f'{name} must be a scalar floating-point field')
        dtype = np.dtype(('>' if cloud.is_bigendian else '<') +
                         ('f4' if field.datatype == PointField.FLOAT32 else 'f8'))
        if field.offset < 0 or field.offset + dtype.itemsize > cloud.point_step:
            raise ValueError(f'{name} extends past point_step')
        formats.append(dtype)
        offsets.append(field.offset)
    occupied = [set(range(offset, offset + dtype.itemsize))
                for offset, dtype in zip(offsets, formats)]
    if any(a & b for i, a in enumerate(occupied) for b in occupied[i + 1:]):
        raise ValueError('Required measurement fields overlap')
    dtype = np.dtype(dict(names=REQUIRED, formats=formats, offsets=offsets,
                          itemsize=cloud.point_step))
    if cloud.width == 0:
        return np.empty((0, len(REQUIRED)), dtype=float)
    points = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                        strides=(cloud.row_step, cloud.point_step))
    return np.column_stack([points[name].reshape(-1) for name in REQUIRED]).astype(float)


def select_measurements(values, config=GateConfig()):
    finite = np.isfinite(values).all(axis=1)
    ranges = np.linalg.norm(values[:, :3], axis=1)
    in_range = np.isfinite(ranges) & (ranges >= config.min_range) & (ranges <= config.max_range)
    enough_snr = values[:, 4] >= config.min_snr_db
    mask = finite & in_range & enough_snr
    stats = dict(input=len(values), accepted=int(mask.sum()),
                 rejected_nonfinite=int((~finite).sum()),
                 rejected_range=int((finite & ~in_range).sum()),
                 rejected_snr=int((finite & in_range & ~enough_snr).sum()))
    return np.flatnonzero(mask), stats


def subset_cloud(cloud, indices):
    """Gather whole point records; indices refer to row-major input points.

    Padding bytes inside a record and unknown fields survive unchanged; row
    padding between input rows is dropped because the output is one packed row.
    """
    indices = np.asarray(indices, dtype=np.int64)
    if indices.ndim != 1 or np.any(indices < 0) or np.any(indices >= cloud.width * cloud.height):
        raise ValueError('Invalid point indices')
    if len(indices):
        try:
            source = np.frombuffer(cloud.data, dtype=np.uint8)
        except TypeError:  # A plain list of byte values.
            source = np.frombuffer(bytes(cloud.data), dtype=np.uint8)
        starts = indices // cloud.width * cloud.row_step + indices % cloud.width * cloud.point_step
        if int(starts.max()) + cloud.point_step > len(source):
            raise ValueError('Point data shorter than its layout')
        data = source[starts[:, None] + np.arange(cloud.point_step)].tobytes()
    else:
        data = b''
    return PointCloud2(header=deepcopy(cloud.header), height=1, width=len(indices),
                       fields=deepcopy(cloud.fields), is_bigendian=cloud.is_bigendian,
                       point_step=cloud.point_step, row_step=len(indices) * cloud.point_step,
                       data=data, is_dense=False)


def empty_cloud(header):
    return PointCloud2(header=deepcopy(header), height=1, width=0,
                       fields=[PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                               for i, name in enumerate(REQUIRED)],
                       point_step=20, row_step=0, data=b'', is_dense=False)
