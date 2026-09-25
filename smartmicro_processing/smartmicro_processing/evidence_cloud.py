# SPDX-License-Identifier: Apache-2.0
"""An explicit derived schema: geometry in output frame, radiometry at source."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('source_radial_speed', '<f8'),
    ('source_snr', '<f8'), ('age_seconds', '<f4'), ('span_seconds', '<f4'),
    ('support_scans', '<u4'), ('source_point_index', '<u4'),
    ('source_stamp_sec', '<i4'), ('source_stamp_nanosec', '<u4'),
])
TYPES = {'<f8': PointField.FLOAT64, '<f4': PointField.FLOAT32,
         '<u4': PointField.UINT32, '<i4': PointField.INT32}


def evidence_cloud(samples, header):
    data = np.empty(len(samples), dtype=DTYPE)
    for index, sample in enumerate(samples):
        stamp = sample['source_stamp_ns']
        data[index] = (*sample['value'], sample['age_seconds'], sample['span_seconds'],
                       sample['support_scans'], sample['source_index'],
                       stamp // 1_000_000_000, stamp % 1_000_000_000)
    fields = [PointField(name=name, offset=DTYPE.fields[name][1],
                         datatype=TYPES[DTYPE.fields[name][0].str], count=1)
              for name in DTYPE.names]
    return PointCloud2(header=header, height=1, width=len(samples), fields=fields,
                       is_bigendian=False, point_step=DTYPE.itemsize,
                       row_step=data.nbytes, data=data.tobytes(), is_dense=True)
