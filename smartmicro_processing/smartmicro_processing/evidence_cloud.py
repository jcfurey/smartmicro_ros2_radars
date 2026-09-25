# SPDX-License-Identifier: Apache-2.0
"""An explicit derived schema: geometry in output frame, radiometry at source."""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

from .accumulation import SNAPSHOT_DTYPE


DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('source_radial_speed', '<f8'),
    ('source_snr', '<f8'), ('age_seconds', '<f4'), ('span_seconds', '<f4'),
    ('support_scans', '<u4'), ('source_point_index', '<u4'),
    ('source_stamp_sec', '<i4'), ('source_stamp_nanosec', '<u4'),
])
TYPES = {'<f8': PointField.FLOAT64, '<f4': PointField.FLOAT32,
         '<u4': PointField.UINT32, '<i4': PointField.INT32}


def _as_snapshot_array(samples):
    if isinstance(samples, np.ndarray):
        return samples
    result = np.empty(len(samples), dtype=SNAPSHOT_DTYPE)
    for index, sample in enumerate(samples):
        result[index] = (sample['value'], sample['source_index'], sample['source_stamp_ns'],
                         sample.get('first_stamp_ns', sample['source_stamp_ns']),
                         sample['support_scans'], sample['age_seconds'], sample['span_seconds'])
    return result


def evidence_cloud(samples, header):
    """Pack a snapshot (record array or list of dicts) into the derived schema."""
    samples = _as_snapshot_array(samples)
    data = np.empty(len(samples), dtype=DTYPE)
    values = samples['value']
    for column, name in enumerate(('x', 'y', 'z', 'source_radial_speed', 'source_snr')):
        data[name] = values[:, column]
    data['age_seconds'] = samples['age_seconds']
    data['span_seconds'] = samples['span_seconds']
    data['support_scans'] = samples['support_scans']
    data['source_point_index'] = samples['source_index']
    stamps = samples['source_stamp_ns']
    data['source_stamp_sec'] = stamps // 1_000_000_000
    data['source_stamp_nanosec'] = stamps % 1_000_000_000
    fields = [PointField(name=name, offset=DTYPE.fields[name][1],
                         datatype=TYPES[DTYPE.fields[name][0].str], count=1)
              for name in DTYPE.names]
    return PointCloud2(header=header, height=1, width=len(samples), fields=fields,
                       is_bigendian=False, point_step=DTYPE.itemsize,
                       row_step=data.nbytes, data=data.tobytes(), is_dense=True)
