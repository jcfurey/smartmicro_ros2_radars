# SPDX-License-Identifier: Apache-2.0
import copy
import struct

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from smartmicro_processing.cloud import measurements, select_measurements, subset_cloud
from std_msgs.msg import Header


def cloud_fixture(big_endian=False):
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'radial_speed', 'snr',
                                        'false_alarm_probability'))]
    rows = [(2., 1., .1, 0., 25., float('nan')),
            (3., -1., -.1, -.5, 30., float('nan')),
            (4., 2., .3, .7, 35., float('nan')),
            (5., -2., -.3, 0., 40., float('nan'))]
    records = [struct.pack(('>' if big_endian else '<') + '6f', *row) + b'KEEP' for row in rows]
    return PointCloud2(header=Header(frame_id='umrr96'), height=2, width=2, fields=fields,
                       is_bigendian=big_endian, point_step=28, row_step=59,
                       data=records[0] + records[1] + b'PAD' + records[2] + records[3] + b'PAD',
                       is_dense=False), records


@pytest.mark.parametrize('big_endian', [False, True])
def test_unknown_nans_padding_endianness_and_byte_preserving_subsets(big_endian):
    cloud, records = cloud_fixture(big_endian)
    original = bytes(cloud.data)
    values = measurements(cloud)
    np.testing.assert_allclose(values[:, 0], [2, 3, 4, 5])
    np.testing.assert_allclose(values[:, 3], [0, -.5, .7, 0])
    indices, stats = select_measurements(values)
    assert stats['accepted'] == 4
    assert indices.tolist() == [0, 1, 2, 3]
    selected = subset_cloud(cloud, [3, 0])
    assert bytes(selected.data) == records[3] + records[0]
    assert selected.height == 1 and selected.width == 2 and selected.row_step == 56
    assert selected.fields == cloud.fields and selected.header == cloud.header
    assert selected.is_bigendian == cloud.is_bigendian
    assert not selected.is_dense  # Optional NaNs are still unknown.
    assert bytes(cloud.data) == original


def test_rejection_counts_and_no_zero_doppler_gate():
    cloud, _ = cloud_fixture()
    values = measurements(cloud)
    values[0, 0] = np.nan
    values[1, :3] = 0
    values[2, 4] = 2
    indices, stats = select_measurements(values)
    assert indices.tolist() == [3]
    assert stats == {'input': 4, 'accepted': 1, 'rejected_nonfinite': 1, 'rejected_range': 1,
                     'rejected_snr': 1}


@pytest.mark.parametrize('failure', ['missing', 'truncated', 'overlap', 'type', 'row_step'])
def test_malformed_schema_fails_closed(failure):
    cloud, _ = cloud_fixture()
    cloud = copy.deepcopy(cloud)
    if failure == 'missing':
        cloud.fields = cloud.fields[1:]
    elif failure == 'truncated':
        cloud.data = cloud.data[:-1]
    elif failure == 'overlap':
        cloud.fields[1].offset = 0
    elif failure == 'type':
        cloud.fields[0].datatype = PointField.UINT32
    elif failure == 'row_step':
        cloud.row_step = 1
    with pytest.raises(ValueError):
        measurements(cloud)


def test_empty_cloud_is_well_formed():
    cloud, _ = cloud_fixture()
    empty = subset_cloud(cloud, [])
    values = measurements(empty)
    assert values.shape == (0, 5)
    indices, stats = select_measurements(values)
    assert not len(indices) and stats['input'] == 0
