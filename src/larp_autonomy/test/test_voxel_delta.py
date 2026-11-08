from types import SimpleNamespace as NS
import numpy as np
from larp_autonomy.voxel_delta import VoxelDelta, cloud_xyz


def test_replacements_deletions_and_reconnect_state():
    delta = VoxelDelta()
    def update(points, epoch=1):
        return delta.update(np.array(points, dtype=np.float32).reshape(-1, 3),
                            [0, 0, 0], [4, 4, 2], .05, 'map', epoch)
    retained = dict(update([[.1, .1, .1], [3.1, 3.1, 1.1]]))
    assert len(retained) == 32
    assert update([[3.1, 3.1, 1.1], [.1, .1, .1]]) == []
    changed = update([[.2, .1, .1]])
    assert [slot for slot, _ in changed] == [0, 31]
    assert changed[1][1].shape == (0, 3)
    retained.update(changed)
    np.testing.assert_allclose(np.concatenate(list(retained.values())), [[.2, .1, .1]])
    assert len(update([[.2, .1, .1]], epoch=2)) == 32
    assert len(update([])) == 32  # changed epoch resets all IDs
    assert update([]) == []


def test_cloud_padding_endianness_and_nan():
    data = bytearray(40)
    view = np.ndarray((2, 1, 3), dtype='>f4', buffer=data, strides=(20, 16, 4))
    view[0, 0] = [1, 2, 3]
    view[1, 0] = [float('nan'), 4, 5]
    cloud = NS(width=1, height=2, point_step=16, row_step=20,
               is_bigendian=True, data=data,
               fields=[NS(name=n, datatype=7, count=1, offset=i*4)
                       for i, n in enumerate(('x', 'y', 'z'))])
    np.testing.assert_array_equal(cloud_xyz(cloud), [[1, 2, 3]])
