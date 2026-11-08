"""Bounded spatial chunk replacements for the visualization copy of a map."""
import numpy as np

CHUNK_COUNT = 32
GRID = np.array([4, 4, 2])


def cloud_xyz(cloud):
    if not cloud.width or not cloud.height:
        return np.empty((0, 3), dtype=np.float32)
    fields = {f.name: f for f in cloud.fields}
    if any(k not in fields or fields[k].datatype != 7 or fields[k].count != 1
           for k in ('x', 'y', 'z')):
        raise ValueError('Map visualization requires FLOAT32 xyz fields')
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if cloud.is_bigendian else '<') + 'f4'] * 3,
                      'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                      'itemsize': cloud.point_step})
    records = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                         buffer=bytes(cloud.data),
                         strides=(cloud.row_step, cloud.point_step))
    xyz = np.column_stack([records[k].ravel() for k in ('x', 'y', 'z')])
    return xyz[np.isfinite(xyz).all(axis=1)]


class VoxelDelta:
    def __init__(self):
        self.metadata = None
        self.chunks = [None] * CHUNK_COUNT

    def update(self, xyz, lower, upper, resolution, frame, epoch):
        lower, upper = np.asarray(lower), np.asarray(upper)
        if (not np.isfinite(lower).all() or not np.isfinite(upper).all()
                or np.any(upper <= lower) or not np.isfinite(resolution)
                or resolution <= 0):
            raise ValueError('Invalid map visualization bounds/resolution')
        metadata = (tuple(lower), tuple(upper), resolution, frame, epoch)
        reset = metadata != self.metadata
        slots = np.clip(np.floor((xyz - lower) / (upper - lower) * GRID),
                        0, GRID - 1).astype(np.int32)
        ids = slots @ np.array([1, 4, 16])
        order = np.lexsort((xyz[:, 2], xyz[:, 1], xyz[:, 0], ids))
        xyz, ids = xyz[order], ids[order]
        edges = np.searchsorted(ids, np.arange(CHUNK_COUNT + 1))
        changed = []
        for slot in range(CHUNK_COUNT):
            points = xyz[edges[slot]:edges[slot + 1]]
            if reset or not np.array_equal(points, self.chunks[slot]):
                self.chunks[slot] = points.copy()
                changed.append((slot, self.chunks[slot]))
        self.metadata = metadata
        return changed
