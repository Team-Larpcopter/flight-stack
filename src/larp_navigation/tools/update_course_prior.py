#!/usr/bin/env python3
"""Transfer the November W1/W2 layout into the existing registered voxel priors.

This deliberately edits only the old/new panel volumes. A PLY point cloud does
not provide the registered origin or observed-free volume expected by this map.
The baseline hash pins the CAD assumptions; unrelated obstacle and free cells
are retained byte-for-byte. Cell/solid overlap conservatively covers full voxels.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def solid_mask(centers, resolution, solid, transform):
    """Closed oriented rectangular prism versus axis-aligned voxel SAT."""
    a = np.asarray(solid['a'], dtype=float)
    if 'b' in solid:
        b = np.asarray(solid['b'], dtype=float)
    else:
        heading = np.deg2rad(solid['heading_from_east_degrees'])
        b = a + solid['length_m'] * np.array([np.cos(heading), np.sin(heading)])
    rotation = np.asarray(transform['rotation'], dtype=float)
    translation = np.asarray(transform['translation_m'], dtype=float)
    # This fixture is z-up and applies only a yaw and translation.
    if not (np.allclose(rotation.T @ rotation, np.eye(3)) and
            np.allclose(rotation[2], [0, 0, 1]) and np.linalg.det(rotation) > 0):
        raise ValueError('Expected a rigid z-up map registration')
    a = (rotation @ np.r_[a, 0] + translation)[:2]
    b = (rotation @ np.r_[b, 0] + translation)[:2]
    length = np.linalg.norm(b-a)
    if length <= 0 or solid['thickness_m'] <= 0:
        raise ValueError('Invalid panel dimensions')
    u = (b-a)/length
    n = np.array([-u[1], u[0]])
    center = (a+b)/2
    half_length, half_width, h = length/2, solid['thickness_m']/2, resolution/2
    extent = half_length*np.abs(u) + half_width*np.abs(n)
    delta = centers[:, :2]-center
    eps = 1e-9
    lo_z, hi_z = np.asarray(solid['z_m']) + translation[2]
    candidate = ((np.abs(delta[:, 0]) <= extent[0]+h+eps) &
                 (np.abs(delta[:, 1]) <= extent[1]+h+eps) &
                 (centers[:, 2]+h >= lo_z-eps) &
                 (centers[:, 2]-h <= hi_z+eps))
    ids = np.flatnonzero(candidate)
    d = delta[ids]
    candidate[ids] &= ((np.abs(d@u) <= half_length+h*np.abs(u).sum()+eps) &
                       (np.abs(d@n) <= half_width+h*np.abs(n).sum()+eps))
    return candidate


def edit_masks(centers, resolution, layout):
    transform = layout['map_from_course']
    masks = {
        kind: {name: solid_mask(centers, resolution, solid, transform)
               for name, solid in layout[kind].items()}
        for kind in ('old_walls', 'new_walls', 'shared_fixed_solids')
    }
    union = lambda kind: np.logical_or.reduce(list(masks[kind].values()))
    old, new, fixed = (union(k) for k in ('old_walls', 'new_walls', 'shared_fixed_solids'))
    return old, new, fixed, masks


def load_prior(path):
    raw = path.read_bytes()
    lines = raw.splitlines(keepends=True)
    magic, resolution = lines[0].decode().split()
    resolution = float(resolution)
    if magic != 'CAD_PRIOR_V1' or resolution not in (.05, .1):
        raise ValueError('Unexpected prior format/resolution')
    data = np.loadtxt(io.BytesIO(raw), skiprows=1)
    if data.ndim != 2 or data.shape[1] != 4 or not np.isfinite(data).all():
        raise ValueError('Invalid prior cells')
    if not np.isin(data[:, 3], [0, 1]).all():
        raise ValueError('Invalid cell classification')
    if not np.allclose(data[:, :3]/resolution-.5,
                       np.rint(data[:, :3]/resolution-.5), atol=1e-8, rtol=0):
        raise ValueError('Prior is not on the voxel-center lattice')
    return raw, lines, data, resolution


def update(path, layout_path, check=False):
    layout_raw = layout_path.read_bytes()
    layout = json.loads(layout_raw)
    raw, lines, data, resolution = load_prior(path)
    meta_path = path.with_suffix('.json')
    meta = json.loads(meta_path.read_text())
    current_digest = sha256(raw)
    baseline_digest = layout['base_map_sha256'][str(resolution)]
    baseline = current_digest == baseline_digest
    if not baseline and not (meta.get('layout_id') == layout['layout_id'] and
                             meta.get('map_sha256') == current_digest and
                             meta.get('source_sha256') == sha256(layout_raw)):
        raise ValueError('Input is neither the pinned baseline nor this verified layout')
    old, new, fixed, masks = edit_masks(data[:, :3], resolution, layout)
    affected = old | new
    retained = b''.join(line for line, touched in zip(lines[1:], affected) if not touched)
    retained_digest = sha256(retained)
    if not baseline and retained_digest != meta['unchanged_cells_sha256']:
        raise ValueError('Cells outside the layout edit have changed')
    if baseline:
        for name, mask in masks['old_walls'].items():
            if int(mask.sum()) != meta['group_cells'][name] or not np.all(data[mask, 3] == 1):
                raise ValueError(f'Baseline geometry does not match {name}')
        base_meta = {k: meta[k] for k in ('source', 'source_sha256', 'map_sha256')}
    else:
        base_meta = meta['base_prior']

    occupancy = data[:, 3].astype(np.uint8)
    occupancy[old & ~fixed] = 0
    occupancy[new] = 1
    changed = np.flatnonzero(occupancy != data[:, 3])
    # Keep header, lattice, record order, and every untouched row exactly.
    for i in changed:
        line = lines[i+1]
        if not line.endswith((b' 0\n', b' 1\n')):
            raise ValueError('Unexpected baseline line formatting')
        lines[i+1] = line[:-2] + str(int(occupancy[i])).encode() + b'\n'
    result = b''.join(lines)
    counts = dict(meta['group_cells'])
    counts.update({name: int(mask.sum()) for name, mask in masks['new_walls'].items()})
    result_meta = {
        'source': 'config/'+layout_path.name,
        'source_sha256': sha256(layout_raw),
        'map_sha256': sha256(result),
        'resolution': resolution,
        'layout_id': layout['layout_id'],
        'base_prior': base_meta,
        'group_cells': counts,
        'alignment': meta['alignment'],
        'semantics': 'CAD-derived free and occupied prior with schematic W1/W2 placement; sensor-revisable; immutable net/geofence separate',
        'unchanged_cells_sha256': retained_digest,
        'edit_volume_cells': int(affected.sum()),
        'preserved_cells': int((~affected).sum()),
        'occupied_cells': int(occupancy.sum()),
        'registration': layout['map_from_course'],
        'placement_source': layout['placement_source'],
    }
    meta_text = json.dumps(result_meta, indent=2)+'\n'
    if check:
        if result != raw or meta_text != meta_path.read_text():
            raise ValueError(f'{path.name} needs the layout update')
    else:
        path.write_bytes(result)
        meta_path.write_text(meta_text)
    return {'file': path.name, 'resolution': resolution, 'changed_cells': len(changed),
            'preserved_cells': int((~affected).sum()), 'map_sha256': sha256(result)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', type=Path, required=True)
    parser.add_argument('--check', action='store_true', help='Check reproducibility without writing')
    args = parser.parse_args()
    layout = args.config_dir/'course_layout_2026_11.json'
    for name in ('course_cad_prior.txt', 'course_cad_prior_fine.txt'):
        print(json.dumps(update(args.config_dir/name, layout, args.check)))


if __name__ == '__main__':
    main()
