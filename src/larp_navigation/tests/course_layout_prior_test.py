"""Check the shipped November layout in the planner's registered voxel frame."""
import hashlib
import json
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'tools'))
from update_course_prior import load_prior, update


class CourseLayoutPriorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.maps = []
        for name in ('course_cad_prior', 'course_cad_prior_fine'):
            path = PACKAGE/'config'/f'{name}.txt'
            raw, _, cells, resolution = load_prior(path)
            ids = np.rint(cells[:, :3]/resolution-.5).astype(int)
            lo, hi = ids.min(axis=0), ids.max(axis=0)
            shape = hi-lo+1
            grid = np.full(shape, -1, dtype=np.int8)
            grid[tuple((ids-lo).T)] = cells[:, 3].astype(np.int8)
            if int(np.prod(shape)) != len(cells) or np.any(grid < 0):
                raise AssertionError('Duplicated or missing voxel centers')
            cls.maps.append((path, raw, grid, lo, resolution))

    def occupied(self, case, course_xyz):
        _, _, grid, lo, resolution = case
        east, north, up = course_xyz
        # Independent explicit course-to-training-map contract.
        point = np.array([north-4.1624, .5334-east, up-.0672])
        return int(grid[tuple(np.floor(point/resolution).astype(int)-lo)])

    def test_new_walls_block_and_old_wall_locations_are_free(self):
        for case in self.maps:
            with self.subTest(resolution=case[-1]):
                for distance in np.linspace(.15, 1.50, 20):
                    self.assertEqual(self.occupied(case, (
                        1.1175+distance/np.sqrt(2),
                        3.556+distance/np.sqrt(2), .9)), 1)
                for east in np.linspace(.20, 1.65, 20):
                    self.assertEqual(self.occupied(case, (east, 5.70, .9)), 1)
                self.assertEqual(self.occupied(case, (1.1176, 4.9, .9)), 0)
                self.assertEqual(self.occupied(case, (2.2352, 5.6, .9)), 0)

    def test_fixed_geometry_and_apertures_remain(self):
        for case in self.maps:
            with self.subTest(resolution=case[-1]):
                self.assertEqual(self.occupied(case, (1.1176, 3.5052, .9)), 1)  # W4 hinge
                self.assertEqual(self.occupied(case, (2.2352, 5.6, -.003)), 1)  # floor
                self.assertEqual(self.occupied(case, (3.3528, 4.5, .9)), 1)  # W3
                # Independent OBJ-space aperture probes from the existing CAD test.
                for north_obj in np.linspace(-.5, 1.45, 30):
                    self.assertEqual(self.occupied(case, (2.3114+3.048, north_obj+3.048, .9398)), 0)
                self.assertEqual(self.occupied(case, (3.04162669+3.048, .8+3.048, .9398)), 1)
                self.assertEqual(self.occupied(case, (-.47094+3.048, -2.2719+3.048, .762)), 0)
                self.assertEqual(self.occupied(case, (-.47094+3.048, -2.2719+3.048, .15)), 1)

    def test_metadata_and_repeat_generation(self):
        for case in self.maps:
            path, raw, _, _, resolution = case
            with self.subTest(resolution=resolution):
                meta = json.loads(path.with_suffix('.json').read_text())
                self.assertEqual(meta['map_sha256'], hashlib.sha256(raw).hexdigest())
                self.assertEqual(meta['layout_id'], 'dronehacks-2026-11-w1-w2')
                result = update(path, PACKAGE/'config/course_layout_2026_11.json', check=True)
                self.assertEqual(result['changed_cells'], 0)


if __name__ == '__main__':
    unittest.main()
