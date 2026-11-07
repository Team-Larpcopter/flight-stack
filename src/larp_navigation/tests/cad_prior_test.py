"""Check CAD rasterization and independent supplied aperture probes."""
import json,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from build_cad_prior import overlaps

class CadPriorTest(unittest.TestCase):
    def test_triangle_crosses_cell_without_contained_vertex(self):
        tri=np.array([[-2.,-2.,0],[2.,-2.,0],[0,2.,0]])
        self.assertTrue(overlaps(tri,np.array([[0.,0.,0.]]),.05)[0])
        self.assertFalse(overlaps(tri,np.array([[0.,0.,.2]]),.05)[0])
    def test_supplied_tunnel_and_window_probes(self):
        a=np.loadtxt('config/course_cad_prior.txt',skiprows=1)
        cells={tuple(np.floor(p[:3]/.1+1e-7).astype(int)):int(p[3]) for p in a}
        def occupied(p):
            q=np.array([p[1]-1.1144,-p[0]-2.5146,p[2]-.0672])
            return cells[tuple(np.floor(q/.1).astype(int))]
        for y in np.linspace(-.5,1.45,30):self.assertEqual(occupied([2.3114,y,.9398]),0)
        self.assertEqual(occupied([3.04162669,.8,.9398]),1)
        self.assertEqual(occupied([-.47094,-2.2719,.762]),0)
        self.assertEqual(occupied([-.47094,-2.2719,.15]),1)

if __name__=='__main__':unittest.main()
