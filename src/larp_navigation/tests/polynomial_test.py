import math,unittest
from types import SimpleNamespace as M
from flight.polynomial import Piece,Trajectory

def stamp(t):return M(sec=int(t),nanosec=int((t-int(t))*1e9))
def message():
    # x: quintic minimum-jerk motion 0 -> 1 over two seconds.
    coeff=[.1875,-.9375,1.25,0.,0.,0.]
    return M(VALID=1,status=1,header=M(frame_id='map',stamp=stamp(10.)),start_time=stamp(10.),
      localization_epoch=1,goal_id=3,sequence=1,
      position=[M(duration=2.,degree=5,coefficients=coeff+[0.]*12)],
      yaw=[M(duration=2.,degree=1,coefficients=[0.,.3])])

class PolynomialTest(unittest.TestCase):
    def test_exact_position_and_derivatives(self):
        t=Trajectory.decode(message(),10.,3)
        p,v,a,y,yr=t.sample(11.)
        self.assertAlmostEqual(p[0],.5);self.assertAlmostEqual(v[0],.9375)
        self.assertAlmostEqual(a[0],0.);self.assertAlmostEqual(y,.3);self.assertEqual(yr,0.)
        self.assertEqual(t.sample(14.)[:3],((1.,0.,0.),(0.,0.,0.),(0.,0.,0.)))
    def test_refuses_wrong_epoch_stale_and_nonfinite(self):
        for field,value in [('localization_epoch',2),('goal_id',2),('status',0)]:
            m=message();setattr(m,field,value)
            with self.assertRaises(ValueError):Trajectory.decode(m,10.,3)
        with self.assertRaises(ValueError):Trajectory.decode(message(),10.5,3)
        m=message();m.position[0].coefficients[0]=math.nan
        with self.assertRaises(ValueError):Trajectory.decode(m,10.,3)
    def test_no_unstoppable_trajectory_or_bad_wire_size(self):
        m=message();m.position[0].coefficients[-2]=1.
        with self.assertRaises(ValueError):Trajectory.decode(m,10.,3)
        with self.assertRaises(ValueError):Piece.decode(M(duration=1.,degree=2,coefficients=[1.,2.]),3)
    def test_piece_boundaries_require_acceleration_continuity(self):
        m=message();m.position.append(M(duration=1.,degree=2,coefficients=[1.,0.,1.]+[0.]*6))
        with self.assertRaisesRegex(ValueError,'discontinuous'):Trajectory.decode(m,10.,3)

if __name__=='__main__':unittest.main()
