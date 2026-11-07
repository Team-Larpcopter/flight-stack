import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from grade_course import grade_samples
class CourseGradeTest(unittest.TestCase):
 def test_direct_finish_does_not_count(self):
  r=grade_samples([(0,-2.5146,1.1144,.0672),(1,-2.04216,-.79248,.0672)],.28,True)
  self.assertFalse(r['pass']);self.assertEqual(r['checkpoints'],[])
 def test_ordered_gate_route_and_landing(self):
  s=[(0,-2.5146,1.1144,1),(1,-1.5,2.6,1),(2,.7,2.6,1),(3,2.3,-.7,1),
     (4,.5,-.1,1),(5,0,-.1,1),(6,-.3,-.1,1),(7,-2.04216,-.79248,.0672)]
  self.assertTrue(grade_samples(s,.28,True)['pass'])
  self.assertFalse(grade_samples(s,.28,False)['pass'])
  s.insert(3,(2.5,1,2.6,2.0));self.assertFalse(grade_samples(s,.28,True)['pass'])
 def test_rounded_divider_crossing_requires_aperture_and_full_body(self):
  s=[(0,-2.1,2.6,1),(1,-1.88,2.6,1),(2,-1.8,2.55,1),(3,-1.5,2.3,1)]
  self.assertEqual(len(grade_samples(s,.28,False)['checkpoints']),1)
  self.assertEqual(grade_samples(s[:-1],.28,False)['checkpoints'],[])
  # Crossing below the divider tip cannot earn the checkpoint.
  bad=[(t,x,2.2,z) for t,x,y,z in s]
  self.assertEqual(grade_samples(bad,.28,False)['checkpoints'],[])
 def test_pad_requires_whole_body(self):
  s=[(0,-1.5,2.6,1),(1,.7,2.6,1),(2,2.3,-.7,1),(3,.5,-.1,1),(4,-.3,-.1,1),(5,-1.4,-.79248,.0672)]
  self.assertFalse(grade_samples(s,.28,True)['pass'])
 def test_window_needs_full_aperture_clearance(self):
  sin,cos=math.sin(math.radians(20)),math.cos(math.radians(20))
  x=(90.6+31.75*sin-120)*.0254;y=(.72+31.75*cos-120)*.0254
  s=[(0,-1.5,2.6,1),(1,.7,2.6,1),(2,2.3,-.7,1),
     (3,x+.5*cos,y-.5*sin,.762),(4,x-.5*cos,y+.5*sin,.762),(5,-2.04216,-.79248,.0672)]
  r=grade_samples(s,.28,True);self.assertTrue(r['pass']);self.assertEqual(r['checkpoints'][3]['name'],'window')
  s[3]=(*s[3][:3],1.1);s[4]=(*s[4][:3],1.1);self.assertFalse(grade_samples(s,.28,True)['pass'])
if __name__=='__main__':unittest.main()
