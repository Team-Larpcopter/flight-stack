import unittest,json,tempfile,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from grade_landing import grade_landing
class LandingTest(unittest.TestCase):
 def grade(self,tilts):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'evidence';p.write_text('\n'.join(json.dumps(['on_truth',[t,[0,0,0],0,tilt],{}]) for t,tilt in tilts))
   return grade_landing(p,{'events':[{'phase':'land','sim':1.}]})
 def test_upright_landing(self):self.assertTrue(self.grade([(1,2),(2,3),(2.5,1),(3,1)])['pass'])
 def test_inversion_cannot_be_hidden_by_final_recovery(self):self.assertFalse(self.grade([(1,170),(2,3),(2.5,1),(3,1)])['pass'])
 def test_sideways_or_missing_settle_is_not_a_pass(self):
  self.assertFalse(self.grade([(1,2),(2,45),(2.5,45),(3,45)])['pass'])
  self.assertFalse(self.grade([(1,1)])['pass'])
if __name__=='__main__':unittest.main()
