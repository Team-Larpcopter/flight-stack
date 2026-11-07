#!/usr/bin/env python3
"""Camera delivery delays must not change presentation time or couple streams."""
import sys,tempfile,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import cv2,numpy as np
from video_timeline import nearest_index,RecordedVideo

class TimelineTest(unittest.TestCase):
 def test_uniform_playback_uses_capture_time(self):
  stamps=np.array([0.,.067,.133,.2,.4,.467])
  selected=[stamps[nearest_index(stamps,t)] for t in np.arange(0,.47,1/15)]
  np.testing.assert_allclose(selected,[0,.067,.133,.2,.2,.4,.4,.467])
 def test_duplicate_legacy_storage_does_not_add_duration(self):
  stamps=np.array([0.,.1,.1,.1,.1,.5,.6])
  timeline=np.arange(0,.601,.1)
  self.assertEqual(len(timeline),7)
  self.assertEqual(stamps[nearest_index(stamps,.5)],.5)
  self.assertEqual(stamps[nearest_index(stamps,.6)],.6)
 def test_video_streams_selected_independently(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'camera.mp4'
   w=cv2.VideoWriter(str(path),cv2.VideoWriter_fourcc(*'mp4v'),15,(64,48))
   for value in (20,100,220):w.write(np.full((48,64,3),value,np.uint8))
   w.release();s=RecordedVideo(path,[1.,1.1,1.8])
   stamp,frame=s.at(1.12);self.assertEqual(stamp,1.1);self.assertAlmostEqual(float(frame.mean()),100,delta=6)
   stamp,frame=s.at(1.8);self.assertEqual(stamp,1.8);self.assertAlmostEqual(float(frame.mean()),220,delta=6)
   s.close()
if __name__=='__main__':unittest.main()
