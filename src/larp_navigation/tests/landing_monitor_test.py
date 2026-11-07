"""Slow simulation must wait for fresh landing evidence, never waive it."""
import unittest
from gps_denied.mission_common import LandingMonitor


class LandingEvidenceTest(unittest.TestCase):
    def monitor(self):
        m=LandingMonitor(timeout_s=20.)
        m.start(0.);m.on_status(1.,18,2);m.on_land_detected(4.,True)
        m.on_status(5.,18,1)
        return m

    def test_slow_status_waits_then_confirms(self):
        m=self.monitor()
        self.assertIsNone(m.outcome(8.))
        m.on_land_detected(8.5,True)
        self.assertIsNone(m.outcome(9.))
        m.on_status(9.5,18,1)
        self.assertTrue(m.outcome(9.5)['confirmed'])

    def test_silent_status_times_out(self):
        m=self.monitor()
        self.assertFalse(m.outcome(20.)['confirmed'])

    def test_contradictory_landed_report_fails(self):
        m=self.monitor();m.on_land_detected(7.,False)
        self.assertFalse(m.outcome(8.)['confirmed'])

    def test_rearmed_fails(self):
        m=self.monitor();m.on_status(7.,18,2)
        self.assertFalse(m.outcome(8.)['confirmed'])


if __name__=='__main__':unittest.main()
