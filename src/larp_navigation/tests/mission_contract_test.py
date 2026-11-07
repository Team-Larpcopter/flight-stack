"""Controller fault/coordinate checks on isolated DDS domain 43; no commands sent."""
import json,math,os,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace as M
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition,VehicleStatus
from flight.mission_node import Mission

class Capture:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(m)

class MissionContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='43':raise RuntimeError('test requires isolated domain 43')
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        cfg=json.loads(Path('config/local_goal_bringup.json').read_text())
        self.n=Mission(cfg,math.pi/2,Path(tempfile.gettempdir())/'unused-mission-contract.json')
        for key in ('pub_mode','pub_setpoint','pub_command','pub_goal','pub_status'):setattr(self.n,key,Capture())
        self.n.st.update(20_000_000_000,25_000_000_000,time.monotonic())
        p=VehicleLocalPosition();p.xy_valid=p.z_valid=p.v_xy_valid=p.v_z_valid=p.heading_good_for_control=True
        s=VehicleStatus();s.arming_state=2;s.nav_state=14
        o=Odometry();o.header.frame_id='map';o.child_frame_id='base_link';o.header.stamp.sec=20
        o.pose.pose.position.z=1.;o.pose.pose.orientation.w=1.
        for key,value in [('status',s),('lp',p),('odom',o),('vio_ok',True)]:self.n.receive(key,value)
        self.n.vision.on_msg(15.,True);self.n.vision.on_msg(20.,True)
        self.n.phase='navigate';self.n.phase_sim=19.;self.n.sp=np.array([0.,0.,1.]);self.n.offset=np.zeros(3)
    def tearDown(self):self.n.destroy_node()
    def test_evaluation_abort_lands_without_truth_correction(self):
        with tempfile.TemporaryDirectory() as directory:
            self.n.report=Path(directory)/'mission.json'
            self.n.report.with_name('evaluation-abort.json').write_text('{}')
            before=self.n.position().copy();self.n.tick()
            self.assertEqual(self.n.phase,'land')
            self.assertIn('evaluation supervisor aborted failed test',self.n.errors)
            self.assertTrue(self.n.pub_goal.messages[-1].cancel)
            np.testing.assert_array_equal(self.n.position(),before)
    def test_stalled_navigation_lands_early(self):
        self.n.progress_anchor=self.n.position().copy();self.n.progress_wall=time.monotonic()-31.
        self.n.tick();self.assertEqual(self.n.phase,'land')
        self.assertIn('navigation made no progress',self.n.errors)
        self.assertTrue(self.n.pub_goal.messages[-1].cancel)
    def test_progress_resets_watchdog(self):
        self.n.progress_anchor=self.n.position().copy();self.n.progress_wall=0.
        self.n.odom.pose.pose.position.x=.2
        self.assertTrue(self.n.progressing(100.));self.assertEqual(self.n.progress_wall,100.)
    def test_enu_to_ned_and_exact_clock_domain(self):
        self.n.sp=np.array([1.,2.,3.]);self.n.offset=np.array([4.,5.,6.]);self.n.velocity=np.array([1.,2.,3.])
        self.n.yaw=.3;self.n.stream();m=self.n.pub_setpoint.messages[-1]
        np.testing.assert_allclose(m.position,[5.,3.,3.]);np.testing.assert_allclose(m.velocity,[1.,-2.,-3.])
        self.assertAlmostEqual(m.yaw,-.3,places=6);self.assertEqual(m.timestamp,25_000_000)
    def test_health_loss_lands_without_disarming(self):
        self.n.vio_ok=False;self.n.tick()
        self.assertEqual(self.n.phase,'land');self.assertTrue(self.n.pub_goal.messages[-1].cancel)
        self.assertEqual(self.n.pub_command.messages,[])
    def test_missing_first_curve_holds_then_aborts(self):
        self.n.tick();self.assertEqual(self.n.phase,'navigate');self.assertEqual(len(self.n.pub_setpoint.messages),1)
        self.n.phase_sim=0.;self.n.tick();self.assertEqual(self.n.phase,'land')
        self.assertIn('no certified route from launch hover',self.n.errors)
    def test_lost_certificate_never_extrapolates(self):
        self.n.trajectory_rx=time.monotonic();self.n.tick()
        self.assertEqual(self.n.phase,'land');self.assertEqual(self.n.pub_setpoint.messages,[])
    def test_fresh_trajectory_hard_limit_is_enforced(self):
        self.n.trajectory_rx=time.monotonic()
        self.n.trajectory=M(certificate=20.,sample=lambda t:([0,0,1],[.61,0,0],[0,0,0],0.,0.))
        self.n.tick();self.assertEqual(self.n.phase,'land');self.assertEqual(self.n.pub_setpoint.messages,[])
    def test_no_stream_after_autoland(self):
        self.n.landing.start(time.monotonic());self.n.landing.on_status(time.monotonic(),18,2)
        self.n.stream();self.assertEqual(self.n.pub_setpoint.messages,[])
    def test_startup_scan_preserves_position_and_defers_goal(self):
        self.n.phase='scan';self.n.phase_sim=10.;self.n.home=np.zeros(3)
        self.n.odom.pose.pose.orientation.z=1.;self.n.odom.pose.pose.orientation.w=0.
        self.n.tick();self.assertEqual(self.n.phase,'scan')
        self.assertEqual(self.n.pub_goal.messages,[])
        np.testing.assert_allclose(self.n.pub_setpoint.messages[-1].position,[0,0,-1])
        self.assertAlmostEqual(self.n.yaw,math.pi)
        self.assertLess(self.n.yaw_rate,1.)
    def test_startup_scan_allows_horizontal_drift_during_yaw(self):
        self.n.phase='scan';self.n.phase_sim=20.;self.n.home=np.zeros(3)
        self.n.odom.pose.pose.position.x=.2
        self.n.tick();self.assertEqual(self.n.phase,'scan')
        self.assertEqual(self.n.errors,[])
        self.assertEqual(len(self.n.pub_setpoint.messages),1)
        np.testing.assert_allclose(self.n.sp,[0,0,1])
    def test_scan_respects_flight_altitude_limit(self):
        self.n.phase='scan';self.n.phase_sim=20.;self.n.home=np.zeros(3)
        self.n.odom.pose.pose.position.z=1.5
        self.n.tick();self.assertEqual(self.n.phase,'land')
    def test_cad_takeoff_box_uses_directional_clearance(self):
        self.n.config['takeoff_horizontal_envelope']={'lower':[-.2,-.1],'upper':[.2,.1]}
        self.n.phase='climb';self.n.phase_sim=15.;self.n.home=np.zeros(3)
        self.n.odom.pose.pose.position.x=-.18;self.n.odom.pose.pose.position.y=.04
        self.n.tick();self.assertEqual(self.n.phase,'climb')
        self.n.odom.pose.pose.position.y=.11
        self.n.tick();self.assertEqual(self.n.phase,'land')
        self.assertIn('left declared takeoff envelope',self.n.errors)

if __name__=='__main__':unittest.main()
