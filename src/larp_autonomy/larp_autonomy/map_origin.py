"""Freeze one stable map origin before any public mission odometry or map update."""
from dataclasses import dataclass
import math
import time
import numpy as np


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


@dataclass(frozen=True)
class Origin:
    position: tuple
    yaw: float


class StableOrigin:
    def __init__(self):
        self.value = None
        self.reset_pending()

    def reset_pending(self):
        self.candidate = None
        self.since = self.last = None

    def observe(self, now, position, heading):
        if self.value is not None:
            return self.value
        if not all(math.isfinite(v) for v in (*position, heading, now)):
            self.reset_pending()
            return None
        if (self.candidate is None or now < self.last or now-self.last > .3 or
                math.dist(position, self.candidate.position) > .05 or
                abs(wrap(heading-self.candidate.yaw)) > .05):
            self.candidate, self.since = Origin(tuple(position), heading), now
        self.last = now
        if now-self.since >= 1.:
            self.value = Origin(tuple(position), heading)
        return self.value


class Registration:
    def __init__(self, origin, target, target_yaw):
        self.origin = np.asarray(origin.position)
        self.target = np.asarray(target)
        self.angle = wrap(target_yaw-origin.yaw)
        c, s = math.cos(self.angle), math.sin(self.angle)
        self.rotation = np.array([[c,-s,0.], [s,c,0.], [0.,0.,1.]])
        self.rotation6 = np.zeros((6,6))
        self.rotation6[:3,:3] = self.rotation6[3:,3:] = self.rotation

    def position(self, point):
        return self.target+self.rotation@(np.asarray(point)-self.origin)

    def orientation(self, q):
        c,s = math.cos(self.angle/2),math.sin(self.angle/2)
        w,x,y,z = q
        return (c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w)

    def covariance(self, values):
        return (self.rotation6@np.asarray(values).reshape(6,6)@self.rotation6.T).ravel().tolist()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from px4_msgs.msg import VehicleStatus, VehicleLandDetected
    from flight_executive.flight_executive_node import topic_name

    class MapOrigin(Node):
        def __init__(self):
            super().__init__('stable_map_origin')
            self.hardware = self.declare_parameter('hardware_enabled', False).value
            self.target = tuple(self.declare_parameter('initial_position_m', [0.,0.,0.]).value)
            self.target_yaw = self.declare_parameter('initial_yaw_rad', 0.).value
            self.system = self.declare_parameter('target_system', 1).value
            self.component = self.declare_parameter('target_component', 1).value
            if len(self.target)!=3 or not all(math.isfinite(v) for v in (*self.target,self.target_yaw)):
                raise ValueError('Invalid initial map pose')
            self.anchor = StableOrigin()
            self.registration = None
            self.origin_stamp = None
            self.aligned = not self.hardware
            self.pose = self.status = self.landed = None
            self.pose_at = self.status_at = self.landed_at = self.health_at = -math.inf
            self.healthy = False
            self.last_stamp = -1
            self.last_health_publish = -math.inf
            self.last_wait_log = -math.inf
            latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.output = self.create_publisher(Odometry,'/autonomy/odometry',10)
            self.health = self.create_publisher(Bool,'/autonomy/vio_healthy',10)
            self.map_ready = self.create_publisher(Bool,'/autonomy/map_update_ready',10)
            self.origin_pub = self.create_publisher(PoseStamped,'/autonomy/map_origin',latched)
            self.create_subscription(Odometry,'/autonomy/unregistered_odometry',self.receive_pose,qos_profile_sensor_data)
            self.create_subscription(Bool,'/autonomy/unregistered_vio_healthy',self.receive_health,qos_profile_sensor_data)
            if self.hardware:
                self.create_subscription(VehicleStatus,topic_name('/fmu','out/vehicle_status',VehicleStatus),self.receive_status,qos_profile_sensor_data)
                self.create_subscription(VehicleLandDetected,'/fmu/out/vehicle_land_detected',self.receive_landed,qos_profile_sensor_data)
                self.create_subscription(PoseStamped,'/autonomy/map_to_px4',self.receive_alignment,latched)
            self.create_timer(.02,self.tick)
            self.get_logger().info('Waiting for stable ground odometry; mission odometry and map updates withheld')

        def receive_pose(self, msg):
            if msg.header.frame_id=='vio_odom' and msg.child_frame_id=='base_link':
                self.pose,self.pose_at=msg,time.monotonic()

        def receive_health(self, msg):
            self.healthy,self.health_at=msg.data,time.monotonic()

        def receive_status(self,msg):
            if msg.system_id==self.system and msg.component_id==self.component:
                self.status,self.status_at=msg,time.monotonic()

        def receive_landed(self,msg):
            self.landed,self.landed_at=msg,time.monotonic()

        def receive_alignment(self,msg):
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            # Do not accept a previous executive's transform after origin-node restart.
            if self.registration is not None and msg.header.frame_id=='local_ned' and stamp>=self.origin_stamp:
                self.aligned=True
                self.get_logger().info('Shared origin and PX4 alignment ready; dynamic map updates enabled')

        def tick(self):
            now=time.monotonic()
            clock=self.get_clock().now().nanoseconds
            m=self.pose
            stamp=-1 if m is None else m.header.stamp.sec*10**9+m.header.stamp.nanosec
            good=(m is not None and self.healthy and 0<=now-self.health_at<.3 and
                  0<=now-self.pose_at<.3 and -5_000_000<=clock-stamp<300_000_000)
            numeric_valid=False
            if m is not None:
                p,q,v,w=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist.linear,m.twist.twist.angular
                numeric_valid=all(math.isfinite(x) for x in (p.x,p.y,p.z,q.w,q.x,q.y,q.z,v.x,v.y,v.z,w.x,w.y,w.z))
                numeric_valid=numeric_valid and abs(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z-1)<.002
            good=good and numeric_valid
            if self.registration is None:
                grounded=(not self.hardware or (self.status is not None and self.landed is not None and
                    0<=now-self.status_at<1. and 0<=now-self.landed_at<2.5 and
                    self.status.arming_state==VehicleStatus.ARMING_STATE_DISARMED and
                    self.status.nav_state!=VehicleStatus.NAVIGATION_STATE_OFFBOARD and
                    not self.status.failsafe and self.landed.landed))
                stationary=numeric_valid
                if now-self.last_wait_log>=3.:
                    reason=('fresh healthy odometry' if not good else
                            'disarmed, landed PX4 telemetry' if not grounded else
                            'stationary odometry' if not stationary else 'one-second stable origin window')
                    self.get_logger().info('Map origin waiting for '+reason)
                    self.last_wait_log=now
                if not grounded or not stationary:
                    self.anchor.reset_pending()
                else:
                    origin=Origin((p.x,p.y,p.z),yaw(q))
                    self.anchor.value=origin
                    if origin is not None:
                        self.registration=Registration(origin,self.target,self.target_yaw)
                        self.origin_stamp=clock
                        view=PoseStamped();view.header.frame_id='vio_odom'
                        view.header.stamp=self.get_clock().now().to_msg()
                        view.pose.position.x,view.pose.position.y,view.pose.position.z=origin.position
                        view.pose.orientation.w=math.cos(origin.yaw/2);view.pose.orientation.z=math.sin(origin.yaw/2)
                        self.origin_pub.publish(view)
                        self.get_logger().info(f'Shared map origin frozen: source={origin.position}, yaw={origin.yaw:.4f}; start={self.target}, yaw={self.target_yaw:.4f}')
            ready=good and self.registration is not None
            # After origin initialization, forward each new valid pose even when
            # health/age diagnostics are false; preserve its original timestamp.
            if self.registration is not None and numeric_valid and stamp>self.last_stamp:
                # This callback owns m; replace only pose/world covariance. Twist stays body FLU.
                self.last_stamp=stamp
                out=Odometry()
                out.header.stamp=m.header.stamp;out.header.frame_id='map';out.child_frame_id='base_link'
                out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=self.registration.position((p.x,p.y,p.z)).tolist()
                qout=out.pose.pose.orientation
                qout.w,qout.x,qout.y,qout.z=self.registration.orientation((q.w,q.x,q.y,q.z))
                out.pose.covariance=self.registration.covariance(m.pose.covariance)
                out.twist=m.twist
                self.output.publish(out)
            if now-self.last_health_publish>=.1:
                self.last_health_publish=now
                self.health.publish(Bool(data=bool(ready)))
                self.map_ready.publish(Bool(data=bool(ready and self.aligned)))

    rclpy.init();node=MapOrigin()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
