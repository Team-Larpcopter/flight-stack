"""Sample certified SUPER curves for the simulation executive or a shadow topic.

No PX4 publisher and no arming calls. Hardware command ownership remains outside
this adapter until the executive's simulation-only contract is expanded.
"""
import math
import time
from .polynomial import Trajectory, seconds


def local_ned(vector, heading, origin=(0.,0.,0.)):
    """Map FLU to NED; heading is map +X heading clockwise from PX4 north."""
    x,y,z=vector;c,s=math.cos(heading),math.sin(heading)
    return (origin[0]+c*x+s*y,origin[1]+s*x-c*y,origin[2]-z)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from dronehacks_msgs.msg import CommittedTrajectory,NavigationGoal
    from flight_executive_msgs.msg import TrajectorySetpoint
    from std_msgs.msg import Bool
    rclpy.init();node=Node('autonomy_trajectory_bridge')
    simulation=bool(node.declare_parameter('simulation_only',False).value)
    alignment=bool(node.declare_parameter('alignment_verified',False).value)
    heading=float(node.declare_parameter('map_heading_ned',0.).value)
    origin=tuple(node.declare_parameter('map_origin_ned',[0.,0.,0.]).value)
    if len(origin)!=3 or not all(math.isfinite(x) for x in (*origin,heading)):
        raise ValueError('Invalid map to PX4 alignment')
    # Fixed shadow destination prevents accidental remapping through a parameter.
    topic='/larper/trajectory_in' if simulation else '/autonomy/shadow/trajectory_in'
    if simulation and not alignment:raise ValueError('Simulation output needs verified map/PX4 alignment')
    pub=node.create_publisher(TrajectorySetpoint,topic,4)
    state={'goal':None,'curve':None,'health':False,'health_at':-math.inf,'sequence':-1,'clock':None}
    def now():return node.get_clock().now().nanoseconds*1e-9
    def goal(m):
        if m.cancel or m.header.frame_id!='map':
            state.update(goal=None,curve=None,sequence=-1);return
        old=state['goal']
        if old is None or (old.goal_id,old.localization_epoch)!=(m.goal_id,m.localization_epoch):
            state.update(curve=None,sequence=-1)
        state['goal']=m
    def health(m):state.update(health=m.data,health_at=time.monotonic())
    def receive(m):
        g=state['goal']
        if g is None:return
        try:
            curve=Trajectory.decode(m,now(),g.goal_id,g.localization_epoch)
            if curve.sequence<state['sequence']:raise ValueError('Out-of-order trajectory')
            state.update(curve=curve,sequence=curve.sequence)
        except ValueError:
            state['curve']=None
    def tick():
        stamp=now();previous=state['clock'];state['clock']=stamp
        if previous is not None and stamp<previous:
            state.update(curve=None,goal=None,sequence=-1);return
        curve=state['curve'];g=state['goal']
        if (curve is None or g is None or not state['health']
                or time.monotonic()-state['health_at']>.3
                or not -.005<=stamp-curve.certificate<.3):return
        p,v,a,yaw,rate=curve.sample(stamp)
        if not all(math.isfinite(x) for x in (*p,*v,*a,yaw,rate)):return
        out=TrajectorySetpoint();out.header.frame_id='local_ned'
        out.header.stamp=node.get_clock().now().to_msg()
        for field,values in ((out.position,local_ned(p,heading,origin)),
                             (out.velocity,local_ned(v,heading)),(out.acceleration,local_ned(a,heading))):
            field.x,field.y,field.z=values
        out.yaw_valid=True;out.yaw=math.atan2(math.sin(heading-yaw),math.cos(heading-yaw));out.yawspeed=-rate
        out.trajectory_id=f'{g.localization_epoch}:{g.goal_id}'
        target=g.pose.position
        # A stopping point en route is not mission completion.
        out.is_final=(stamp>=curve.start+curve.duration and math.dist(p,(target.x,target.y,target.z))<.05
                      and all(abs(x)<1e-6 for x in (*v,*a,rate)))
        out.certificate_required=True
        deadline=round((curve.certificate+.3)*1e9)
        out.valid_until.sec=deadline//10**9;out.valid_until.nanosec=deadline%10**9
        pub.publish(out)
    node.create_subscription(NavigationGoal,'/autonomy/goal',goal,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(CommittedTrajectory,'/autonomy/committed_trajectory',receive,4)
    node.create_subscription(Bool,'/autonomy/vio_healthy',health,qos_profile_sensor_data)
    node.create_timer(.05,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
