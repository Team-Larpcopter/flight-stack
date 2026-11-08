"""Sample certified planner curves; only the executive owns PX4 commands.

Default output is shadow-only. Active output requires explicit hardware or
simulation selection, surveyed alignment, and a flight-stable mission ID.
"""
import math
import time
from .polynomial import Trajectory, seconds


def local_ned(vector, heading, origin=(0.,0.,0.)):
    """Map FLU to NED; heading is map +X heading clockwise from PX4 north."""
    x,y,z=vector;c,s=math.cos(heading),math.sin(heading)
    return (origin[0]+c*x+s*y,origin[1]+s*x-c*y,origin[2]-z)


class MissionGate:
    """Keep planner restarts and goal changes within one executive flight ID."""
    def __init__(self, active, mission_id):
        if active and not mission_id.strip():
            raise ValueError('Active output requires a mission_id fixed for this flight')
        self.active = active
        self.mission_id = mission_id
        self.phase = None
        self.epoch = None
        self.fault = False

    def accepts_epoch(self, epoch):
        if not self.active:
            return True
        if self.epoch is None:
            self.epoch = epoch
        if epoch != self.epoch:
            self.fault = True
        return not self.fault

    @property
    def ready(self):
        return not self.fault and (not self.active or self.phase in ('HOLD', 'TRACKING'))

    def trajectory_id(self, goal):
        return self.mission_id or f'{goal.localization_epoch}:{goal.goal_id}'


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from dronehacks_msgs.msg import CommittedTrajectory,NavigationGoal
    from flight_executive_msgs.msg import TrajectorySetpoint
    from std_msgs.msg import Bool, String
    from geometry_msgs.msg import PoseStamped
    rclpy.init();node=Node('autonomy_trajectory_bridge')
    simulation=bool(node.declare_parameter('simulation_only',False).value)
    hardware=bool(node.declare_parameter('hardware_enabled',False).value)
    if hardware and simulation:raise ValueError('Select hardware or simulation, not both')
    active=hardware or simulation
    mission_id=str(node.declare_parameter('mission_id','').value)
    gate=MissionGate(active,mission_id)
    alignment=bool(node.declare_parameter('alignment_verified',False).value)
    heading=float(node.declare_parameter('map_heading_ned',0.).value)
    origin=tuple(node.declare_parameter('map_origin_ned',[0.,0.,0.]).value)
    max_age=float(node.declare_parameter('trajectory_max_age_s',.3).value)
    if not math.isfinite(max_age) or not 0<max_age<=1.:
        raise ValueError('trajectory_max_age_s must be in (0,1]')
    auto_alignment=bool(node.declare_parameter('align_px4_to_map_on_ground',False).value)
    alignment_received=not auto_alignment
    if len(origin)!=3 or not all(math.isfinite(x) for x in (*origin,heading)):
        raise ValueError('Invalid map to PX4 alignment')
    # Fixed shadow destination prevents accidental remapping through a parameter.
    topic='/larper/trajectory_in' if active else '/autonomy/shadow/trajectory_in'
    if active and not alignment:raise ValueError('Executive output needs verified map/PX4 alignment')
    pub=node.create_publisher(TrajectorySetpoint,topic,4)
    state={'goal':None,'curve':None,'health':False,'health_at':-math.inf,'sequence':-1,'clock':None,'received_at':-math.inf}
    def now():return node.get_clock().now().nanoseconds*1e-9
    def align(m):
        nonlocal origin,heading,alignment_received
        if not auto_alignment or m.header.frame_id!='local_ned':return
        p,q=m.pose.position,m.pose.orientation
        values=(p.x,p.y,p.z,q.w,q.x,q.y,q.z)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1)>.002:return
        new_origin=(p.x,p.y,p.z)
        new_heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if alignment_received:
            changed = (any(abs(a-b)>1e-6 for a,b in zip(origin,new_origin)) or
                       abs(math.atan2(math.sin(heading-new_heading),math.cos(heading-new_heading)))>1e-6)
            if not changed:return
            # Executive republishes ground registration after PX4 startup resets.
            # Never mix an already active flight with a different NED frame.
            if gate.phase not in (None, 'WAITING', 'TAKEOFF'):
                gate.fault=True
                return
            state['curve']=None
        origin,heading,alignment_received=new_origin,new_heading,True
    if auto_alignment:
        node.create_subscription(PoseStamped,'/autonomy/map_to_px4',align,
                                 QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def executive(m):
        if not active:return
        gate.phase=m.data
        if m.data not in ('HOLD','TRACKING'):state['curve']=None
    node.create_subscription(String,'/flight_executive/state',executive,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def goal(m):
        if m.cancel or m.header.frame_id!='map':
            state.update(goal=None,curve=None,sequence=-1);return
        if not gate.accepts_epoch(m.localization_epoch):
            state.update(goal=None,curve=None);return
        old=state['goal']
        if old is None or (old.goal_id,old.localization_epoch)!=(m.goal_id,m.localization_epoch):
            state.update(curve=None,sequence=-1)
        state['goal']=m
    def health(m):state.update(health=m.data,health_at=time.monotonic())
    def receive(m):
        g=state['goal']
        if g is None or not gate.ready:return
        # An older queued packet must not erase a newer accepted trajectory.
        if m.sequence<state['sequence']:return
        try:
            curve=Trajectory.decode(m,now(),g.goal_id,g.localization_epoch,None)
            if curve.sequence<state['sequence']:raise ValueError('Out-of-order trajectory')
            state.update(curve=curve,sequence=curve.sequence,received_at=time.monotonic())
        except ValueError:
            state['curve']=None
    def tick():
        if not gate.ready or not alignment_received:return
        stamp=now();previous=state['clock'];state['clock']=stamp
        if previous is not None and stamp<previous:
            state.update(curve=None,goal=None,sequence=-1);return
        curve=state['curve'];g=state['goal']
        if curve is None or g is None:return
        # Producer heartbeat, independent of sensor clocks/certificate lifetime.
        if time.monotonic()-state['received_at']>1.:return
        p,v,a,yaw,rate=curve.sample(stamp)
        if not all(math.isfinite(x) for x in (*p,*v,*a,yaw,rate)):return
        out=TrajectorySetpoint();out.header.frame_id='local_ned'
        out.header.stamp=node.get_clock().now().to_msg()
        for field,values in ((out.position,local_ned(p,heading,origin)),
                             (out.velocity,local_ned(v,heading)),(out.acceleration,local_ned(a,heading))):
            field.x,field.y,field.z=values
        out.yaw_valid=True;out.yaw=math.atan2(math.sin(heading-yaw),math.cos(heading-yaw));out.yawspeed=-rate
        out.trajectory_id=gate.trajectory_id(g)
        target=g.pose.position
        # A stopping point en route is not mission completion.
        out.is_final=(stamp>=curve.start+curve.duration and math.dist(p,(target.x,target.y,target.z))<.05
                      and all(abs(x)<1e-6 for x in (*v,*a,rate)))
        out.certificate_required=False
        deadline=round((curve.certificate+max_age)*1e9)
        out.valid_until.sec=deadline//10**9;out.valid_until.nanosec=deadline%10**9
        pub.publish(out)
    node.create_subscription(NavigationGoal,'/autonomy/goal',goal,
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(CommittedTrajectory,'/autonomy/committed_trajectory',receive,1)
    node.create_subscription(Bool,'/autonomy/vio_healthy',health,qos_profile_sensor_data)
    node.create_timer(.02,tick)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
