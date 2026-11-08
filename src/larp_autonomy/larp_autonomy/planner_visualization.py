"""Bounded, throttled planner views under stack fox's /viz whitelist."""
from collections import deque
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from dronehacks_msgs.msg import MapSnapshot, RoutePolyline, NavigationGoal
    from visualization_msgs.msg import Marker, MarkerArray
    from .voxel_delta import CHUNK_COUNT, VoxelDelta, cloud_xyz, hide_boundary_cells
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import PoseStamped, Point
    from std_msgs.msg import Bool
    rclpy.init()
    node = Node('planner_visualization')
    from rcl_interfaces.msg import ParameterDescriptor
    array_param = ParameterDescriptor(dynamic_typing=True)
    boundary_lower = node.declare_parameter('hide_boundary_lower', [], array_param).value
    ceiling_z = node.declare_parameter('ceiling_cut_z', 1e6).value
    boundary_upper = node.declare_parameter('hide_boundary_upper', [], array_param).value
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # One retained writer per chunk: bridge 3.6 caches each writer separately,
    # replaying the current map to late clients without periodic full broadcasts.
    chunk_pubs = [node.create_publisher(
        MarkerArray, '/viz/autonomy/occupied_chunks', latched)
        for _ in range(CHUNK_COUNT)]
    delta = VoxelDelta()
    def publish_chunk(slot, points, header=None, resolution=.05):
        marker = Marker()
        if header is not None:
            marker.header = header
        else:
            marker.header.frame_id = 'map'
        marker.ns, marker.id = 'occupied_voxel_chunks', slot
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD if len(points) else Marker.DELETE
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = resolution * .82
        # Chunk-level palette avoids per-voxel color bandwidth and processing.
        palette = [(0.24, .52, .68), (.38, .68, .76), (.52, .60, .78), (.30, .62, .58)]
        marker.color.r, marker.color.g, marker.color.b = palette[slot % len(palette)]
        marker.color.a = 1.
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
        chunk_pubs[slot].publish(MarkerArray(markers=[marker]))
    for slot in range(CHUNK_COUNT):
        publish_chunk(slot, [])
    route_pub = node.create_publisher(Path, '/viz/autonomy/route', latched)
    plan_pub = node.create_publisher(Path, '/viz/autonomy/committed_path', 1)
    pose_pub = node.create_publisher(PoseStamped, '/viz/autonomy/body_pose', 1)
    odom_pub = node.create_publisher(Odometry, '/viz/autonomy/body_odom', 1)
    history_pub = node.create_publisher(Path, '/viz/autonomy/body_path', 1)
    health_pub = node.create_publisher(Bool, '/viz/autonomy/vio_healthy', 1)
    goal_pub = node.create_publisher(PoseStamped, '/viz/autonomy/goal_pose', latched)
    def goal_view(msg):
        pose = PoseStamped()
        pose.header, pose.pose = msg.header, msg.pose
        goal_pub.publish(pose)
    node.create_subscription(NavigationGoal, '/autonomy/goal', goal_view, latched)
    latest = {'map': None, 'route': None, 'plan': None, 'odom': None}
    history = deque(maxlen=1500)
    previous_stamp = None
    last_map = -float('inf')
    def store(name):
        def receive(msg):
            latest[name] = msg
        return receive
    node.create_subscription(MapSnapshot, '/autonomy/map_snapshot', store('map'), latched)
    node.create_subscription(RoutePolyline, '/autonomy/route', store('route'), qos_profile_sensor_data)
    node.create_subscription(Path, '/autonomy/committed_path', store('plan'), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/autonomy/odometry', store('odom'), qos_profile_sensor_data)
    node.create_subscription(Bool, '/autonomy/vio_healthy', health_pub.publish, qos_profile_sensor_data)
    def tick():
        nonlocal last_map, previous_stamp
        stamp = time.monotonic()
        if latest['map'] is not None and stamp-last_map >= 1.0:
            snapshot = latest['map']
            try:
                changed = delta.update(
                    hide_boundary_cells(cloud_xyz(snapshot.occupied), boundary_lower,
                                        boundary_upper, snapshot.resolution, ceiling_z),
                    [snapshot.lower.x, snapshot.lower.y, snapshot.lower.z],
                    [snapshot.upper.x, snapshot.upper.y, snapshot.upper.z],
                    snapshot.resolution, snapshot.header.frame_id,
                    snapshot.localization_epoch)
                for slot, points in changed:
                    publish_chunk(slot, points, snapshot.header, snapshot.resolution)
            except (ValueError, TypeError) as error:
                node.get_logger().warning(f'Skipping invalid visualization map: {error}')
            latest['map'] = None
            last_map = stamp
        route = latest['route']
        if route is not None:
            path = Path()
            path.header = route.header
            if route.status == route.VALID:
                for point in route.points:
                    p = PoseStamped()
                    p.header = route.header
                    p.pose.position = point
                    p.pose.orientation.w = 1.0
                    path.poses.append(p)
            route_pub.publish(path)
            latest['route'] = None
        if latest['plan'] is not None:
            plan_pub.publish(latest['plan'])
            latest['plan'] = None
        odom = latest['odom']
        if odom is not None:
            p = PoseStamped()
            p.header, p.pose = odom.header, odom.pose.pose
            pose_pub.publish(p)
            odom_pub.publish(odom)
            key = (p.header.stamp.sec, p.header.stamp.nanosec)
            if key != previous_stamp:
                history.append(p)
                previous_stamp = key
            path = Path()
            path.header, path.poses = p.header, list(history)
            history_pub.publish(path)
            latest['odom'] = None
    node.create_timer(.2, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
