"""Publish one latched map goal in the first observed localization epoch.

A changed epoch is never automatically accepted as a new mission.
"""
import math


def goal_values(x, y, z, yaw):
    values = tuple(float(v) for v in (x, y, z, yaw))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Goal coordinates and yaw must be finite')
    return values


def main():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from dronehacks_msgs.msg import NavigationGoal, MapSnapshot, MapState
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('mission_goal')
    values = goal_values(*(node.declare_parameter(k, 0.0).value
                          for k in ('x', 'y', 'z', 'yaw')))
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(NavigationGoal, '/autonomy/goal', latched)
    epoch = None
    published = False
    wait_for_hold = node.declare_parameter('wait_for_hold', False).value
    ready = not wait_for_hold

    def publish():
        nonlocal published
        if published or epoch is None or not ready:
            return
        published = True
        goal = NavigationGoal()
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.localization_epoch, goal.goal_id = epoch, 1
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = values[:3]
        goal.pose.orientation.w = math.cos(values[3] / 2)
        goal.pose.orientation.z = math.sin(values[3] / 2)
        publisher.publish(goal)
        node.get_logger().info(f'Published mission goal in localization epoch {epoch}')

    def receive(message):
        nonlocal epoch
        if epoch is None and message.localization_epoch != 0:
            epoch = message.localization_epoch
        publish()

    def executive(message):
        nonlocal ready
        if wait_for_hold:
            ready = message.data == 'HOLD'
            publish()

    if wait_for_hold:
        node.create_subscription(String, '/flight_executive/state', executive, latched)
    node.create_subscription(MapSnapshot, '/autonomy/map_snapshot', receive, latched)
    node.create_subscription(MapState, '/autonomy/map_state', receive, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
