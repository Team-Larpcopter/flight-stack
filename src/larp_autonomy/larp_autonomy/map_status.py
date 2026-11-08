"""Small map-pane observer; no cloud copying and no second map instance."""
import time


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from dronehacks_msgs.msg import MapState
    from std_msgs.msg import Bool
    rclpy.init()
    node=rclpy.create_node('map_status')
    planner=node.declare_parameter('planner','super').value.upper()
    state={'revision':0,'received':None,'ready':False}
    def receive(msg):
        state.update(revision=msg.revision,received=time.monotonic())
    def ready(msg): state['ready']=msg.data
    def report():
        age='waiting' if state['received'] is None else f'{time.monotonic()-state["received"]:.2f}s'
        node.get_logger().info(f'ROG shared with {planner} | revision={state["revision"]} age={age} takeoff_ready={state["ready"]}')
    node.create_subscription(MapState,'/autonomy/map_state',receive,qos_profile_sensor_data)
    node.create_subscription(Bool,'/autonomy/takeoff_ready',ready,qos_profile_sensor_data)
    node.create_timer(1.,report)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
