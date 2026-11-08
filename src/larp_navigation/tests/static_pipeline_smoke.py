"""Offline ROS integration test: static geometry, live pose leases, fail closed.

Run only on an isolated ROS domain/network; starts mapper and SUPER, no hardware.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from dronehacks_msgs.msg import MapSnapshot, MapState, NavigationGoal, RoutePolyline, CommittedTrajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapper', required=True)
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '187':
        raise RuntimeError('This synthetic test requires isolated ROS_DOMAIN_ID=187')
    rclpy.init()
    node = rclpy.create_node('static_pipeline_test')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, '/navigation/odometry', qos_profile_sensor_data)
    health_pub = node.create_publisher(Bool, '/navigation/vio_healthy', 10)
    goal_pub = node.create_publisher(NavigationGoal, '/navigation/goal', latched)
    state_pub = node.create_publisher(MapState, '/navigation/map_state', qos_profile_sensor_data)
    maps, states, routes, curves = [], [], [], []
    subscriptions = [
        node.create_subscription(MapSnapshot, '/navigation/map_snapshot', maps.append, latched),
        node.create_subscription(MapState, '/navigation/map_state', states.append, qos_profile_sensor_data),
        node.create_subscription(RoutePolyline, '/navigation/route', routes.append, 20),
        node.create_subscription(CommittedTrajectory, '/navigation/trajectory', curves.append, 20),
    ]
    x, healthy = 0., True
    def inputs():
        msg = Odometry();msg.header.frame_id = 'map';msg.child_frame_id = 'base_link'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x;msg.pose.pose.position.z = 1.
        msg.pose.pose.orientation.w = 1.
        odom_pub.publish(msg);health_pub.publish(Bool(data=healthy))
    timer = node.create_timer(.02, inputs)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
    processes = []
    with tempfile.TemporaryDirectory(prefix='static-pipeline-') as directory:
        directory = Path(directory)
        config = yaml.safe_load(Path(args.config).read_text())
        # Use production geometry/clearance parameters with a small synthetic map.
        assert abs(config['rog_map']['resolution']-.1)<1e-6
        (directory/'nav.yaml').write_text(yaml.safe_dump(config))
        identity = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        (directory/'rig.yaml').write_text(yaml.safe_dump({'T_body_imu':identity,'T_imu_left':identity}))
        with (directory/'cad.txt').open('w') as f:
            f.write('CAD_PRIOR_V1 0.1\n')
            for ix in range(-20,20):
                for iy in range(-20,20):
                    for iz in range(0,25):
                        f.write(f'{(ix+.5)*.1:.3f} {(iy+.5)*.1:.3f} {(iz+.5)*.1:.3f} 0\n')
        def launch(binary, params, name):
            command = [binary, '--ros-args']
            for key, value in params.items():
                if key == 'params-file':command += ['--params-file', str(value)]
                else:command += ['-p', f'{key}:={value}']
            log = (directory/(name+'.log')).open('w')
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            processes.append((process,log))
            return process
        try:
            mapper = launch(args.mapper, {'config_file':directory/'nav.yaml','rig_file':directory/'rig.yaml',
                'cad_prior_file':directory/'cad.txt','static_cad_only':'true','route_backend':'super_astar'}, 'mapper')
            deadline = time.monotonic()+45
            while not maps and time.monotonic()<deadline:spin(.1)
            assert len(maps)==1, 'mapper did not publish static geometry'
            spin(1.5)  # Geometry is already older than the dynamic deadline.
            assert len(maps)==1, 'unchanged geometry was republished'
            assert len(states)>10, 'missing lightweight pose leases'
            late_maps = []
            subscriptions.append(node.create_subscription(MapSnapshot, '/navigation/map_snapshot', late_maps.append, latched))
            goal = NavigationGoal();goal.header.frame_id='map';goal.localization_epoch=1;goal.goal_id=1
            goal.pose.position.x=.8;goal.pose.position.z=1.;goal.pose.orientation.w=1.
            goal_pub.publish(goal)
            deadline=time.monotonic()+60
            while not any(m.status==m.VALID for m in routes) and time.monotonic()<deadline:spin(.1)
            assert late_maps, 'late subscriber missed retained geometry'
            assert any(m.status==m.VALID for m in routes), 'no valid route using retained static geometry'
            spin(2)  # Allow SUPER to commit its trajectory.
            nroute,ncurve=len(routes),len(curves)
            spin(2)
            assert len(maps)==1, 'geometry revision changed without a geometry change'
            assert sum(m.status==m.VALID for m in routes[nroute:])>=8, 'cached route not refreshed'
            assert any(m.status==m.VALID for m in curves[ncurve:]), 'SUPER produced no certified trajectory'
            # Small movement must preserve the cached global goal.
            x=.02;nroute=len(routes);spin(1)
            valid=[m for m in routes[nroute:] if m.status==m.VALID]
            assert valid, 'no route after movement'
            assert abs(valid[-1].points[-1].x-.8)<.005, 'cached global route lost goal'
            # Stop leases by making VIO unhealthy. Geometry alone cannot keep routes alive.
            healthy=False;spin(.7);nroute,ncurve=len(routes),len(curves);spin(.7)
            assert routes[nroute:] and all(m.status!=m.VALID for m in routes[nroute:]), 'stale localization authorized a route'
            assert all(m.status!=m.VALID for m in curves[ncurve:]), 'unhealthy VIO authorized trajectory'
            state=MapState();state.header.frame_id='map';state.localization_epoch=1;state.revision=2
            state.body_pose.position.z=1.;state.body_pose.orientation.w=1.
            nroute=len(routes)
            for _ in range(20):
                state.header.stamp=node.get_clock().now().to_msg();state_pub.publish(state);spin(.05)
            assert all(m.status!=m.VALID for m in routes[nroute:]), 'wrong geometry revision authorized a route'
            healthy=True;spin(1)
            assert routes[-1].status==routes[-1].VALID, 'fresh matching pose did not restore route'
            print('PASS: single retained CAD, late join, cached global goal, SUPER certificate, stale/identity rejection and recovery')
        except BaseException:
            for name in ('mapper',):
                path=directory/(name+'.log')
                if path.exists():print(name+' LOG\n'+path.read_text()[-18000:])
            raise
        finally:
            for process,log in reversed(processes):
                if process.poll() is None:process.send_signal(signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                log.close()
            node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
