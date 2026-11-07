#!/usr/bin/env python3
"""Real Air-FAR process integration test on an isolated, synthetic ROS domain.

Checks the route contract, not flight success or global exploration completeness.
"""
import json
import os
from pathlib import Path
import subprocess
import time
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs_py import point_cloud2
from dronehacks_msgs.msg import MapSnapshot, NavigationGoal, RoutePolyline


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '43':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=43; never inject synthetic inputs into a flight domain')
    root=Path(__file__).resolve().parents[1]
    out=root/'artifacts/build/airfar-contract';out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('airfar_contract_test')
    routes=[]
    node.create_subscription(RoutePolyline,'/navigation/route',lambda m:routes.append(m),10)
    snapshots=node.create_publisher(MapSnapshot,'/navigation/map_snapshot',1)
    goals=node.create_publisher(NavigationGoal,'/navigation/goal',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    free=[(x*.1+.05,y*.1+.05,z*.1+.05) for x in range(-3,25) for y in range(-8,8) for z in range(5,16)]
    occupied=[(3.05,y*.1+.05,z*.1+.05) for y in range(-20,21) for z in range(0,25)]
    revision=0
    def publish_snapshot():
        nonlocal revision
        revision+=1
        m=MapSnapshot();m.header=Header(stamp=node.get_clock().now().to_msg(),frame_id='map')
        m.localization_epoch=1;m.revision=revision;m.resolution=.1
        m.lower=Point(x=-4.,y=-4.,z=0.);m.upper=Point(x=4.,y=4.,z=3.)
        m.occupied=point_cloud2.create_cloud_xyz32(m.header,occupied)
        m.free=point_cloud2.create_cloud_xyz32(m.header,free)
        m.body_pose.header=m.header;m.body_pose.pose.position.z=1.;m.body_pose.pose.orientation.w=1.
        snapshots.publish(m)
    def pump(seconds,publish=False):
        started=time.monotonic();last=0
        while time.monotonic()-started<seconds:
            if child.poll() is not None:raise RuntimeError(f'Air-FAR exited {child.returncode}')
            if publish and time.monotonic()-last>.25:publish_snapshot();last=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.02)
    result={'scope':'Air-FAR ROS contract only; synthetic inputs; no vehicle or mission'}
    with (out/'node.log').open('w') as log:
        child=subprocess.Popen([str(root/'build/navigation/airfar_route_node'),'--ros-args','--params-file',str(root/'config/airfar.yaml')],stdout=log,stderr=subprocess.STDOUT)
        try:
            pump(2.,True)
            g=NavigationGoal();g.header.frame_id='map';g.localization_epoch=1;g.goal_id=1
            g.pose.position=Point(x=1.5,y=0.,z=1.);g.pose.orientation.w=1.;goals.publish(g)
            pump(8.,True)
            valid=[r for r in routes if r.status==r.VALID and r.goal_id==1]
            assert valid,'no full route published'
            route=valid[-1]
            assert route.header.frame_id=='map' and route.localization_epoch==1 and route.map_revision>0
            assert len(route.points)>=2 and abs(route.points[0].x)<.2 and abs(route.points[0].z-1)<.2
            assert abs(route.points[-1].x-1.5)<.2 and abs(route.points[-1].z-1)<.2,'route ordering or goal altitude wrong'
            assert all(b.route_id>a.route_id for a,b in zip(routes,routes[1:])),'non-monotonic route IDs'
            result['route_vertices']=len(route.points);result['route_revision']=route.map_revision
            routes.clear();pump(2.,False)
            assert routes and routes[-1].status==RoutePolyline.INVALID,'stale map left valid route active'
            result['stale_map_invalidates']=True
            routes.clear();g.goal_id=2;g.cancel=True;goals.publish(g);pump(1.,True)
            assert routes and all(r.status==RoutePolyline.INVALID for r in routes if r.goal_id==2),'cancel left route active'
            assert any(r.goal_id==2 for r in routes),'cancel not acknowledged'
            result['cancellation_invalidates']=True
            log.flush()
            sizes=[int(n) for n in re.findall(r'current number of navigation nodes: (\d+)',(out/'node.log').read_text())]
            assert len(sizes)>10 and max(sizes[-10:])-min(sizes[-10:])<=2,'stationary graph grows on each snapshot'
            result['stationary_graph_nodes']=sizes[-1];result['pass']=True
        except Exception as exc:
            result.update({'pass':False,'error':str(exc)});raise
        finally:
            child.terminate()
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            node.destroy_node();rclpy.shutdown()
            (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,indent=2))


if __name__=='__main__':main()
