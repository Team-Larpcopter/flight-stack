#!/usr/bin/env python3
"""Record message timing without a rosbag writer or raw-image storage."""
import json
import sys
import time
from collections import defaultdict
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, CompressedImage, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64, Bool
from visualization_msgs.msg import MarkerArray

rclpy.init()
node = rclpy.create_node('vins_load_probe')
samples = defaultdict(list)
start = time.monotonic()

def receive(topic, message):
    if isinstance(message, Bool):
        samples[topic].append([time.monotonic()-start, None, message.data])
        return
    stamp = (message.data * 1e-9 if isinstance(message, Int64) else
             message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
             if hasattr(message, 'header') else None)
    samples[topic].append([time.monotonic()-start,
                          node.get_clock().now().nanoseconds * 1e-9-stamp if stamp is not None else None])

subs=[]
for kind, topic in [(Imu, '/autonomy/imu/data'), (Odometry, '/autonomy/odomimu'),
                    (Odometry, '/autonomy/odometry'), (Int64, '/autonomy/processed_vio'),
                    (Image, '/stereo/depth/image'),
                    (Bool, '/autonomy/route_ready'),
                    (CompressedImage, '/viz/native_oak/left/image_undistorted/compressed'),
                    (CompressedImage, '/viz/stereo/depth/image/compressed'),
                    (MarkerArray, '/viz/autonomy/occupied_chunks')]:
    subs.append(node.create_subscription(kind, topic, lambda m,t=topic:receive(t,m), qos_profile_sensor_data))
while time.monotonic()-start < float(sys.argv[1]):
    rclpy.spin_once(node, timeout_sec=.1)
with open('/profile/topic_timing.json','w') as output:
    json.dump(samples, output)
node.destroy_node()
rclpy.shutdown()
