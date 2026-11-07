"""Depth filter + Foxglove for the native OAK camera (larp_hardware oak_native_node).

    ros2 launch larp_perception perception.launch.py [depth_filter:=true] [foxglove:=true]
        [port:=8766] [viz_rate_hz:=5.0] [viz_scale:=0.5] [viz_quality:=60]
        [viz_right:=false] [viz_raw_depth:=false] [viz_tracks:=false]

The camera node itself is started separately (it owns the device):
oak_native_node publishes the stereo pair, IMU and raw on-device depth
(/native_oak/depth/image_raw). This launch adds:

* depth_filter: Jetson CUDA filter -> /stereo/depth/image + camera_info
  (what larp_navigation depth_map_node reads).
* foxglove_bridge on ws://<drone>:<port>, READ-ONLY (no client publish, services
  or parameters) and serving ONLY /viz/*: nothing reaches Wi-Fi at full rate.
  Queue depth 1 and a 64 KB send buffer: Foxglove always gets the newest
  message and a slow link drops instead of queueing (no lag build-up).
* viz_relay, the only source of /viz:
  - images as JPEG (/viz/<topic>/compressed) at viz_rate_hz, scaled by viz_scale:
    left camera and filtered depth (colourised: near red, far blue, none black);
    viz_right / viz_raw_depth / viz_tracks add the right camera, raw camera depth
    and sqrtVINS's feature-track image (that one makes sqrtVINS draw every
    frame on its main thread and cost it ~10 Hz of pose rate, so off by default)
  - sqrtVINS pose 10 Hz (poseimu: free, once per frame; subscribing to the
    IMU-rate odomimu makes sqrtVINS propagate and publish ~400 Hz), corrected IMU 20 Hz, feature clouds 2 Hz, forwarded
    as raw bytes (newest only), and a bounded path built from the poses (instead
    of sqrtVINS's path, which resends the whole growing trajectory each update)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

THROTTLE = ["/autonomy/poseimu@10", "/autonomy/imu/data@20", "/autonomy/points_slam@2", "/autonomy/points_msckf@2"]
SERVE = "['^/viz/.*$']"  # only the throttled /viz copies; no client writes at all
NOTHING = "['^$']"


def relay(context):
    on = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"  # noqa: E731
    raw = ["/native_oak/left/image_undistorted"] + (["/native_oak/right/image_undistorted"] if on("viz_right") else [])
    raw += ["/autonomy/trackhist"] if on("viz_tracks") else []
    depth = ["/stereo/depth/image"] + (["/native_oak/depth/image_raw"] if on("viz_raw_depth") else [])
    arg = LaunchConfiguration
    return [
        Node(
            package="larp_perception",
            executable="viz_relay",
            name="viz_relay",
            output="screen",
            parameters=[
                {
                    "topics": [""],
                    "raw_topics": raw,
                    "depth_topics": depth,
                    "throttle_topics": THROTTLE,
                    "path_pose_topic": "/autonomy/poseimu",
                    "path_hz": 5.0,
                    "path_max_points": 1500,
                    "rate_hz": ParameterValue(arg("viz_rate_hz"), value_type=float),
                    "scale": ParameterValue(arg("viz_scale"), value_type=float),
                    "jpeg_quality": ParameterValue(arg("viz_quality"), value_type=int),
                    "min_depth_m": 0.3,
                    "max_depth_m": 6.0,
                }
            ],
        )
    ]


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_filter", default_value="true"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("viz_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("viz_scale", default_value="0.5"),
            DeclareLaunchArgument("viz_quality", default_value="60"),
            DeclareLaunchArgument("viz_right", default_value="false"),
            DeclareLaunchArgument("viz_raw_depth", default_value="false"),
            DeclareLaunchArgument("viz_tracks", default_value="false"),
            Node(
                package="larp_perception",
                executable="depth_filter",
                name="depth_filter",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
                condition=IfCondition(arg("depth_filter")),
            ),
            GroupAction([OpaqueFunction(function=relay)], condition=IfCondition(arg("foxglove"))),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"])
                ),
                condition=IfCondition(arg("foxglove")),
                launch_arguments={
                    "port": arg("port"),
                    "address": "0.0.0.0",
                    "capabilities": "[connectionGraph,assets]",
                    "send_buffer_limit": "65536",
                    "min_qos_depth": "1",
                    "max_qos_depth": "1",
                    "topic_whitelist": SERVE,
                    "client_topic_whitelist": NOTHING,
                    "service_whitelist": NOTHING,
                    "param_whitelist": NOTHING,
                }.items(),
            ),
        ]
    )
