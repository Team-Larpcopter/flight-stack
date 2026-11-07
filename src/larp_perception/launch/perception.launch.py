"""Depth filter + Foxglove for the native OAK camera (larp_hardware oak_native_node).

    ros2 launch larp_perception perception.launch.py [depth_filter:=true] [foxglove:=true]
        [viz_rate_hz:=5.0] [viz_scale:=0.5] [port:=8766]

The camera node itself is started separately (it owns the device):
oak_native_node publishes the stereo pair, IMU and raw on-device depth
(/native_oak/depth/image_raw). This launch adds:

* depth_filter: Jetson CUDA filter -> /stereo/depth/image + camera_info
  (what larp_navigation depth_map_node reads).
* foxglove_bridge on ws://<drone>:<port>, READ-ONLY (no client publish, services
  or parameters), serving every topic except full-rate images; small send buffer
  so a slow link drops old messages instead of queueing them.
* viz_relay: images for Foxglove as /viz/<topic>/compressed JPEGs at
  viz_rate_hz, scaled by viz_scale (full-rate images are more than the Wi-Fi
  uplink): the undistorted stereo pair, sqrtVINS's feature tracks, and the
  filtered and raw depth colourised (near red, far blue, no depth black).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RAW_IMAGES = ["/native_oak/left/image_undistorted", "/native_oak/right/image_undistorted", "/autonomy/trackhist"]
DEPTH_IMAGES = ["/stereo/depth/image", "/native_oak/depth/image_raw"]
# Every topic except images (those only as /viz previews); no client writes at all.
SERVE = "['^(/viz/.*|(?!.*/image)(?!.*trackhist).*)$']"
NOTHING = "['^$']"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_filter", default_value="true"),
            DeclareLaunchArgument("foxglove", default_value="true"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("viz_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("viz_scale", default_value="0.5"),
            DeclareLaunchArgument("viz_quality", default_value="80"),
            Node(
                package="larp_perception",
                executable="depth_filter",
                name="depth_filter",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
                condition=IfCondition(arg("depth_filter")),
            ),
            Node(
                package="larp_perception",
                executable="viz_relay",
                name="viz_relay",
                output="screen",
                condition=IfCondition(arg("foxglove")),
                parameters=[
                    {
                        "topics": [""],
                        "raw_topics": RAW_IMAGES,
                        "depth_topics": DEPTH_IMAGES,
                        "rate_hz": ParameterValue(arg("viz_rate_hz"), value_type=float),
                        "scale": ParameterValue(arg("viz_scale"), value_type=float),
                        "jpeg_quality": ParameterValue(arg("viz_quality"), value_type=int),
                        "min_depth_m": 0.3,
                        "max_depth_m": 6.0,
                    }
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"])
                ),
                condition=IfCondition(arg("foxglove")),
                launch_arguments={
                    "port": arg("port"),
                    "address": "0.0.0.0",
                    "capabilities": "[connectionGraph,assets]",
                    "send_buffer_limit": "131072",
                    "min_qos_depth": "1",
                    "max_qos_depth": "5",
                    "topic_whitelist": SERVE,
                    "client_topic_whitelist": NOTHING,
                    "service_whitelist": NOTHING,
                    "param_whitelist": NOTHING,
                }.items(),
            ),
        ]
    )
