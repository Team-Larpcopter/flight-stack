"""Foxglove for the drone: read-only bridge + low-bandwidth /viz relay.

    ros2 launch larp_perception perception.launch.py [port:=8766]
        [viz_rate_hz:=5.0] [viz_scale:=0.5] [viz_quality:=60]
        [viz_right:=false] [viz_rectified:=false] [viz_tracks:=false]

* foxglove_bridge on ws://<drone>:<port>, READ-ONLY (no client publish, services
  or parameters) and serving ONLY /viz/*: bounded visualization topics reach Wi-Fi.
  Queue depth 1 and a configurable send buffer (64 KB by default): Foxglove always gets the newest
  message and a slow link drops instead of queueing (no lag build-up).
* viz_relay, the source of /viz (depth nodes may publish their own /viz previews,
  e.g. larp_liteanystereo's /viz/las_depth/*):
  - images as JPEG (/viz/<topic>/compressed) at viz_rate_hz, scaled by viz_scale:
    left camera and the planner's depth /stereo/depth/image (colourised: near red,
    far blue, no depth black). viz_right / viz_rectified / viz_tracks add the
    right camera, the camera's rectified pair and sqrtVINS's feature-track image
    (that one makes sqrtVINS draw every frame on its main thread and costs it
    several Hz of pose rate, so it is off by default)
  - sqrtVINS pose 10 Hz (poseimu: published once per frame anyway; subscribing to
    the IMU-rate odomimu makes sqrtVINS propagate and publish ~400 Hz), IMU 20 Hz,
    feature clouds 2 Hz, forwarded as raw bytes (newest only), and a bounded path
    built from the poses (sqrtVINS's own path resends the whole growing
    trajectory every update)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# PX4's own position estimate (EKF2, NED, px4_msgs MESSAGE_VERSION 0 -> no _v suffix).
PX4_POSITION = "/fmu/out/vehicle_local_position@10"
VINS_THROTTLE = ["/autonomy/poseimu@10", "/autonomy/imu/data@20",
                 *[f"/autonomy/points_{kind}@2" for kind in ("slam", "msckf", "aruco", "sim")]]
THROTTLE = VINS_THROTTLE + ["/autonomy/goal@2", "/autonomy/map_state@5",
            "/autonomy/committed_trajectory@5", "/autonomy/rejected_map@1", PX4_POSITION]
SERVE = "['^/viz/.*$', '^/tf_static$']"  # monitoring plus fixed frame registration; no client writes
NOTHING = "['^$']"


def relay(context):
    on = lambda name: LaunchConfiguration(name).perform(context).lower() == "true"  # noqa: E731
    raw = ["/native_oak/left/image_undistorted"]
    raw += ["/native_oak/right/image_undistorted"] if on("viz_right") else []
    raw += ["/native_oak/rectified/left/image", "/native_oak/rectified/right/image"] if on("viz_rectified") else []
    raw += ["/autonomy/trackhist"] if on("viz_tracks") else []
    flight = on("flight_monitor_only")
    throttle = VINS_THROTTLE + ["/flight_executive/state@2", "/autonomy/map_state@2", "/autonomy/takeoff_ready@2", PX4_POSITION] if flight else THROTTLE
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
                    "depth_topics": ["/stereo/depth/image"],
                    "throttle_topics": throttle,
                    "path_pose_topic": "/autonomy/poseimu",
                    "path_hz": 5.0,
                    "path_max_points": 1500,
                    "rate_hz": ParameterValue(arg("viz_rate_hz"), value_type=float),
                    "scale": ParameterValue(arg("viz_scale"), value_type=float),
                    "jpeg_quality": ParameterValue(arg("viz_quality"), value_type=int),
                    "min_depth_m": 0.2,
                    "max_depth_m": float("inf") if flight else 6.0,
                }
            ],
        )
    ]


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument("send_buffer_limit", default_value="65536"),
            DeclareLaunchArgument("viz_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("viz_scale", default_value="0.5"),
            DeclareLaunchArgument("viz_quality", default_value="60"),
            DeclareLaunchArgument("flight_monitor_only", default_value="false"),
            DeclareLaunchArgument("viz_right", default_value="false"),
            DeclareLaunchArgument("viz_rectified", default_value="false"),
            DeclareLaunchArgument("viz_tracks", default_value="false"),
            OpaqueFunction(function=relay),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"])
                ),
                launch_arguments={
                    "port": arg("port"),
                    "address": "0.0.0.0",
                    "capabilities": "[connectionGraph,assets]",
                    "send_buffer_limit": arg("send_buffer_limit"),
                    "min_qos_depth": "1",
                    # 32 depth-one retained map writers; single-writer streams stay depth one.
                    "max_qos_depth": "32",
                    "topic_whitelist": SERVE,
                    "client_topic_whitelist": NOTHING,
                    "service_whitelist": NOTHING,
                    "param_whitelist": NOTHING,
                }.items(),
            ),
        ]
    )
