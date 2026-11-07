"""Launch standalone TensorRT neural stereo depth node (larp_neural_depth).

    ros2 launch larp_neural_depth neural_depth.launch.py \
        [engine_path:=/root/neural_depth_ws/models/fast_foundationstereo_320x480_fp16.engine] \
        [target_height:=320] [target_width:=480] [viz_rate_hz:=5.0]

Does not start or modify oak_native_node, depth_filter, or foxglove_bridge.
Publishes:
  - /neural_depth/depth/image_raw                 (16UC1 mm)
  - /neural_depth/depth/camera_info
  - /viz/neural_depth/depth/image/compressed      (5 Hz JET JPEG automatically picked up by foxglove_bridge)
  - /viz/neural_depth/disparity/image/compressed  (5 Hz TURBO JPEG automatically picked up by foxglove_bridge)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    default_engine = os.environ.get(
        "NEURAL_STEREO_ENGINE",
        "/models/fast_foundationstereo_320x480_fp16.engine",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("engine_path", default_value=default_engine),
            DeclareLaunchArgument("left_topic", default_value="/native_oak/left/image_undistorted"),
            DeclareLaunchArgument("right_topic", default_value="/native_oak/right/image_undistorted"),
            DeclareLaunchArgument("left_info_topic", default_value="/native_oak/left/camera_info"),
            DeclareLaunchArgument("depth_info_topic", default_value="/native_oak/depth/camera_info"),
            DeclareLaunchArgument("output_depth_topic", default_value="/neural_depth/depth/image_raw"),
            DeclareLaunchArgument("output_info_topic", default_value="/neural_depth/depth/camera_info"),
            DeclareLaunchArgument("publish_as_primary", default_value="false"),
            DeclareLaunchArgument("primary_depth_topic", default_value="/stereo/depth/image"),
            DeclareLaunchArgument("primary_info_topic", default_value="/stereo/depth/camera_info"),
            DeclareLaunchArgument("target_height", default_value="320"),
            DeclareLaunchArgument("target_width", default_value="480"),
            DeclareLaunchArgument("output_height", default_value="0"),
            DeclareLaunchArgument("output_width", default_value="0"),
            DeclareLaunchArgument("min_depth_m", default_value="0.15"),
            DeclareLaunchArgument("max_depth_m", default_value="8.0"),
            DeclareLaunchArgument("viz_enable", default_value="true"),
            DeclareLaunchArgument("viz_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("viz_min_depth_m", default_value="0.2"),
            DeclareLaunchArgument("viz_max_depth_m", default_value="6.0"),
            DeclareLaunchArgument("viz_jpeg_quality", default_value="60"),
            Node(
                package="larp_neural_depth",
                executable="neural_stereo_node",
                name="neural_stereo_node",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
                parameters=[
                    {
                        "engine_path": arg("engine_path"),
                        "left_topic": arg("left_topic"),
                        "right_topic": arg("right_topic"),
                        "left_info_topic": arg("left_info_topic"),
                        "depth_info_topic": arg("depth_info_topic"),
                        "output_depth_topic": arg("output_depth_topic"),
                        "output_info_topic": arg("output_info_topic"),
                        "publish_as_primary": ParameterValue(arg("publish_as_primary"), value_type=bool),
                        "primary_depth_topic": arg("primary_depth_topic"),
                        "primary_info_topic": arg("primary_info_topic"),
                        "target_height": ParameterValue(arg("target_height"), value_type=int),
                        "target_width": ParameterValue(arg("target_width"), value_type=int),
                        "output_height": ParameterValue(arg("output_height"), value_type=int),
                        "output_width": ParameterValue(arg("output_width"), value_type=int),
                        "min_depth_m": ParameterValue(arg("min_depth_m"), value_type=float),
                        "max_depth_m": ParameterValue(arg("max_depth_m"), value_type=float),
                        "viz_enable": ParameterValue(arg("viz_enable"), value_type=bool),
                        "viz_rate_hz": ParameterValue(arg("viz_rate_hz"), value_type=float),
                        "viz_min_depth_m": ParameterValue(arg("viz_min_depth_m"), value_type=float),
                        "viz_max_depth_m": ParameterValue(arg("viz_max_depth_m"), value_type=float),
                        "viz_jpeg_quality": ParameterValue(arg("viz_jpeg_quality"), value_type=int),
                    }
                ],
            ),
        ]
    )
