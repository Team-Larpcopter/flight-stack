"""Lite Any Stereo V2 (LAS2) depth on the native OAK stereo pair, TensorRT FP16.

    ros2 launch larp_liteanystereo las_depth.launch.py [input:=rectified] \\
        [engine_path:=/models/las2_m_320x480_d192_fp16.engine] [publish_as_primary:=false]

Runs las_stereo_node (disparity -> metric depth, CameraInfo, /viz previews) with
the LAS2 engine (inputs raw 0-255; LAS normalizes inside the network):

* /las_depth/depth/image_raw (16UC1 mm) + /las_depth/depth/camera_info,
  /las_depth/disparity/image_raw, 5 Hz /viz/las_depth/{depth,disparity}/image/compressed
* publish_as_primary:=true also publishes /stereo/depth/image (+ camera_info),
  the depth the planner's depth_map_node reads

input:=rectified (default): the camera's own rectified, hardware-synced pair
(/native_oak/rectified/*, 800x500, oak_native_node publish_rectified): no
rectification on the Jetson and 2.5x less image data per pair.

Engine: reuse the existing serialized engine; see models/README.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

INPUTS = {
    "rectified": {
        "left_topic": "/native_oak/rectified/left/image",
        "right_topic": "/native_oak/rectified/right/image",
        "left_info_topic": "/native_oak/rectified/left/camera_info",
        "right_info_topic": "/native_oak/rectified/right/camera_info",
    },

}


def node(context):
    arg = LaunchConfiguration
    source = arg("input").perform(context)
    if source not in INPUTS:
        raise RuntimeError(f"input must be one of {sorted(INPUTS)}")
    return [
        Node(
            package="larp_liteanystereo",
            executable="las_stereo_node",
            name="las_stereo_node",
            output="screen",
            parameters=[
                dict(INPUTS[source]),
                {
                    "engine_path": arg("engine_path"),
                    "publish_as_primary": ParameterValue(arg("publish_as_primary"), value_type=bool),
                    "target_height": ParameterValue(arg("target_height"), value_type=int),
                    "target_width": ParameterValue(arg("target_width"), value_type=int),
                    "min_depth_m": ParameterValue(arg("min_depth_m"), value_type=float),
                    "max_depth_m": ParameterValue(arg("max_depth_m"), value_type=float),
                    "viz_rate_hz": ParameterValue(arg("viz_rate_hz"), value_type=float),
                    "output_depth_topic": "/las_depth/depth/image_raw",
                    "output_info_topic": "/las_depth/depth/camera_info",
                    "output_disp_topic": "/las_depth/disparity/image_raw",
                    "viz_depth_topic": "/viz/las_depth/depth/image/compressed",
                    "viz_disp_topic": "/viz/las_depth/disparity/image/compressed",
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input", default_value="rectified"),
            DeclareLaunchArgument("engine_path", default_value="/models/las2_m_320x480_d192_fp16.engine"),
            DeclareLaunchArgument("publish_as_primary", default_value="false"),
            DeclareLaunchArgument("target_height", default_value="320"),
            DeclareLaunchArgument("target_width", default_value="480"),
            DeclareLaunchArgument("min_depth_m", default_value="0.15"),
            DeclareLaunchArgument("max_depth_m", default_value="8.0"),
            DeclareLaunchArgument("viz_rate_hz", default_value="5.0"),
            OpaqueFunction(function=node),
        ]
    )
