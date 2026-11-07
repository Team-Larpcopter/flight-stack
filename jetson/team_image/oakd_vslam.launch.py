"""Larpcopter override of skybeta's oakd_vslam_bringup/launch/oakd_vslam.launch.py.

Derived from Artash-N/skybeta 0.3.2 (Apache-2.0). Installed over the original by
the team image (jetson/team_image/Dockerfile), so skybeta's run_slam.sh launches
it unchanged: same name, same launch arguments, same TF, same cuVSLAM tuning.

Three differences from skybeta's version, all from enabling on-device depth
(see oakd_vslam.yaml next to this file):

  1. cuVSLAM reads the OAK's RECTIFIED mono8 pair from StereoDepth
     (/oak/left/image_rect, /oak/right/image_rect) instead of the raw
     bgr8 sockets, so ``rectified_images`` is True.
  2. The two bgr8 -> mono8 ImageFormatConverterNodes are gone: the device
     already sends mono8.
  3. The input topics are launch arguments, so a driver naming difference can be
     corrected without rebuilding the team image.

Everything else (camera mount TF, cuVSLAM parameters and their reasons) is
skybeta's. Read skybeta's original for the measurements behind each value.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    depthai_share = get_package_share_directory("depthai_ros_driver_v3")
    bringup_share = get_package_share_directory("oakd_vslam_bringup")

    args = [
        # ---- camera mount pose (skybeta, unchanged) -----------------------------
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="0.0"),
        # ---- cuVSLAM tuning (skybeta, unchanged) --------------------------------
        DeclareLaunchArgument("image_jitter_threshold_ms", default_value="40.0"),
        DeclareLaunchArgument("enable_localization_n_mapping", default_value="False"),
        DeclareLaunchArgument("enable_slam_visualization", default_value="False"),
        DeclareLaunchArgument("enable_landmarks_view", default_value="False"),
        DeclareLaunchArgument("enable_observations_view", default_value="False"),
        DeclareLaunchArgument("path_max_size", default_value="1024"),
        # ---- Larpcopter: rectified mono8 inputs from StereoDepth -----------------
        DeclareLaunchArgument("left_image", default_value="/oak/left/image_rect"),
        DeclareLaunchArgument("left_info", default_value="/oak/left/camera_info"),
        DeclareLaunchArgument("right_image", default_value="/oak/right/image_rect"),
        DeclareLaunchArgument("right_info", default_value="/oak/right/camera_info"),
    ]

    base_to_oak_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_oak",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", LaunchConfiguration("camera_roll"),
            "--pitch", LaunchConfiguration("camera_pitch"),
            "--yaw", LaunchConfiguration("camera_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", "oak_parent_frame",
        ],
    )  # fmt: skip

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(depthai_share, "launch", "driver.launch.py")),
        launch_arguments={
            "name": "oak",
            "params_file": os.path.join(bringup_share, "config", "oakd_vslam.yaml"),
        }.items(),
    )

    vslam = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam_node",
        parameters=[
            {
                "enable_image_denoising": False,
                "rectified_images": True,  # Larpcopter: StereoDepth rectifies on device
                "enable_imu_fusion": False,
                "gyro_noise_density": 0.000244,
                "gyro_random_walk": 0.000019393,
                "accel_noise_density": 0.001862,
                "accel_random_walk": 0.003,
                "imu_frame": "oak_imu_frame",
                "base_frame": "base_link",
                "map_frame": "map",
                "odom_frame": "odom",
                "camera_optical_frames": [
                    "oak_left_camera_optical_frame",
                    "oak_right_camera_optical_frame",
                ],
                "publish_odom_to_base_tf": True,
                "publish_map_to_odom_tf": True,
                "image_jitter_threshold_ms": ParameterValue(
                    LaunchConfiguration("image_jitter_threshold_ms"), value_type=float
                ),
                "enable_localization_n_mapping": ParameterValue(
                    LaunchConfiguration("enable_localization_n_mapping"), value_type=bool
                ),
                "enable_slam_visualization": ParameterValue(
                    LaunchConfiguration("enable_slam_visualization"), value_type=bool
                ),
                "enable_landmarks_view": ParameterValue(
                    LaunchConfiguration("enable_landmarks_view"), value_type=bool
                ),
                "enable_observations_view": ParameterValue(
                    LaunchConfiguration("enable_observations_view"), value_type=bool
                ),
                "path_max_size": ParameterValue(
                    LaunchConfiguration("path_max_size"), value_type=int
                ),
            }
        ],
        remappings=[
            ("visual_slam/image_0", LaunchConfiguration("left_image")),
            ("visual_slam/camera_info_0", LaunchConfiguration("left_info")),
            ("visual_slam/image_1", LaunchConfiguration("right_image")),
            ("visual_slam/camera_info_1", LaunchConfiguration("right_info")),
            ("visual_slam/imu", "/oak/imu/data"),
        ],
    )

    container = ComposableNodeContainer(
        name="vslam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[vslam],
        output="screen",
    )

    return LaunchDescription(args + [base_to_oak_tf, camera, container])
