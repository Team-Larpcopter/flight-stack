"""larp_oak camera + cuVSLAM on full-resolution images + Jetson depth filter.

Drop-in for skybeta's oakd_vslam.launch.py (same name of the TF root, same
cuVSLAM tuning, same output topics), with these differences:
  * camera: larp_oak instead of depthai_ros_driver (respawned on device error)
  * cuVSLAM input: raw 1280x800 mono8 /oak/{left,right}/image_raw with the
    calibration's rational_polynomial model (rectified_images False), no
    bgr8 -> mono8 converters (the device sends mono8)
  * depth: /oak/stereo/image_raw (raw) and /oak/stereo/filtered/image
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    arg = LaunchConfiguration
    args = [
        DeclareLaunchArgument(n, default_value=v)
        for n, v in (
            ("camera_x", "0.0"), ("camera_y", "0.0"), ("camera_z", "0.0"),
            ("camera_roll", "0.0"), ("camera_pitch", "0.0"), ("camera_yaw", "0.0"),
            ("image_jitter_threshold_ms", "40.0"),
            ("enable_localization_n_mapping", "False"),
            ("enable_slam_visualization", "False"),
            ("enable_landmarks_view", "False"),
            ("enable_observations_view", "False"),
            ("path_max_size", "1024"),
            ("fps", "30.0"), ("vio_width", "1280"), ("vio_height", "800"),
            ("stereo_width", "800"), ("stereo_height", "500"), ("depth_preset", "ROBOTICS"),
            ("subpixel", "True"), ("depth_filter", "True"), ("vslam", "True"),
        )
    ]  # fmt: skip

    base_to_oak_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_oak",
        arguments=[
            "--x", arg("camera_x"), "--y", arg("camera_y"), "--z", arg("camera_z"),
            "--roll", arg("camera_roll"), "--pitch", arg("camera_pitch"), "--yaw", arg("camera_yaw"),
            "--frame-id", "base_link", "--child-frame-id", "oak_parent_frame",
        ],
    )  # fmt: skip

    oak = Node(
        package="larp_oak",
        executable="oak_node",
        name="oak",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            {
                "fps": ParameterValue(arg("fps"), value_type=float),
                "vio_width": ParameterValue(arg("vio_width"), value_type=int),
                "vio_height": ParameterValue(arg("vio_height"), value_type=int),
                "stereo_width": ParameterValue(arg("stereo_width"), value_type=int),
                "stereo_height": ParameterValue(arg("stereo_height"), value_type=int),
                "depth_preset": arg("depth_preset"),
                "subpixel": ParameterValue(arg("subpixel"), value_type=bool),
            }
        ],
    )

    depth_filter = Node(
        package="larp_oak",
        executable="depth_filter",
        name="depth_filter",
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(arg("depth_filter")),
    )

    vslam = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam_node",
        parameters=[
            {
                "enable_image_denoising": False,
                "rectified_images": False,  # raw sensor images + calibration distortion
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
                "image_jitter_threshold_ms": ParameterValue(arg("image_jitter_threshold_ms"), value_type=float),
                "enable_localization_n_mapping": ParameterValue(
                    arg("enable_localization_n_mapping"), value_type=bool
                ),
                "enable_slam_visualization": ParameterValue(arg("enable_slam_visualization"), value_type=bool),
                "enable_landmarks_view": ParameterValue(arg("enable_landmarks_view"), value_type=bool),
                "enable_observations_view": ParameterValue(arg("enable_observations_view"), value_type=bool),
                "path_max_size": ParameterValue(arg("path_max_size"), value_type=int),
            }
        ],
        remappings=[
            ("visual_slam/image_0", "/oak/left/image_raw"),
            ("visual_slam/camera_info_0", "/oak/left/camera_info"),
            ("visual_slam/image_1", "/oak/right/image_raw"),
            ("visual_slam/camera_info_1", "/oak/right/camera_info"),
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
        condition=IfCondition(arg("vslam")),
    )

    return LaunchDescription(args + [base_to_oak_tf, oak, depth_filter, container])
