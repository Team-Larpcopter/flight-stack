"""sqrtVINS on the OAK-D S2: ov_srvins run_subscribe_msckf.

Inputs (from larp_oak): /oak/{left,right}/image_undistorted (undistorted on the
camera), /oak/imu/data. undistort:=true adds the old host-side Python undistorter
(only if larp_oak runs with vio_undistorted:=false: both publish the same topics).
Outputs (namespace ov_srvins): odomimu (nav_msgs/Odometry), poseimu, pathimu,
points_msckf / points_slam (feature clouds)

    ros2 launch larp_localization sqrtvins.launch.py
    ros2 launch larp_localization sqrtvins.launch.py config:=/path/estimator_config.yaml

sqrtVINS runs alongside cuVSLAM here; it does not feed PX4.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("larp_localization"), "config", "oak_d_s2", "estimator_config.yaml"
    )
    arg = LaunchConfiguration
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("namespace", default_value="ov_srvins"),
            DeclareLaunchArgument("verbosity", default_value="INFO"),
            DeclareLaunchArgument("undistort", default_value="false"),
            Node(
                package="larp_localization",
                executable="undistort",
                name="undistort",
                output="screen",
                condition=IfCondition(arg("undistort")),
            ),
            Node(
                package="ov_srvins",
                executable="run_subscribe_msckf",
                namespace=arg("namespace"),
                name="ov_srvins",
                output="screen",
                parameters=[
                    {"config_path": arg("config"), "verbosity": arg("verbosity")},
                ],
            ),
        ]
    )
