"""sqrtVINS for the native OAK driver using bundled device calibration.

Inputs: /native_oak/{left,right}/image_undistorted, /autonomy/imu/data.
Launch the native camera separately; this launch starts the estimator only.
Legacy Python undistortion is opt-in and requires a matching config override.
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
            DeclareLaunchArgument("namespace", default_value="autonomy"),
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
                remappings=[("/sim/processed/vio", "/autonomy/processed_vio")],
                output="screen",
                parameters=[
                    {"config_path": arg("config"), "verbosity": arg("verbosity")},
                ],
            ),
        ]
    )
