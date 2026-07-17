from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    use_rviz = LaunchConfiguration("use_rviz")

    config_pkg_path = get_package_share_directory("config_pkg")
    rviz_config = os.path.join(
        config_pkg_path,
        "config",
        "vins_euroc_rviz.rviz",
    )

    feature_tracker = Node(
        package="feature_tracker",
        executable="feature_tracker",
        name="feature_tracker",
        namespace="feature_tracker",
        output="screen",
        parameters=[
            {
                "config_file": config_file,
                "vins_folder": config_pkg_path + os.sep,
            }
        ],
    )

    estimator = Node(
        package="vins_estimator",
        executable="vins_estimator",
        name="vins_estimator",
        namespace="vins_estimator",
        output="screen",
        parameters=[
            {
                "config_file": config_file,
                "vins_folder": config_pkg_path + os.sep,
            }
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                description="Absolute path to the UCC VINS YAML configuration.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            LogInfo(msg=["[UCC VINS] config: ", config_file]),
            feature_tracker,
            estimator,
            rviz,
        ]
    )
