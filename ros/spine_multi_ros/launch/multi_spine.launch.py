#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    robot_config = PathJoinSubstitution([pkg_spine_multi, "config", "spine_multi.yaml"])

    return LaunchDescription(
        [
            Node(
                package="spine_multi_ros",
                executable="spine_multi_node.py",
                name="spine_multi_node",
                output="screen",
                parameters=[robot_config],
            )
        ]
    )
