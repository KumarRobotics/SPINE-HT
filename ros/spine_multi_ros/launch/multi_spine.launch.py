#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    robot_config = PathJoinSubstitution([pkg_spine_multi, "config", "spine_multi.yaml"])

    declare_run_bridge = DeclareLaunchArgument(
        "run_bridge", default_value="True", description="Run ros bridge"
    )

    run_bridge = LaunchConfiguration("run_bridge")

    callisto_bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge_callisto.yaml"]
    )

    io_bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge_io.yaml"]
    )

    bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge.yaml"]
    )

    ld = [
        declare_run_bridge,
        Node(
            package="spine_multi_ros",
            executable="spine_multi_node.py",
            name="spine_multi_node",
            output="screen",
            parameters=[robot_config],
        ),
    ]

    
    if run_bridge:
        robots_to_bridge = [
            # ("callisto", callisto_bridge_config),
            # ("io", io_bridge_config),
            ("all", bridge_config)
        ]

        for name, config_path in robots_to_bridge:
            ld.append(
                Node(
                    package="domain_bridge",
                    executable="domain_bridge",
                    name=f"domain_bridge_{name}",
                    output="screen",
                    arguments=[config_path],
                )
            )

    return LaunchDescription(ld)
