#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_map_frame_arg = DeclareLaunchArgument(
        "robot_map_frame", default_value="j100_0000", description="Name for the robot"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    start_y_arg = DeclareLaunchArgument(
        "start_y", default_value="0.0", description="Name for the robot"
    )
    start_x_arg = DeclareLaunchArgument(
        "start_x", default_value="0.0", description="Name for the robot"
    )

    robot_map_frame = LaunchConfiguration("robot_map_frame")
    start_y = LaunchConfiguration("start_y")
    start_x = LaunchConfiguration("start_x")

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    start_x,
                    "--y",
                    start_y,
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    robot_map_frame,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    robot_map_frame,
                    "--child-frame-id",
                    "odom",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "zed_camera_link",
                ],
            ),
        ],
    )

    return LaunchDescription(
        [
            robot_map_frame_arg,
            start_y_arg,
            start_x_arg,
            use_sim_time_arg,
            namespaced_group,
        ]
    )
