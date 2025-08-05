#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # declare args
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Namespace for the robot (empty for no namespace)",
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="warthog1", description="Name for the robot"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    # get pkgs and configs
    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    launch_nav2 = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "navigation_launch.py"]
    )
    nav2_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "nav2_sim_warthog.yaml"]
    )

    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution([vision_pkg, "launch", "vision_sim.launch.py"])

    # args for launching
    namespace = LaunchConfiguration("robot_namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("spine_multi_ros"),
                                "launch",
                                "tf_odom_broadcast.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "odom_topic": ["/", robot_name, "/platform/odom"],
                    "parent_frame": [robot_name, "/odom"],
                    "child_frame": [robot_name, "/base_link"],
                    "node_name": "odom_tf_broadcaster",
                }.items(),
            ),
            IncludeLaunchDescription(
                launch_nav2,
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("params_file", nav2_config),
                    ("namespace", namespace),
                    ("use_lifecycle_mgr", "false"),
                    ("use_namespace", "true"),
                ],
            ),
            IncludeLaunchDescription(launch_vision, launch_arguments=[]),
            Node(
                package="spine_multi_ros",  # Replace with your package name
                executable="twist_converter.py",
                name="twist_converter",
                parameters=[{"publish_stamped": False}],
                remappings=[("/cmd_vel", "/warthog1/cmd_vel")],  # Remap output
            ),
        ],
    )

    return LaunchDescription(
        [robot_namespace_arg, robot_name_arg, use_sim_time_arg, namespaced_group]
    )
