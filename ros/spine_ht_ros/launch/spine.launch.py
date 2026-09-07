#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_GRAPH = "/home/dcist/dcist_ws/src/spine-ht/ros/spine_ht_ros/data/perch.json"


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="top level namespace"
    )

    declare_log_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="top level log"
    )

    declare_graph_cmd = DeclareLaunchArgument(
        "init_graph", default_value=DEFAULT_GRAPH, description="top level default graph"
    )

    log_level = LaunchConfiguration("log_level")
    init_graph = LaunchConfiguration("init_graph")
    namespace = LaunchConfiguration("namespace")

    load_nodes = GroupAction(
        actions=[
            Node(
                package="spine_ht_ros",
                executable="spine_node.py",
                name="spine_node",
                namespace=namespace,
                output="screen",
                respawn_delay=2.0,
                arguments=["--ros-args", "--log-level", log_level],
                remappings=[("tracks", "grounding_dino_node/tracks")],  # asume same ns
                parameters=[{"init_graph": init_graph}],
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_log_cmd)
    ld.add_action(declare_graph_cmd)
    ld.add_action(load_nodes)

    return ld
