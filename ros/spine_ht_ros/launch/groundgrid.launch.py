#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="j100_0000",
        description="Namespace for all nodes and topics",
    )

    # Get launch configuration
    namespace = LaunchConfiguration("namespace")

    # Define the groundgrid composable node
    groundgrid_component = ComposableNode(
        package="groundgrid",
        plugin="groundgrid::GroundGridNode",
        name="groundgrid_node",
        namespace=namespace,
        remappings=[
            ("points", "ouster/points"),
        ],
        parameters=[],
    )

    # Load component into existing ouster container
    load_composable_nodes = LoadComposableNodes(
        target_container=[namespace, "/ouster/os_container"],
        composable_node_descriptions=[
            groundgrid_component,
        ],
    )

    return LaunchDescription(
        [
            namespace_arg,
            load_composable_nodes,
        ]
    )
