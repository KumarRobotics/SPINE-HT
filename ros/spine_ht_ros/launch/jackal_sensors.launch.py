#!/usr/bin/env python3

import os

import ament_index_python.packages
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer, SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# TODO quickfix. Needs to find if there's a cleaner way to do this
def get_ublox_launch():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory("ublox_gps"), "config"
    )
    param_config = os.path.join(config_directory, "c94_m8p_rover.yaml")
    with open(param_config, "r") as f:
        params = yaml.safe_load(f)["ublox_gps_node"]["ros__parameters"]
    container = ComposableNodeContainer(
        name="ublox_gps_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_gps",
                plugin="ublox_node::UbloxNode",
                name="ublox_gps_node",
                parameters=[params],
            ),
        ],
        output="both",
    )
    return container


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for all nodes"
    )

    # Get launch configuration
    namespace = LaunchConfiguration("namespace")

    container_name = PythonExpression(
        [
            "'",
            namespace,
            "/ouster/os_container' if '",
            namespace,
            "' else '/ouster/os_container'",
        ]
    )
    ouster_namespace = PythonExpression(
        ["'", namespace, "/ouster' if '", namespace, "' else 'ouster'"]
    )

    # Ouster LiDAR launch
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ouster_ros"),
                        "launch",
                        "sensor.composite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"viz": "false", "ouster_ns": ouster_namespace}.items(),
    )

    # ZED Camera launch
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "camera_model": "zed2i",
            "publish_tf": "false",
            "publish_map_tf": "false",
            "namespace": namespace,
        }.items(),
    )

    # DLIO launch
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("direct_lidar_inertial_odometry"),
                        "launch",
                        "dlio.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "rviz": "false",
            "namespace": namespace,
            "pointcloud_topic": "ouster/points",
            "imu_topic": "ouster/imu",
            "container_name": container_name,
        }.items(),
    )

    # VectorNav Launch
    vectornav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("vectornav"), "launch", "vectornav.launch.py"]
                )
            ]
        )
    )

    # Ublox GPS Launch
    ublox_launch = get_ublox_launch()

    glider_launch = GroupAction(
        [
            SetRemap("/dgps", "/dgps/fix"),
            SetRemap("/gps", "/ublox_gps_node/fix"),
            SetRemap("/imu", "/vectornav/imu"),
            SetRemap("/odom", "/Odometry"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("glider"),
                                "launch",
                                "glider-node.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "config_file": PathJoinSubstitution(
                        [
                            FindPackageShare("spine_ht_ros"),
                            "config",
                            "glider-params.yaml",
                        ]
                    ),
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            namespace_arg,
            ouster_launch,
            zed_launch,
            vectornav_launch,
            ublox_launch,
            glider_launch,
            dlio_launch,
        ]
    )
