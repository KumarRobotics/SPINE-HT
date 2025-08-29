#!/usr/bin/env python3

from launch.actions import ExecuteProcess, DeclareLaunchArgument
from pathlib import Path 
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declare_run_bridge = DeclareLaunchArgument(
        "use_bridge", default_value="False", description="Run ros bridge"
    )
    declare_run_mocha = DeclareLaunchArgument(
        "use_mocha", default_value="True", description="Run mocha"
    )

    run_bridge = LaunchConfiguration("use_bridge")
    run_mocha = LaunchConfiguration("use_mocha")

    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    robot_config = PathJoinSubstitution([pkg_spine_multi, "config", "spine_multi.yaml"])

    callisto_bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge_callisto.yaml"]
    )
    io_bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge_io.yaml"]
    )
    bridge_config = PathJoinSubstitution(
        [pkg_spine_multi, "config", "spine_bridge.yaml"]
    )

    pkg_mocha = get_package_share_directory("mocha_launch")
    mocha_launch_path = PathJoinSubstitution(
        [pkg_mocha, "launch", "basestation.launch.py"]
    )

    mocha_launch = IncludeLaunchDescription(
        mocha_launch_path,
        condition=IfCondition(run_mocha),
    )

    ld = [
        declare_run_bridge,
        declare_run_mocha,
        mocha_launch,
        Node(
            package="spine_multi_ros",
            executable="spine_multi_node.py",
            name="spine_multi_node",
            output="screen",
            parameters=[robot_config],
        ),
    ]

    # robots_to_bridge = [
    #     # ("callisto", callisto_bridge_config),
    #     # ("io", io_bridge_config),
    #     ("all", bridge_config)
    # ]

    # for name, config_path in robots_to_bridge:
    #     ld.append(
    #         Node(
    #             package="domain_bridge",
    #             executable="domain_bridge",
    #             name=f"domain_bridge_{name}",
    #             output="screen",
    #             condition=IfCondition(run_bridge),
    #             arguments=[config_path],
    #         )
    #     )


    ld.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-d", "60",
                "-a",
                "/tf",
                "/tf_static",
                "-o",
                get_log_dir(),
            ],
            output="screen",
        )
    )

    return LaunchDescription(ld)

def get_log_dir() -> str:
    # TODO duplicate code from jackal launch
    log_dir = Path.home() / "data/bags"
    log_dir.mkdir(exist_ok=True, parents=True)

    n_bags = len(list(log_dir.glob("*")))
    bag_name = str(
        log_dir / f"{n_bags:03d}_spine-multi-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}"
    )
    return bag_name

