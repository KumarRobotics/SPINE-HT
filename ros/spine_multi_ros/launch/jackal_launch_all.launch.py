#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

from pathlib import Path
from datetime import datetime


def generate_launch_description():

    # declare args
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="",
        description="Namespace for the robot (empty for no namespace)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    should_bag_arg = DeclareLaunchArgument(
        "record", default_value="false", description="Use simulation time"
    )
    detection_confidence_arg = DeclareLaunchArgument(
        "detection_confidence", default_value="0.5", description="detection confidence"
    )

 

    # get pkgs and configs
    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    launch_nav2 = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "navigation_launch.py"]
    )

    # args for launching
    namespace = LaunchConfiguration("robot_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    record = LaunchConfiguration("record")
    detection_confidence = LaunchConfiguration("detection_confidence")

    nav2_config_name = ["nav2_", namespace, ".yaml"]

    nav2_config = PathJoinSubstitution([pkg_spine_multi, "config", nav2_config_name])
    jackal_static_transforms = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "jackal_static_transforms.launch.py"]
    )

    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution(
        [vision_pkg, "launch", "vision_jackal.launch.py"]
    )

    robot_map_frame = ["map_", namespace]

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration("robot_namespace")),
            IncludeLaunchDescription(
                jackal_static_transforms,
                launch_arguments=[
                    # ("use_sim_time", use_sim_time),
                    ("robot_map_frame", robot_map_frame)
                ],
            ),
            IncludeLaunchDescription(
                launch_nav2,
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("params_file", nav2_config),
                    ("namespace", namespace),
                    ("use_lifecycle_mgr", "false"),
                    ("use_namespace", "true"),
                    ("cmd_vel_topic", "platform/cmd_vel"),
                ],
            ),
            IncludeLaunchDescription(
                launch_vision,
                launch_arguments=[
                    ("input_rgb_topic", "zed/left/image_rect_color"),
                    ("input_depth_topic", "zed/depth/depth_registered"),
                    ("camera_info_topic", "zed/depth/camera_info"),
                    ("confidence", detection_confidence),
                ],
            ),
            Node(
                package="spine_multi_ros",  # Replace with your package name
                executable="twist_converter.py",
                name="twist_converter",
                remappings=[("cmd_vel", "autonomous/cmd_vel")],  # Remap output
            ),
            Node(
                package="safety_controller",
                executable="safety_controller",
                name="safety_controller",
            ),
            Node(
                package="spine_multi_ros",
                executable="nav_service_translator.py",
                name="nav_service_translator",
                parameters=[{"robot_name": namespace}],
            ),
            Node(
                package="spine_multi_ros",
                executable="vlm_service_translator.py",
                name="vlm_service_translator",
            ),
            Node(
                package="spine_multi_ros",
                executable="label_service_translator.py",
                name="label_service_translator",
            ),
        ],
    )

    launch_process = [
        robot_namespace_arg,
        use_sim_time_arg,
        should_bag_arg,
        detection_confidence_arg,
        namespaced_group,
    ]

    if record:
        record_launch_path = PathJoinSubstitution(
            [pkg_spine_multi, "launch", "record.launch.py"]
        )

        log_dir = Path.home() / "data/bags"
        log_dir.mkdir(exist_ok=True, parents=True)

        n_bags = len(list(log_dir.glob("*")))
        bag_name = str(
            log_dir / f"{n_bags:03d}_spine-multi-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}"
        )

        print(bag_name)

        launch_record = IncludeLaunchDescription(
            record_launch_path,
            launch_arguments=[
                ("namespace", namespace), 
                ("output_bag", bag_name)
            ]
        )

        launch_process.append(launch_record)

    return LaunchDescription(launch_process)
