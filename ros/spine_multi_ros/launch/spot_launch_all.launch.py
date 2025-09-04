#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare



from pathlib import Path
from datetime import datetime


def generate_launch_description():
    # declare args
    declare_robot_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot (empty for no namespace)",
    )
    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="robot name"
    )
    declare_subscription_prefix_arg = DeclareLaunchArgument(
        "subscription_prefix",
        default_value="",
        description="prefix intra-robot subscriptions with this",
    )

    declare_should_bag_arg = DeclareLaunchArgument(
        "record", default_value="false", description="Use simulation time"
    )
    declare_detection_confidence_arg = DeclareLaunchArgument(
        "detection_confidence", default_value="0.5", description="detection confidence"
    )
    declare_use_mocha_arg = DeclareLaunchArgument(
        "use_mocha", default_value="false", description="use mocha"
    )
    declare_start_y_arg = DeclareLaunchArgument(
        "start_y", default_value="start_y", description="starting y"
    )

    # args for launching
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_mocha = LaunchConfiguration("use_mocha")
    record = LaunchConfiguration("record")
    detection_confidence = LaunchConfiguration("detection_confidence")
    subscription_prefix = LaunchConfiguration("subscription_prefix")
    start_y = LaunchConfiguration("start_y")

    # get pkgs and configs
    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    jackal_static_transforms = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "jackal_static_transforms.launch.py"]
    )
    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution(
        [vision_pkg, "launch", "vision_jackal.launch.py"]
    )

    robot_map_frame = ["map_", robot_name]

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
            'namespace': namespace
        }.items()
    )
 

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            #PushRosNamespace(LaunchConfiguration("namespace")),
             IncludeLaunchDescription(
                 jackal_static_transforms,
                 launch_arguments=[
                     # ("use_sim_time", use_sim_time),
                     ("robot_map_frame", robot_map_frame),
                     ("start_y", start_y)
                 ],
            ),
            zed_launch,
            IncludeLaunchDescription(
                launch_vision,
                launch_arguments=[
                    ("input_rgb_topic", "/zed/zed_node/left/image_rect_color"),
                    ("input_depth_topic", "/zed/zed_node/depth/depth_registered"),
                    ("camera_info_topic", "/zed/zed_node/depth/camera_info"),
                    ("confidence", detection_confidence),
                ],
            ),
            Node(
                package="spine_multi_ros",
                executable="spot_autonomy_server.py",
                name="spot_autonomy_server",
                parameters=[{"subscription_prefix": subscription_prefix}]
            )
        ],
    )

    launch_process = [
        declare_robot_namespace_arg,
        declare_robot_name_arg,
        declare_use_mocha_arg,
        declare_should_bag_arg,
        declare_detection_confidence_arg,
        declare_subscription_prefix_arg,
        declare_start_y_arg,
        namespaced_group,
    ]

    record_launch_path = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "record_spot.launch.py"]
    )
    bag_name = get_log_dir()
    launch_record = IncludeLaunchDescription(
        record_launch_path,
        condition=IfCondition(record),
        launch_arguments=[
            ("namespace", namespace), 
            ("output_bag", bag_name)
        ]
    )
    launch_process.append(launch_record)

    pkg_mocha = get_package_share_directory("mocha_launch")
    mocha_launch_path = PathJoinSubstitution(
        [pkg_mocha, "launch", "jackal.launch.py"]
    )

    mocha_launch = IncludeLaunchDescription(
            mocha_launch_path,
            condition=IfCondition(use_mocha),
            launch_arguments=[
                ("robot_name", robot_name)
            ],
        )

    launch_process.append(mocha_launch)
 


    return LaunchDescription(launch_process)



def get_log_dir() -> str:
    log_dir = Path.home() / "data/bags"
    log_dir.mkdir(exist_ok=True, parents=True)

    n_bags = len(list(log_dir.glob("*")))
    bag_name = str(
        log_dir / f"{n_bags:03d}_spine-multi-{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}"
    )
    return bag_name

