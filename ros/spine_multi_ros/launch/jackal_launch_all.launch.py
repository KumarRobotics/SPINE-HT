#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # declare args
    declare_robot_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the robot (empty for no namespace)",
    )
    declare_robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="", description="robot name"
    )
    declare_subscription_prefix_arg = DeclareLaunchArgument(
        "subscription_prefix",
        default_value="",
        description="prefix intra-robot subscriptions with this",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    declare_should_bag_arg = DeclareLaunchArgument(
        "record", default_value="false", description="Use simulation time"
    )
    declare_scale_depth_arg = DeclareLaunchArgument(
        "scale_depth", default_value="False", description="scale depth images"
    )
    declare_detection_confidence_arg = DeclareLaunchArgument(
        "detection_confidence", default_value="0.3", description="detection confidence"
    )
    declare_tracker_number_arg = DeclareLaunchArgument(
        "tracker_n_dets",
        default_value="5",
        description="number of detections for a valid track",
    )
    declare_model_choice_arg = DeclareLaunchArgument(
        "model_choice",
        default_value="large",
        description="Model choice for Florence (base or large)",
    )
    declare_camera_transform_arg = DeclareLaunchArgument(
        "camera_transform",
        default_value="zed_camera",
        description="which camera transform to use",
    )
    declare_use_mocha_arg = DeclareLaunchArgument(
        "use_mocha", default_value="true", description="use mocha"
    )
    declare_start_y_arg = DeclareLaunchArgument(
        "start_y", default_value="0.0", description="starting y"
    )
    declare_start_x_arg = DeclareLaunchArgument(
        "start_x", default_value="0.0", description="starting x"
    )
    declare_ground_ground_z_threshold_arg = DeclareLaunchArgument(
        "ground_grid_z_threshold",
        default_value="1.5",
        description="z threshold- all points in the ouster cloud above this height will be ignored by the ground grid (1.5 work best for tall scafoldings)",
    )

    # args for launching
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mocha = LaunchConfiguration("use_mocha")
    record = LaunchConfiguration("record")
    subscription_prefix = LaunchConfiguration("subscription_prefix")
    start_y = LaunchConfiguration("start_y")
    start_x = LaunchConfiguration("start_x")
    ground_grid_z_threshold = LaunchConfiguration("ground_grid_z_threshold")

    # get pkgs and configs
    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    launch_nav2 = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "navigation_launch.py"]
    )

    nav2_config_name = ["nav2_jackal_stvox.yaml"]

    nav2_config = PathJoinSubstitution([pkg_spine_multi, "config", nav2_config_name])
    clearpath_static_transforms = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "clearpath_static_transforms.launch.py"]
    )

    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution([vision_pkg, "launch", "vision.launch.py"])
    vision_config = PathJoinSubstitution([vision_pkg, "config", "jackal.yaml"])

    groundgrid_pkg = get_package_share_directory("groundgrid")
    launch_groundgrid = PathJoinSubstitution(
        [groundgrid_pkg, "launch", "ground_grid.launch.py"]
    )

    robot_map_frame = ["map_", robot_name]

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            IncludeLaunchDescription(  # TODO should this be namespaced too ?
                launch_groundgrid,
                launch_arguments=[
                    ("namespace", namespace),
                    ("z_threshold", ground_grid_z_threshold),
                ],
            ),
            IncludeLaunchDescription(
                clearpath_static_transforms,
                launch_arguments=[
                    # ("use_sim_time", use_sim_time),
                    ("robot_map_frame", robot_map_frame),
                    ("start_y", start_y),
                    ("start_x", start_x),
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name="throttle_costmap",
                arguments=[
                    "messages",
                    "/local_costmap/costmap",
                    "0.1",
                    "/local_costmap/costmap_throttled",
                ],
                output="screen",
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
                launch_arguments=[("config_file", vision_config)],
            ),
            Node(
                package="spine_multi_ros",
                executable="clearpath_autonomy_server.py",
                name="clearpath_autonomy_server",
                parameters=[{"subscription_prefix": subscription_prefix}],
            ),
            Node(
                package="spine_multi_ros",
                executable="convert_vel_to_joy.py",
                name="cmd_vel_to_joy",
            ),
            Node(
                package="spine_multi_ros",
                executable="goal_frame_converter.py",
                name="goal_frame_converter",
                output="screen",
                remappings=[
                    ("/odom/local", "/dlio/odom_node/odom"),
                    ("/odom/global", "/glider/odom"),
                ],
            ),
        ],
    )

    launch_process = [
        declare_robot_namespace_arg,
        declare_robot_name_arg,
        declare_use_sim_time_arg,
        declare_use_mocha_arg,
        declare_tracker_number_arg,
        declare_model_choice_arg,
        declare_camera_transform_arg,
        declare_scale_depth_arg,
        declare_should_bag_arg,
        declare_detection_confidence_arg,
        declare_subscription_prefix_arg,
        declare_start_y_arg,
        declare_start_x_arg,
        declare_ground_ground_z_threshold_arg,
        namespaced_group,
    ]

    record_launch_path = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "record_jackal.launch.py"]
    )
    bag_name = get_log_dir()
    launch_record = IncludeLaunchDescription(
        record_launch_path,
        condition=IfCondition(record),
        launch_arguments=[("namespace", namespace), ("output_bag", bag_name)],
    )
    launch_process.append(launch_record)

    pkg_mocha = get_package_share_directory("mocha_launch")
    mocha_launch_path = PathJoinSubstitution([pkg_mocha, "launch", "jackal.launch.py"])

    mocha_launch = IncludeLaunchDescription(
        mocha_launch_path,
        condition=IfCondition(use_mocha),
        launch_arguments=[("robot_name", robot_name)],
    )

    launch_process.append(mocha_launch)

    return LaunchDescription(launch_process)


def get_log_dir() -> str:
    log_dir = Path.home() / "data/bags"
    log_dir.mkdir(exist_ok=True, parents=True)

    n_bags = len(list(log_dir.glob("*")))
    datetime_now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_name = str(log_dir / f"{n_bags:03d}_spine-multi-{datetime_now_str}")
    return bag_name
