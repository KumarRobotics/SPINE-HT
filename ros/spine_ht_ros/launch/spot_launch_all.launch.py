#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
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

    declare_should_bag_arg = DeclareLaunchArgument(
        "record", default_value="false", description="Use simulation time"
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
        default_value="spot_camera",
        description="which camera transform to use",
    )
    declare_use_mocha_arg = DeclareLaunchArgument(
        "use_mocha", default_value="true", description="use mocha"
    )
    declare_scale_depth_arg = DeclareLaunchArgument(
        "scale_depth", default_value="True", description="scale depth images"
    )
    declare_start_y_arg = DeclareLaunchArgument(
        "start_y", default_value="0.0", description="starting y"
    )
    declare_start_x_arg = DeclareLaunchArgument(
        "start_x", default_value="0.0", description="starting y"
    )

    # args for launching
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_mocha = LaunchConfiguration("use_mocha")
    scale_depth = LaunchConfiguration("scale_depth")
    record = LaunchConfiguration("record")
    detection_confidence = LaunchConfiguration("detection_confidence")
    tracker_n_dets = LaunchConfiguration("tracker_n_dets")
    model_choice = LaunchConfiguration("model_choice")
    camera_transform = LaunchConfiguration("camera_transform")
    subscription_prefix = LaunchConfiguration("subscription_prefix")
    start_y = LaunchConfiguration("start_y")
    start_x = LaunchConfiguration("start_x")

    # get pkgs and configs
    pkg_spine_ht = get_package_share_directory("spine_ht_ros")
    jackal_static_transforms = PathJoinSubstitution(
        [pkg_spine_ht, "launch", "spot_static_transforms.launch.py"]
    )
    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution(
        [vision_pkg, "launch", "vision_clearpath.launch.py"]
    )
    spot_client_pkg = get_package_share_directory("spot_client_ros")
    spot_camera_launch = PathJoinSubstitution(
        [spot_client_pkg, "launch", "spot_sensors.launch.yaml"]
    )

    robot_map_frame = ["map_", robot_name]

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            SetRemap(src="/dlio/odom_node/odom", dst="/odom"),
            IncludeLaunchDescription(
                jackal_static_transforms,
                launch_arguments=[
                    ("robot_map_frame", robot_map_frame),
                    ("start_y", start_y),
                    ("start_x", start_x),
                ],
            ),
            IncludeLaunchDescription(spot_camera_launch),
            IncludeLaunchDescription(
                launch_vision,
                launch_arguments=[
                    ("input_rgb_topic", "/prometheus/frontleft/color/image_raw"),
                    ("input_depth_topic", "/prometheus/frontleft/depth/image_rect"),
                    ("camera_info_topic", "/prometheus/frontleft/color/camera_info"),
                    ("camera_frame", "promethues/frontleft"),
                    ("confidence", detection_confidence),
                    ("model_choice", model_choice),
                    ("camera_transform", camera_transform),
                    ("tracker_n_dets", tracker_n_dets),
                    ("scale_depth", scale_depth),
                    ("labels", "Persons and Vehicles"),
                ],
            ),
            Node(
                package="spine_ht_ros",
                executable="spot_autonomy_server.py",
                name="spot_autonomy_server",
                parameters=[{"subscription_prefix": subscription_prefix}],
            ),
        ],
    )

    launch_process = [
        declare_robot_namespace_arg,
        declare_robot_name_arg,
        declare_use_mocha_arg,
        declare_scale_depth_arg,
        declare_should_bag_arg,
        declare_camera_transform_arg,
        declare_detection_confidence_arg,
        declare_tracker_number_arg,
        declare_model_choice_arg,
        declare_subscription_prefix_arg,
        declare_start_y_arg,
        declare_start_x_arg,
        namespaced_group,
    ]

    record_launch_path = PathJoinSubstitution(
        [pkg_spine_ht, "launch", "record_spot.launch.py"]
    )
    bag_name = get_log_dir(user="dcist")
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


def get_log_dir(user: str) -> str:
    log_dir = Path(f"/home/{user}/data/bags")
    log_dir.mkdir(exist_ok=True, parents=True)
    n_bags = len(list(log_dir.glob("*")))
    datetime_now_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    bag_name = str(log_dir / f"{n_bags:03d}_spine-ht-{datetime_now_str}")
    return bag_name
