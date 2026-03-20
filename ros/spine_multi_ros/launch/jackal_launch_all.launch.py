#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

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
        "tracker_n_dets", default_value="5", description="number of detections for a valid track"
    )
    declare_model_choice_arg = DeclareLaunchArgument(
        "model_choice", default_value="large", description="Model choice for Florence (base or large)"
    )
    declare_camera_transform_arg = DeclareLaunchArgument(
        "camera_transform",
        default_value="zed_camera",
        description="which camera transform to use",
    )
    declare_use_mocha_arg = DeclareLaunchArgument(
        "use_mocha", default_value="false", description="use mocha"
    )
    declare_start_y_arg = DeclareLaunchArgument(
        "start_y", default_value="0.0", description="starting y"
    )
    declare_start_x_arg = DeclareLaunchArgument(
        "start_x", default_value="0.0", description="starting x"
    )
    declare_ground_ground_z_threshold_arg = DeclareLaunchArgument(
        "ground_grid_z_threshold", default_value="2.5", description="z threshold"
    )



    # args for launching
    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mocha = LaunchConfiguration("use_mocha")
    record = LaunchConfiguration("record")
    scale_depth = LaunchConfiguration("scale_depth")
    detection_confidence = LaunchConfiguration("detection_confidence")
    tracker_n_dets = LaunchConfiguration("tracker_n_dets")
    model_choice = LaunchConfiguration("model_choice")
    camera_transform = LaunchConfiguration("camera_transform")
    subscription_prefix = LaunchConfiguration("subscription_prefix")
    start_y = LaunchConfiguration("start_y")
    start_x = LaunchConfiguration("start_x")
    ground_grid_z_threshold = LaunchConfiguration("ground_grid_z_threshold")
 

    # get pkgs and configs
    pkg_spine_multi = get_package_share_directory("spine_multi_ros")
    launch_nav2 = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "navigation_launch.py"]
    )

    nav2_config_name = ["nav2_jackal.yaml"]

    nav2_config = PathJoinSubstitution([pkg_spine_multi, "config", nav2_config_name])
    jackal_static_transforms = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "jackal_static_transforms.launch.py"]
    )

    vision_pkg = get_package_share_directory("vision_ros2")
    launch_vision = PathJoinSubstitution(
        [vision_pkg, "launch", "vision_jackal.launch.py"]
    )

    groundgrid_pkg = get_package_share_directory("groundgrid")
    launch_groundgrid = PathJoinSubstitution(
        [groundgrid_pkg, "launch", "ground_grid.launch.py"]
    )

    robot_map_frame = ["map_", robot_name]

    # launch everything in a namespace
    namespaced_group = GroupAction(
        actions=[
            IncludeLaunchDescription( # TODO should this be namespaced too ?
                launch_groundgrid,
                launch_arguments=[
                    ("namespace", namespace),
                    ("z_threshold", ground_grid_z_threshold)
                ],
            ),
            #PushRosNamespace(LaunchConfiguration("namespace")),
            IncludeLaunchDescription(
                jackal_static_transforms,
                launch_arguments=[
                    # ("use_sim_time", use_sim_time),
                    ("robot_map_frame", robot_map_frame),
                    ("start_y", start_y),
                    ("start_x", start_x)
                ],
            ),
            Node(
                package="topic_tools",
                executable="throttle",
                name=f"throttle_costmap",
                arguments=["messages", "/local_costmap/costmap", "0.1", "/local_costmap/costmap_throttled" ],
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
                launch_arguments=[
                    ("input_rgb_topic", "/zed/zed_node/left/image_rect_color"),
                    ("input_depth_topic", "/zed/zed_node/depth/depth_registered"),
                    ("camera_info_topic", "/zed/zed_node/depth/camera_info"),
                    ("confidence", detection_confidence),
                    ("tracker_n_dets", tracker_n_dets),
                    ("model_choice", model_choice),
                    ("camera_transform", camera_transform),
                    ("scale_depth", scale_depth),
                    ("labels", "Persons and Vehicles"),
                ],
            ),
            Node(
                package="spine_multi_ros",  # Replace with your package name
                executable="twist_converter.py",
                name="twist_converter",
                remappings=[("cmd_vel", "autonomous/cmd_vel")],  # Remap output
                parameters=[{"scale_x": 3.0},
                            {"scale_z": 3.0}]
            ),
            Node(
                package="safety_controller",
                executable="safety_controller",
                name="safety_controller",
                remappings=[
                    ("joy_teleop/joy", [robot_name, "/joy_teleop/joy"]),
                    ("auto_mode/cmd_vel", [robot_name, "/auto_mode/cmd_vel"])
                ]
            ),
            Node(
                package="spine_multi_ros",
                executable="jackal_autonomy_server.py",
                name="jackal_autonomy_server",
                parameters=[{"subscription_prefix": subscription_prefix}]
            )
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

    # TODO clean up

    record_launch_path = PathJoinSubstitution(
        [pkg_spine_multi, "launch", "record_jackal.launch.py"]
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

