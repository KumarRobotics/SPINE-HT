from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="j100_0000",
        description="Namespace for all recorded topics",
    )

    declare_output_arg = DeclareLaunchArgument(
        "output_bag",
        default_value="/home/dcist/data/bags",
        description="Namespace for all recorded topics",
    )

    namespace_arg = LaunchConfiguration("namespace")
    output_arg = LaunchConfiguration("output_bag")

    ld = [declare_namespace_arg]
    ld.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-d", "60",
                [namespace_arg, "/grounding_dino_node/detections"],
                [namespace_arg, "/grounding_dino_node/detection_img"],
                [namespace_arg, "/grounding_dino_node/detections_marker"],
                [namespace_arg, "/grounding_dino_node/track_markers"],
                [namespace_arg, "/grounding_dino_node/tracks"],
                [namespace_arg, "/graph_viz"],
                [namespace_arg, "/behavior_request_ack"],
                [namespace_arg, "/behavior_result"],
                [namespace_arg, "/prometheus/frontleft/color/camera_info"],
                [namespace_arg, "/prometheus/frontleft/color/image_raw/compressed"],
                [namespace_arg, "/prometheus/frontleft/depth/camera_info"],
                [namespace_arg, "/prometheus/frontleft/depth/image_rect"],
                "/tf",
                "/tf_static",
                "/odom",
                "-o",
                output_arg,
            ],
            output="screen",
        )
    )

    return LaunchDescription(ld)
