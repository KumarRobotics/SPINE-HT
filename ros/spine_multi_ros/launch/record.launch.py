from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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

    return LaunchDescription(
        [
            declare_namespace_arg,
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    [namespace_arg, "/dlio/odom_node/odom"],
                    [namespace_arg, "/local_costmap/costmap"],
                    [namespace_arg, "/global_costmap/costmap"],
                    [namespace_arg, "/grounding_dino_node/detections"],
                    [namespace_arg, "/grounding_dino_node/detection_img"],
                    [namespace_arg, "/cmd_vel"],
                    [namespace_arg, "/grounding_dino_node/detections_marker"],
                    [namespace_arg, "/goal_pose"],
                    [namespace_arg, "/plan"],
                    "/tf",
                    "/tf_static",
                    "-o",
                    output_arg,
                ],
                output="screen",
            ),
        ]
    )
