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

    msg_throttles = [
        (
            [namespace_arg, "/zed/depth/depth_registered"],
            [namespace_arg, "/zed/throttled/depth/depth_registered"],
        ),
        (
            [namespace_arg, "/zed/rgb/image_rect_color/compressed"],
            [namespace_arg, "/zed/throttled/rgb/image_rect_color/compressed"],
        ),
        (
            [namespace_arg, "/zed/depth/depth_info"],
            [namespace_arg, "/zed/throttled/depth/depth_info"],
        ),
    ]

    ld = [declare_namespace_arg]

    for idx, (orig_topic, throttle_topic) in enumerate(msg_throttles):
        ld.append(
            Node(
                package="topic_tools",
                executable="throttle",
                name=f"throttle_{idx}",
                arguments=["messages", orig_topic, "1.0", throttle_topic],
                output="screen",
            )
        )

    # image_compressor_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='image_compressor',
    #     arguments=[
    #         'raw', 'compressed',
    #         '--ros-args',
    #         '-r', ['in:=', namespace_arg, "grounding_dino_node/detection_img"],
    #         '-r', ['out:=', namespace_arg, "grounding_dino_node/detection_img/compressed"],
    #         '-p', ['compressed.jpeg_quality:=80']
    #     ],
    #     output='screen'
    # )

    ld.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                [namespace_arg, "/dlio/odom_node/odom"],
                [namespace_arg, "/zed/throttled/depth/depth_registered/compressed"],
                [namespace_arg, "/zed/throttled/rgb/image_rect_color/compressed"],
                [namespace_arg, "/zed/throttled/depth/depth_info"],
                [namespace_arg, "/local_costmap/costmap"],
                [namespace_arg, "/global_costmap/costmap"],
                [namespace_arg, "/grounding_dino_node/detections"],
                [namespace_arg, "/grounding_dino_node/detection_img"],
                [namespace_arg, "/cmd_vel"],
                [namespace_arg, "/grounding_dino_node/detections_marker"],
                [namespace_arg, "/goal_pose"],
                [namespace_arg, "/plan"],
                [namespace_arg, "/local_plan"],
                [namespace_arg, "/local_costmap/published_footprint"],
                [namespace_arg, "/graph_viz"],
                "/tf",
                "/tf_static",
                "-o",
                output_arg,
            ],
            output="screen",
        )
    )

    return LaunchDescription(ld)
