from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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

    output_arg = LaunchConfiguration("output_bag")

    ld = [declare_namespace_arg, declare_output_arg]

    # for idx, (orig_topic, throttle_topic) in enumerate(msg_throttles):
    #     ld.append(
    #         Node(
    #             package="topic_tools",
    #             executable="throttle",
    #             name=f"throttle_{idx}",
    #             arguments=["messages", orig_topic, "1.0", throttle_topic],
    #             output="screen",
    #         )
    #     )

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
                ["/dlio/odom_node/odom"],
                ["/dlio/odom_node/path"],
                ["/groundgrid/obstacle_cloud"],
                ["/groundgrid/segmented_cloud"],
                ["/ouster/points"],
                ["/ouster/lidar_packets"],
                ["/ouster/imu"],
                ["/dlio/odom_node/pointcloud/deskewed"],
                ["/zed/zed_node/left/image_rect_color/compressed"],
                ["/zed/zed_node/left/camera_info"],
                ["/zed/zed_node/depth/camera_info"],
                ["/zed/zed_node/depth/depth_registered"],
                # ["/detector_node/detection_img/front"],
                # ["/detector_node/detections_marker"],
                # ["/detector_node/track_markers"],
                # ["/detector_node/info"],
                # ["/callisto/graph_viz"],
                ["/ublox_gps_node/fix"],
                ["/glider/odom"],
                ["/glider/odom/viz"],
                ["/vectornav/imu"],
                # ["/global_costmap/costmap"],
                # ["/local_costmap/costmap"],
                ["/plan"],
                "/tf",
                "/tf_static",
                "-o",
                output_arg,
            ],
            output="screen",
        )
    )

    return LaunchDescription(ld)
