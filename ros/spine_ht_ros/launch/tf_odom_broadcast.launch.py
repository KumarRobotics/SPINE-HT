from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/warthog1/platform/odom",
        description="Input odometry topic",
    )

    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="warthog1/odom",
        description="Parent frame for transform",
    )

    child_frame_arg = DeclareLaunchArgument(
        "child_frame",
        default_value="warthog1/base_link",
        description="Child frame for transform",
    )

    node_name_arg = DeclareLaunchArgument(
        "node_name", default_value="odom_tf_broadcaster", description="Name of the node"
    )

    # Node with configurable remapping
    odom_tf_broadcaster_node = Node(
        package="spine_ht_ros",
        executable="odom_tf_broadcaster",
        name=LaunchConfiguration("node_name"),
        parameters=[
            {"parent_frame": LaunchConfiguration("parent_frame")},
            {"child_frame": LaunchConfiguration("child_frame")},
        ],
        remappings=[
            ("/odom", LaunchConfiguration("odom_topic")),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            odom_topic_arg,
            parent_frame_arg,
            child_frame_arg,
            node_name_arg,
            odom_tf_broadcaster_node,
        ]
    )
