#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )
    
    # Get launch configuration
    namespace = LaunchConfiguration('namespace')
    
    # Create namespaced topic names for DLIO
    pointcloud_topic = PythonExpression([
        "'", namespace, "/ouster/points' if '", namespace, "' else '/ouster/points'"
    ])

    imu_topic = PythonExpression([
        "'", namespace, "/ouster/imu' if '", namespace, "' else '/ouster/imu'"
    ])
    container_name = PythonExpression([
        "'", namespace, "/ouster/os_container' if '", namespace, "' else '/ouster/os_container'"
    ])
    ouster_namespace = PythonExpression([
        "'", namespace, "/ouster' if '", namespace, "' else 'ouster'"
    ])

    pointcloud_topic_fix = PathJoinSubstitution([namespace, '/ouster/points'])

    
    # Ouster LiDAR launch
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ouster_ros'),
                'launch',
                'sensor.composite.launch.py'
            ])
        ]),
        launch_arguments={
            'viz': 'false',
            'ouster_ns': ouster_namespace
        }.items()
    )
    
    # ZED Camera launch
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
    
    # DLIO launch
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('direct_lidar_inertial_odometry'),
                'launch',
                'dlio.launch.py'
            ])
        ]),
        launch_arguments={
            'rviz': 'false',
            'namespace': namespace,
            'pointcloud_topic': 'ouster/points',
            'imu_topic': 'ouster/imu',
            'container_name': container_name,
        }.items()
    )

    
    return LaunchDescription([
        namespace_arg,
        ouster_launch,
        zed_launch,
        dlio_launch
    ])
