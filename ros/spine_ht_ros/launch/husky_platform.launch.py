from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_launch_platform_arg = DeclareLaunchArgument(
        "launch_platform",
        default_value="true",
        description="true -> launches serial connection to platform. false -> only launches sensors.",
    )

    declare_robot_ns_arg = DeclareLaunchArgument(
        "namespace",
        default_value="triton",
        description="namespace of robot. Used for formatting simulation topic names to match real hardware.",
    )

    declare_setup_path_arg = DeclareLaunchArgument(
        "setup_path",
        default_value="/etc/clearpath",
        description="path to robot.yaml file",
    )

    declare_launch_dlio_arg = DeclareLaunchArgument(
        "launch_dlio", default_value="true", description="true -> launches dli"
    )
    declare_ground_ground_z_threshold_arg = DeclareLaunchArgument(
        "ground_grid_z_threshold", default_value="2.5", description="z threshold"
    )

    namespace = LaunchConfiguration("namespace")
    launch_dlio = LaunchConfiguration("launch_dlio")
    ground_grid_z_threshold = LaunchConfiguration("ground_grid_z_threshold")

    ouster_namespace = PythonExpression(
        ["'", namespace, "/ouster' if '", namespace, "' else 'ouster'"]
    )

    container_name = PythonExpression(
        [
            "'",
            namespace,
            "/ouster/os_container' if '",
            namespace,
            "' else '/ouster/os_container'",
        ]
    )

    # Launch the hardware platform if use_sim argument is false
    platform_launch_path = "/etc/clearpath/platform/launch/platform-service.launch.py"
    platform_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(platform_launch_path),
        condition=IfCondition(LaunchConfiguration("launch_platform")),
    )

    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ouster_ros"),
                        "launch",
                        "sensor.composite.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"viz": "false", "ouster_ns": ouster_namespace}.items(),
    )

    # DLIO launch
    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("direct_lidar_inertial_odometry"),
                        "launch",
                        "dlio.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(launch_dlio),
        launch_arguments={
            "rviz": "false",
            "namespace": namespace,
            "pointcloud_topic": "ouster/points",
            "imu_topic": "ouster/imu",
            "container_name": container_name,
        }.items(),
    )

    groundgrid_pkg = get_package_share_directory("groundgrid")

    launch_groundgrid_path = PathJoinSubstitution(
        [groundgrid_pkg, "launch", "ground_grid.launch.py"]
    )

    launch_groundgrid = (
        IncludeLaunchDescription(  # TODO should this be namespaced too ?
            launch_groundgrid_path,
            launch_arguments={
                "namespace": namespace,
                "z_threshold": ground_grid_z_threshold,
            }.items(),
        )
    )

    return LaunchDescription(
        [
            declare_setup_path_arg,
            declare_robot_ns_arg,
            declare_launch_platform_arg,
            declare_launch_dlio_arg,
            declare_ground_ground_z_threshold_arg,
            launch_groundgrid,
            # vn100_include,
            # ublox_node,
            ouster_launch,
            dlio_launch,
            platform_include,
            # zed_launch,
        ]
    )
