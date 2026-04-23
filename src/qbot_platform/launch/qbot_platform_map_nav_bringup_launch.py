import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    nav2_dir = get_package_share_directory("nav2_bringup")

    default_map = "/home/nvidia/857ChuanLi/maps/lab_map.yaml"
    params_file = os.path.join(package_dir, "config", "qbot_platform_slam_and_nav.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir, "launch", "qbot_platform_launch.py"))
    )

    lidar_tf_node = Node(
        package="qbot_platform",
        executable="fixed_lidar_frame",
        name="fixed_lidar_frame",
        output="screen",
    )

    wheel_odom_node = Node(
        package="qbot_platform",
        executable="wheel_odometry.py",
        name="wheel_odometry",
        output="screen",
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "localization_launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": "False",
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "navigation_launch.py")),
        launch_arguments={
            "use_sim_time": "False",
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            map_arg,
            base_launch,
            lidar_tf_node,
            wheel_odom_node,
            localization_launch,
            navigation_launch,
        ]
    )
