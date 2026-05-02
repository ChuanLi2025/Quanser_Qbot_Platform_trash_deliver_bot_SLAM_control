import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    nav2_dir = get_package_share_directory("nav2_bringup")

    default_map = "/home/nvidia/857ChuanLi/maps/lab_map_v5.yaml"
    default_targets = os.path.join(package_dir, "config", "trash_targets_v5.yaml")
    params_file = os.path.join(package_dir, "config", "qbot_platform_slam_and_nav.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    targets_file_arg = DeclareLaunchArgument("targets_file", default_value=default_targets)
    target_name_arg = DeclareLaunchArgument("target_name", default_value="blue")
    startup_delay_arg = DeclareLaunchArgument("startup_delay_sec", default_value="22.0")
    wait_at_target_arg = DeclareLaunchArgument("wait_at_target_sec", default_value="10.0")
    turn_left_arg = DeclareLaunchArgument("turn_left_degrees", default_value="90.0")

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
        parameters=[
            {
                "use_imu_yaw": False,
                "imu_angular_velocity_scale": 0.970,
            }
        ],
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

    initial_pose_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="qbot_platform",
                executable="publish_home_initial_pose.py",
                name="publish_home_initial_pose_v5_turn_home",
                output="screen",
                parameters=[
                    {
                        "targets_file": LaunchConfiguration("targets_file"),
                        "home_name": "home",
                        "initial_pose_topic": "/initialpose",
                        "publish_count": 20,
                        "publish_period_sec": 0.5,
                    }
                ],
            )
        ],
    )

    turn_home_node = Node(
        package="qbot_platform",
        executable="target_wait_turn_home_node.py",
        name="target_turn_home_v5",
        output="screen",
        parameters=[
            {
                "targets_file": LaunchConfiguration("targets_file"),
                "home_name": "home",
                "target_name": LaunchConfiguration("target_name"),
                "status_topic": "/trash_mission_status",
                "startup_delay_sec": ParameterValue(
                    LaunchConfiguration("startup_delay_sec"),
                    value_type=float,
                ),
                "wait_at_target_sec": ParameterValue(
                    LaunchConfiguration("wait_at_target_sec"),
                    value_type=float,
                ),
                "turn_left_degrees": ParameterValue(
                    LaunchConfiguration("turn_left_degrees"),
                    value_type=float,
                ),
                "retry_period_sec": 2.0,
            }
        ],
    )

    return LaunchDescription(
        [
            map_arg,
            targets_file_arg,
            target_name_arg,
            startup_delay_arg,
            wait_at_target_arg,
            turn_left_arg,
            base_launch,
            lidar_tf_node,
            wheel_odom_node,
            localization_launch,
            navigation_launch,
            initial_pose_node,
            turn_home_node,
        ]
    )
