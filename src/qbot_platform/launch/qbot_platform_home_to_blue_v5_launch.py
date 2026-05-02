import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    bringup_launch = os.path.join(package_dir, "launch", "qbot_platform_map_nav_bringup_launch.py")
    targets_file = os.path.join(package_dir, "config", "trash_targets_v5.yaml")
    map_file = "/home/nvidia/857ChuanLi/maps/lab_map_v5.yaml"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_launch),
                launch_arguments={"map": map_file}.items(),
            ),

            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="qbot_platform",
                        executable="publish_home_initial_pose.py",
                        name="publish_home_initial_pose_v5",
                        output="screen",
                        parameters=[
                            {
                                "targets_file": targets_file,
                                "home_name": "home",
                                "initial_pose_topic": "/initialpose",
                                "publish_count": 20,
                                "publish_period_sec": 0.5,
                            }
                        ],
                    )
                ],
            ),

            TimerAction(
                period=22.0,
                actions=[
                    Node(
                        package="qbot_platform",
                        executable="navigate_to_target_node.py",
                        name="navigate_to_blue_v5",
                        output="screen",
                        parameters=[
                            {
                                "targets_file": targets_file,
                                "target_name": "blue",
                                "startup_delay_sec": 1.0,
                                "status_topic": "/trash_mission_status",
                            }
                        ],
                    )
                ],
            ),
        ]
    )
