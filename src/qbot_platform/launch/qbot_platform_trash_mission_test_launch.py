import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    bringup_launch = os.path.join(package_dir, "launch", "qbot_platform_map_nav_bringup_launch.py")
    targets_file = os.path.join(package_dir, "config", "trash_targets.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch)),
            Node(
                package="qbot_platform",
                executable="blue_roundtrip_nav_node.py",
                name="blue_roundtrip_nav_node",
                output="screen",
                parameters=[
                    {
                        "targets_file": targets_file,
                        "status_topic": "/trash_mission_status",
                        "dump_command_topic": "/dump_trash_cmd",
                        "home_name": "home",
                        "blue_name": "blue",
                        "startup_delay_sec": 12.0,
                        "post_blue_wait_sec": 2.0,
                        "turnaround_degrees": 180.0,
                    }
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="qbot_platform",
                        executable="publish_home_initial_pose.py",
                        name="publish_home_initial_pose_test",
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
        ]
    )
