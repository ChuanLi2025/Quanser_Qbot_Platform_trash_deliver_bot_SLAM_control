import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    base_launch = os.path.join(package_dir, "launch", "qbot_platform_launch.py")
    targets_file = os.path.join(package_dir, "config", "trash_targets.yaml")

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
            Node(
                package="qbot_platform",
                executable="trash_mission_test_node.py",
                name="trash_mission_test_node",
                output="screen",
                parameters=[
                    {
                        "targets_file": targets_file,
                        "command_topic": "/trash_target",
                        "status_topic": "/trash_mission_status",
                        "dump_command_topic": "/dump_trash_cmd",
                        "home_name": "home",
                        "task_wait_seconds": 20.0,
                        "travel_time_blue": 18.0,
                        "travel_time_black": 20.0,
                        "travel_time_home": 16.0,
                        "obstacle_distance": 0.75,
                        "backup_time": 1.2,
                        "turn_time": 1.5,
                        "front_sector_degrees": 180.0,
                        "front_sector_count": 5,
                    }
                ],
            ),
            TimerAction(
                period=2.0,
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
                                "publish_count": 5,
                                "publish_period_sec": 0.5,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
