import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    base_launch = os.path.join(package_dir, "launch", "qbot_platform_launch.py")

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch)),
            Node(
                package="qbot_platform",
                executable="blue_route_demo.py",
                name="blue_route_demo",
                output="screen",
                parameters=[
                    {
                        "forward_1_sec": 26.0,
                        "turn_left_sec": 3.0,
                        "forward_2_sec": 18.0,
                        "wait_at_blue_sec": 5.0,
                        "turn_back_sec": 6.0,
                        "return_1_sec": 18.0,
                        "turn_right_home_sec": 3.0,
                        "return_2_sec": 26.0,
                        "linear_speed": 0.20,
                        "angular_speed": 0.35,
                        "obstacle_distance": 0.9,
                        "front_arc_degrees": 140.0,
                        "front_sector_count": 5,
                        "avoid_backup_sec": 0.8,
                        "avoid_turn_sec": 1.6,
                        "avoid_forward_sec": 1.4,
                        "avoid_realign_sec": 1.2,
                    }
                ],
            ),
        ]
    )
