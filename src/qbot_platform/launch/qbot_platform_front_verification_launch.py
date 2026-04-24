import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")

    calibration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "qbot_platform_calibration_launch.py")
        )
    )

    motion_node = Node(
        package="qbot_platform",
        executable="calibration_motion_runner.py",
        name="front_verification_runner",
        output="screen",
        parameters=[
            {
                "mode": "straight",
                "target_distance_m": 1.0,
                "linear_speed": 0.30,
                "startup_delay_sec": 2.0,
            }
        ],
    )

    return LaunchDescription([calibration_launch, motion_node])
