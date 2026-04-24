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
        name="rotation_verification_runner",
        output="screen",
        parameters=[
            {
                "mode": "rotate",
                "target_rotation_deg": 360.0,
                "angular_speed": 0.50,
                "scan_summary_output_file": "/home/nvidia/857ChuanLi/rotation_scan_summary.yaml",
                "startup_delay_sec": 2.0,
            }
        ],
    )

    return LaunchDescription([calibration_launch, motion_node])
