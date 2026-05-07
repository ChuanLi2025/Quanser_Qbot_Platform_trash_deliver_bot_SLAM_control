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

    return LaunchDescription([
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
                    name="publish_home_initial_pose_v5_black_roundtrip",
                    output="screen",
                    parameters=[{
                        "targets_file": targets_file,
                        "home_name": "home",
                        "publish_count": 20,
                        "publish_period_sec": 0.5,
                    }],
                )
            ],
        ),

        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package="qbot_platform",
                    executable="roundtrip_to_target_node.py",
                    name="black_roundtrip_v5",
                    output="screen",
                    parameters=[{
                        "targets_file": targets_file,
                        "home_name": "home",
                        "target_name": "black",
                        "startup_delay_sec": 1.0,
                        "wait_at_target_sec": 10.0,
                        "post_turn_settle_sec": 2.5,
                        "use_front_route_anchor": True,
                        "front_route_distance": 2.2,
                        "front_route_lateral_offset": 0.0,
                        "status_topic": "/trash_mission_status",
                        "led_topic": "/qbot_led_strip",
                        "wait_flash_hz": 2.0,
                        "task_led_r": 0.35,
                        "task_led_g": 0.0,
                        "task_led_b": 0.75,
                    }],
                )
            ],
        ),
    ])
