import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory("qbot_platform")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "qbot_platform_launch.py")
        )
    )

    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/color_image/compressed"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("model", default_value="nvidia/nemotron-3-nano-omni-30b-a3b-reasoning:free"),
        DeclareLaunchArgument("attempts", default_value="3"),
        DeclareLaunchArgument("target_distance_m", default_value="0.30"),
        DeclareLaunchArgument("linear_speed", default_value="0.10"),
        DeclareLaunchArgument("distance_tolerance_m", default_value="0.01"),
        DeclareLaunchArgument("inter_attempt_delay_sec", default_value="10.0"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.65"),
        DeclareLaunchArgument("max_frame_age_sec", default_value="3.0"),
        DeclareLaunchArgument("max_odom_age_sec", default_value="1.0"),
        DeclareLaunchArgument("max_move_time_sec", default_value="6.0"),
        DeclareLaunchArgument("max_tokens", default_value="2048"),
        DeclareLaunchArgument("reasoning_max_tokens", default_value="256"),
        DeclareLaunchArgument("reasoning_effort", default_value="low"),
        DeclareLaunchArgument("exclude_reasoning", default_value="true"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("api_key_env", default_value="OPENROUTER_API_KEY"),
    ]

    camera_node = Node(
        package="qbot_platform",
        executable="rgbd",
        name="rgbd_camera",
        output="screen",
    )

    odom_node = Node(
        package="qbot_platform",
        executable="wheel_odometry.py",
        name="wheel_odometry",
        output="screen",
        parameters=[
            {
                "odom_topic": LaunchConfiguration("odom_topic"),
            }
        ],
    )

    agent_node = Node(
        package="qbot_platform",
        executable="vision_forward_agent.py",
        name="vision_forward_agent",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "model": LaunchConfiguration("model"),
                "attempts": ParameterValue(LaunchConfiguration("attempts"), value_type=int),
                "target_distance_m": ParameterValue(LaunchConfiguration("target_distance_m"), value_type=float),
                "linear_speed": ParameterValue(LaunchConfiguration("linear_speed"), value_type=float),
                "distance_tolerance_m": ParameterValue(LaunchConfiguration("distance_tolerance_m"), value_type=float),
                "inter_attempt_delay_sec": ParameterValue(LaunchConfiguration("inter_attempt_delay_sec"), value_type=float),
                "confidence_threshold": ParameterValue(LaunchConfiguration("confidence_threshold"), value_type=float),
                "max_frame_age_sec": ParameterValue(LaunchConfiguration("max_frame_age_sec"), value_type=float),
                "max_odom_age_sec": ParameterValue(LaunchConfiguration("max_odom_age_sec"), value_type=float),
                "max_move_time_sec": ParameterValue(LaunchConfiguration("max_move_time_sec"), value_type=float),
                "max_tokens": ParameterValue(LaunchConfiguration("max_tokens"), value_type=int),
                "reasoning_max_tokens": ParameterValue(LaunchConfiguration("reasoning_max_tokens"), value_type=int),
                "reasoning_effort": LaunchConfiguration("reasoning_effort"),
                "exclude_reasoning": ParameterValue(LaunchConfiguration("exclude_reasoning"), value_type=bool),
                "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                "api_key_env": LaunchConfiguration("api_key_env"),
            }
        ],
    )

    return LaunchDescription(args + [base_launch, camera_node, odom_node, agent_node])
