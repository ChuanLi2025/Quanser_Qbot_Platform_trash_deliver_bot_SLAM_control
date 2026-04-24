#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class BlueRoundtripNavNode(Node):
    def __init__(self):
        super().__init__("blue_roundtrip_nav_node")

        default_targets = f"{get_package_share_directory('qbot_platform')}/config/trash_targets.yaml"
        self.declare_parameter("targets_file", default_targets)
        self.declare_parameter("status_topic", "/trash_mission_status")
        self.declare_parameter("dump_command_topic", "/dump_trash_cmd")
        self.declare_parameter("home_name", "home")
        self.declare_parameter("blue_name", "blue")
        self.declare_parameter("startup_delay_sec", 12.0)
        self.declare_parameter("post_blue_wait_sec", 2.0)
        self.declare_parameter("turnaround_degrees", 180.0)

        self.targets_file = self.get_parameter("targets_file").value
        self.status_topic = self.get_parameter("status_topic").value
        self.dump_command_topic = self.get_parameter("dump_command_topic").value
        self.home_name = self.get_parameter("home_name").value
        self.blue_name = self.get_parameter("blue_name").value
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.post_blue_wait_sec = float(self.get_parameter("post_blue_wait_sec").value)
        self.turnaround_degrees = float(self.get_parameter("turnaround_degrees").value)

        self.targets = self._load_targets(self.targets_file)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.dump_pub = self.create_publisher(String, self.dump_command_topic, 10)

        self._phase = "startup_wait"
        self._active_goal_name = None
        self._wait_timer = self.create_timer(self.startup_delay_sec, self._start_sequence)

        self._publish_status("roundtrip_waiting_to_start")
        self.get_logger().info("Blue roundtrip nav node ready")

    def _load_targets(self, filename):
        path = Path(filename)
        if not path.exists():
            raise FileNotFoundError(f"Targets file not found: {filename}")
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        required = {self.home_name, self.blue_name}
        missing = [name for name in required if name not in data]
        if missing:
            raise RuntimeError(f"Missing targets in {filename}: {', '.join(missing)}")
        return data

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def _publish_dump_command(self, target_name: str):
        msg = String()
        msg.data = f"start_{target_name}"
        self.dump_pub.publish(msg)
        self.get_logger().info(f"dump_command={msg.data}")

    def _start_sequence(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None
        self._phase = "go_to_blue"
        self._publish_status("roundtrip_navigating_to_blue")
        self._send_named_goal(self.blue_name)

    def _send_named_goal(self, target_name: str, yaw_override: float = None):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self._publish_status("roundtrip_error_no_nav_server")
            self.get_logger().error("NavigateToPose action server is not available")
            return

        target = dict(self.targets[target_name])
        if yaw_override is not None:
            target["yaw"] = yaw_override

        self._active_goal_name = target_name
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(target)
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _build_pose(self, target):
        pose = PoseStamped()
        pose.header.frame_id = target.get("frame_id", "map")
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target["x"])
        pose.pose.position.y = float(target["y"])
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(float(target["yaw"]))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status(f"roundtrip_goal_rejected_{self._phase}")
            self.get_logger().error(f"Goal rejected during phase {self._phase}")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f"roundtrip_navigation_failed_{self._phase}_{status}")
            self.get_logger().warning(
                f"Navigation failed during phase {self._phase} with status {status}"
            )
            return

        if self._phase == "go_to_blue":
            self._publish_status("roundtrip_arrived_blue")
            self._publish_dump_command(self.blue_name)
            self._phase = "turnaround_wait"
            self._wait_timer = self.create_timer(self.post_blue_wait_sec, self._start_turnaround)
            return

        if self._phase == "turnaround":
            self._publish_status("roundtrip_turned_around")
            self._phase = "return_home"
            self._publish_status("roundtrip_returning_home")
            self._send_named_goal(self.home_name)
            return

        if self._phase == "return_home":
            self._publish_status("roundtrip_arrived_home")
            return

    def _start_turnaround(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

        blue_target = self.targets[self.blue_name]
        turnaround_yaw = normalize_angle(
            float(blue_target["yaw"]) + math.radians(self.turnaround_degrees)
        )
        self._phase = "turnaround"
        self._publish_status("roundtrip_turning_180_at_blue")
        self._send_named_goal(self.blue_name, yaw_override=turnaround_yaw)


def main():
    rclpy.init()
    node = None
    try:
        node = BlueRoundtripNavNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
