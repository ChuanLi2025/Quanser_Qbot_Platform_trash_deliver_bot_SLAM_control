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


class RouteToTargetsNode(Node):
    def __init__(self):
        super().__init__("route_to_targets_node")

        default_targets = f"{get_package_share_directory('qbot_platform')}/config/trash_targets_v5.yaml"
        self.declare_parameter("targets_file", default_targets)
        self.declare_parameter("route_names", "blue,black,home")
        self.declare_parameter("wait_target_name", "blue")
        self.declare_parameter("wait_at_target_sec", 10.0)
        self.declare_parameter("status_topic", "/trash_mission_status")
        self.declare_parameter("startup_delay_sec", 22.0)
        self.declare_parameter("retry_period_sec", 2.0)

        self.targets_file = self.get_parameter("targets_file").value
        self.route_names = self._parse_route(self.get_parameter("route_names").value)
        self.wait_target_name = self.get_parameter("wait_target_name").value.strip().lower()
        self.wait_at_target_sec = float(self.get_parameter("wait_at_target_sec").value)
        self.status_topic = self.get_parameter("status_topic").value
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.retry_period_sec = float(self.get_parameter("retry_period_sec").value)

        self.targets = self._load_targets(self.targets_file)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self._route_index = 0
        self._active_goal_name = None
        self._timer = self.create_timer(self.startup_delay_sec, self._send_next_goal)

        self._publish_status(f"route_waiting_to_start_{'_'.join(self.route_names)}")
        self.get_logger().info(
            f"Will follow route {self.route_names} using targets file {self.targets_file}"
        )

    @staticmethod
    def _parse_route(route_text):
        route = [name.strip().lower() for name in str(route_text).split(",") if name.strip()]
        if not route:
            raise RuntimeError("route_names must contain at least one target")
        return route

    def _load_targets(self, filename):
        path = Path(filename)
        if not path.exists():
            raise FileNotFoundError(f"Targets file not found: {filename}")

        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        missing = [name for name in self.route_names if name not in data]
        if missing:
            raise RuntimeError(f"Missing targets in {filename}: {', '.join(missing)}")

        return data

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def _send_next_goal(self):
        self._cancel_timer()

        if self._route_index >= len(self.route_names):
            self._publish_status("route_complete")
            self.get_logger().info("Route complete")
            return

        target_name = self.route_names[self._route_index]
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self._publish_status("route_waiting_for_nav2_action_server")
            self._timer = self.create_timer(self.retry_period_sec, self._send_next_goal)
            return

        self._active_goal_name = target_name
        self._publish_status(f"route_navigating_to_{target_name}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(self.targets[target_name])
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_cb)

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

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
            self._publish_status(f"route_goal_rejected_{self._active_goal_name}")
            self.get_logger().error(f"Goal to '{self._active_goal_name}' was rejected")
            return

        self._publish_status(f"route_goal_accepted_{self._active_goal_name}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(f"route_navigation_failed_{self._active_goal_name}_{status}")
            self.get_logger().warning(
                f"Navigation to '{self._active_goal_name}' ended with status {status}"
            )
            return

        arrived_name = self._active_goal_name
        self._publish_status(f"route_arrived_{arrived_name}")
        self._route_index += 1

        if arrived_name == self.wait_target_name and self.wait_at_target_sec > 0.0:
            self._publish_status(f"route_waiting_{self.wait_at_target_sec:.1f}_sec_at_{arrived_name}")
            self._timer = self.create_timer(self.wait_at_target_sec, self._send_next_goal)
            return

        self._timer = self.create_timer(0.5, self._send_next_goal)


def main():
    rclpy.init()
    node = None
    try:
        node = RouteToTargetsNode()
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
