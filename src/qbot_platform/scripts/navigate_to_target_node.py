#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)

class NavigateToTargetNode(Node):
    def __init__(self):
        super().__init__("navigate_to_target_node")
        self.declare_parameter("targets_file", "/home/nvidia/857ChuanLi/src/qbot_platform/config/trash_targets_v5.yaml")
        self.declare_parameter("target_name", "blue")
        self.declare_parameter("startup_delay_sec", 22.0)
        self.declare_parameter("status_topic", "/trash_mission_status")

        self.targets_file = self.get_parameter("targets_file").value
        self.target_name = self.get_parameter("target_name").value
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        with Path(self.targets_file).open("r") as f:
            self.targets = yaml.safe_load(f)

        if self.target_name not in self.targets:
            raise RuntimeError(f"target not found: {self.target_name}")

        self.timer = self.create_timer(self.startup_delay_sec, self.send_goal)
        self.publish_status(f"waiting_to_go_{self.target_name}")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def send_goal(self):
        self.timer.cancel()
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.publish_status("error_no_nav_server")
            return

        target = self.targets[self.target_name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = target.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target["x"])
        goal.pose.pose.position.y = float(target["y"])
        qz, qw = yaw_to_quaternion(float(target["yaw"]))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.publish_status(f"navigating_to_{self.target_name}")
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_status("goal_rejected")
            return
        self.publish_status("goal_accepted")
        handle.get_result_async().add_done_callback(self.result)

    def result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f"arrived_{self.target_name}")
        else:
            self.publish_status(f"navigation_failed_{status}")

def main():
    rclpy.init()
    node = None
    try:
        node = NavigateToTargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
