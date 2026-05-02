#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )

def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)

class RoundtripToTargetNode(Node):
    def __init__(self):
        super().__init__("roundtrip_to_target_node")

        self.declare_parameter("targets_file", "/home/nvidia/857ChuanLi/src/qbot_platform/config/trash_targets_v5.yaml")
        self.declare_parameter("home_name", "home")
        self.declare_parameter("target_name", "blue")
        self.declare_parameter("startup_delay_sec", 22.0)
        self.declare_parameter("wait_at_target_sec", 10.0)
        self.declare_parameter("turn_degrees", 180.0)
        self.declare_parameter("turn_angular_speed", 0.85)
        self.declare_parameter("turn_tolerance_deg", 3.0)
        self.declare_parameter("status_topic", "/trash_mission_status")

        self.targets_file = self.get_parameter("targets_file").value
        self.home_name = self.get_parameter("home_name").value
        self.target_name = self.get_parameter("target_name").value
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.wait_at_target_sec = float(self.get_parameter("wait_at_target_sec").value)
        self.turn_radians = math.radians(float(self.get_parameter("turn_degrees").value))
        self.turn_speed = float(self.get_parameter("turn_angular_speed").value)
        self.turn_tolerance = math.radians(float(self.get_parameter("turn_tolerance_deg").value))

        self.status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        with Path(self.targets_file).open("r") as f:
            self.targets = yaml.safe_load(f)

        self.current_yaw = None
        self.turn_start_yaw = None
        self.phase = "startup"
        self.active_goal = None
        self.timer = self.create_timer(self.startup_delay_sec, self.go_to_target)
        self.turn_timer = None

        self.publish_status(f"waiting_to_start_{self.target_name}_180_home")

    def odom_cb(self, msg):
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def cancel_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def build_goal(self, target):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = target.get("frame_id", "map")
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target["x"])
        goal.pose.pose.position.y = float(target["y"])
        qz, qw = yaw_to_quaternion(float(target["yaw"]))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        return goal

    def send_goal(self, name, target):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.publish_status("error_no_nav_server")
            return

        self.active_goal = name
        self.publish_status(f"navigating_to_{name}")
        future = self.nav_client.send_goal_async(self.build_goal(target))
        future.add_done_callback(self.goal_response)

    def go_to_target(self):
        self.cancel_timer()
        self.phase = "go_to_target"
        self.send_goal(self.target_name, self.targets[self.target_name])

    def start_turn_180(self):
        self.cancel_timer()
        if self.current_yaw is None:
            self.publish_status("waiting_for_odom_before_turn")
            self.timer = self.create_timer(0.5, self.start_turn_180)
            return

        self.phase = "turn_180"
        self.turn_start_yaw = self.current_yaw
        self.publish_status(f"turning_180_at_{self.target_name}")
        self.turn_timer = self.create_timer(0.05, self.turn_step)

    def turn_step(self):
        if self.current_yaw is None:
            return

        turned = abs(normalize_angle(self.current_yaw - self.turn_start_yaw))
        if turned >= max(0.0, self.turn_radians - self.turn_tolerance):
            self.stop_robot()
            self.turn_timer.cancel()
            self.turn_timer = None
            self.publish_status(f"turn_180_complete_at_{self.target_name}")
            self.timer = self.create_timer(0.5, self.return_home)
            return

        cmd = Twist()
        cmd.angular.z = self.turn_speed
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        for _ in range(5):
            self.cmd_pub.publish(cmd)

    def return_home(self):
        self.cancel_timer()
        self.phase = "return_home"
        self.send_goal(self.home_name, self.targets[self.home_name])

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.publish_status(f"goal_rejected_{self.active_goal}")
            return

        self.publish_status(f"goal_accepted_{self.active_goal}")
        handle.get_result_async().add_done_callback(self.result)

    def result(self, future):
        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f"navigation_failed_{self.active_goal}_{status}")
            return

        if self.phase == "go_to_target":
            self.publish_status(f"arrived_{self.target_name}")
            self.publish_status(f"waiting_{self.wait_at_target_sec:.1f}_sec_at_{self.target_name}")
            self.timer = self.create_timer(self.wait_at_target_sec, self.start_turn_180)
            return

        if self.phase == "return_home":
            self.stop_robot()
            self.publish_status(f"arrived_{self.home_name}")
            self.publish_status("roundtrip_complete")

def main():
    rclpy.init()
    node = RoundtripToTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
