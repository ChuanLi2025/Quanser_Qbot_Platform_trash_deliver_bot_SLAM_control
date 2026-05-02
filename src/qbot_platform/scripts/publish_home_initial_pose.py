#!/usr/bin/env python3
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

class PublishHomeInitialPose(Node):
    def __init__(self):
        super().__init__("publish_home_initial_pose")
        self.declare_parameter("targets_file", "/home/nvidia/857ChuanLi/src/qbot_platform/config/trash_targets_v5.yaml")
        self.declare_parameter("home_name", "home")
        self.declare_parameter("publish_count", 20)
        self.declare_parameter("publish_period_sec", 0.5)

        self.targets_file = self.get_parameter("targets_file").value
        self.home_name = self.get_parameter("home_name").value
        self.remaining = int(self.get_parameter("publish_count").value)
        period = float(self.get_parameter("publish_period_sec").value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.msg = self.load_pose()
        self.timer = self.create_timer(period, self.publish_once)

    def load_pose(self):
        with Path(self.targets_file).open("r") as f:
            data = yaml.safe_load(f)

        home = data[self.home_name]
        yaw = float(home["yaw"])
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = home.get("frame_id", "map")
        msg.pose.pose.position.x = float(home["x"])
        msg.pose.pose.position.y = float(home["y"])
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg

    def publish_once(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.remaining -= 1
        self.get_logger().info(f"published initial pose, remaining={self.remaining}")
        if self.remaining <= 0:
            self.timer.cancel()

def main():
    rclpy.init()
    node = PublishHomeInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
