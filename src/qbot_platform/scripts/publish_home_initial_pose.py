#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class PublishHomeInitialPoseNode(Node):
    def __init__(self):
        super().__init__("publish_home_initial_pose")

        default_targets = f"{get_package_share_directory('qbot_platform')}/config/trash_targets.yaml"
        self.declare_parameter("targets_file", default_targets)
        self.declare_parameter("home_name", "home")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("publish_count", 5)
        self.declare_parameter("publish_period_sec", 0.5)

        self.targets_file = self.get_parameter("targets_file").value
        self.home_name = self.get_parameter("home_name").value
        self.initial_pose_topic = self.get_parameter("initial_pose_topic").value
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)
        self._pose_msg = self._load_home_pose()
        self._remaining = self.publish_count
        self._timer = self.create_timer(self.publish_period_sec, self._publish_once)

        self.get_logger().info(
            f"Publishing home initial pose to {self.initial_pose_topic} {self.publish_count} times"
        )

    def _load_home_pose(self):
        path = Path(self.targets_file)
        if not path.exists():
            raise FileNotFoundError(f"Targets file not found: {self.targets_file}")

        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        if self.home_name not in data:
            raise RuntimeError(f"Target '{self.home_name}' not found in {self.targets_file}")

        home = data[self.home_name]
        yaw = float(home["yaw"])
        half = yaw * 0.5

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = home.get("frame_id", "map")
        msg.pose.pose.position.x = float(home["x"])
        msg.pose.pose.position.y = float(home["y"])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        # Conservative planar covariance for manual initialization near home.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        return msg

    def _publish_once(self):
        self._pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self._pose_msg)
        self._remaining -= 1
        self.get_logger().info(
            f"Published initial pose for '{self.home_name}', remaining={self._remaining}"
        )

        if self._remaining <= 0:
            self._timer.cancel()
            self.get_logger().info("Initial pose publishing complete")
            self.destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = PublishHomeInitialPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
