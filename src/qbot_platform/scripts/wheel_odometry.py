#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    half = yaw * 0.5
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


class WheelOdometry(Node):
    def __init__(self):
        super().__init__("wheel_odometry")

        # Matches the physical parameters used in qbot_platform_driver_interface.cpp.
        self.declare_parameter("joint_topic", "/qbot_joint")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("left_index", 0)
        self.declare_parameter("right_index", 1)
        self.declare_parameter("wheel_radius", 3.5 * 0.0254 / 2.0)
        self.declare_parameter("wheel_separation", 0.3928)
        self.declare_parameter(
            "pose_covariance_diagonal",
            [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
        )
        self.declare_parameter(
            "twist_covariance_diagonal",
            [0.001, 0.001, 0.001, 0.001, 0.001, 0.01],
        )

        self.joint_topic = self.get_parameter("joint_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.left_index = int(self.get_parameter("left_index").value)
        self.right_index = int(self.get_parameter("right_index").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.pose_covariance = list(self.get_parameter("pose_covariance_diagonal").value)
        self.twist_covariance = list(self.get_parameter("twist_covariance_diagonal").value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_sub = self.create_subscription(JointState, self.joint_topic, self._joint_cb, 50)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_left = None
        self.last_right = None
        self.last_stamp = None

    def _joint_cb(self, msg: JointState):
        if len(msg.position) <= max(self.left_index, self.right_index):
            return

        left_pos = float(msg.position[self.left_index])
        right_pos = float(msg.position[self.right_index])

        if self.last_left is None or self.last_right is None:
            self.last_left = left_pos
            self.last_right = right_pos
            self.last_stamp = msg.header.stamp
            return

        d_left = (left_pos - self.last_left) * self.wheel_radius
        d_right = (right_pos - self.last_right) * self.wheel_radius
        d_center = 0.5 * (d_left + d_right)
        d_theta = (d_right - d_left) / self.wheel_separation

        self.x += d_center * math.cos(self.yaw + 0.5 * d_theta)
        self.y += d_center * math.sin(self.yaw + 0.5 * d_theta)
        self.yaw = math.atan2(math.sin(self.yaw + d_theta), math.cos(self.yaw + d_theta))

        linear_x = 0.0
        angular_z = 0.0
        if self.last_stamp is not None:
            dt = (
                (msg.header.stamp.sec - self.last_stamp.sec)
                + (msg.header.stamp.nanosec - self.last_stamp.nanosec) / 1e9
            )
            if dt > 0.0:
                linear_x = d_center / dt
                angular_z = d_theta / dt

        quat = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.angular.z = angular_z
        odom.pose.covariance[0] = self.pose_covariance[0]
        odom.pose.covariance[7] = self.pose_covariance[1]
        odom.pose.covariance[14] = self.pose_covariance[2]
        odom.pose.covariance[21] = self.pose_covariance[3]
        odom.pose.covariance[28] = self.pose_covariance[4]
        odom.pose.covariance[35] = self.pose_covariance[5]
        odom.twist.covariance[0] = self.twist_covariance[0]
        odom.twist.covariance[7] = self.twist_covariance[1]
        odom.twist.covariance[14] = self.twist_covariance[2]
        odom.twist.covariance[21] = self.twist_covariance[3]
        odom.twist.covariance[28] = self.twist_covariance[4]
        odom.twist.covariance[35] = self.twist_covariance[5]
        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.odom_frame_id
        tf_msg.child_frame_id = self.base_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation = quat
        self.tf_broadcaster.sendTransform(tf_msg)

        self.last_left = left_pos
        self.last_right = right_pos
        self.last_stamp = msg.header.stamp


def main():
    rclpy.init()
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
