#!/usr/bin/env python3

import math
from pathlib import Path

import yaml

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CalibrationMotionRunner(Node):
    def __init__(self):
        super().__init__("calibration_motion_runner")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("angular_speed", 0.20)
        self.declare_parameter("target_distance_m", 1.0)
        self.declare_parameter("target_rotation_deg", 360.0)
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("angle_tolerance_deg", 2.0)
        self.declare_parameter("record_scan_summary", True)
        self.declare_parameter("scan_sector_count", 8)
        self.declare_parameter("min_valid_scan_distance_m", 0.12)
        self.declare_parameter(
            "scan_summary_output_file",
            "/home/nvidia/857ChuanLi/rotation_scan_summary.yaml",
        )
        self.declare_parameter("mode", "straight")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("startup_delay_sec", 1.0)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.target_rotation_rad = math.radians(
            float(self.get_parameter("target_rotation_deg").value)
        )
        self.distance_tolerance_m = float(self.get_parameter("distance_tolerance_m").value)
        self.angle_tolerance_rad = math.radians(
            float(self.get_parameter("angle_tolerance_deg").value)
        )
        self.record_scan_summary = bool(self.get_parameter("record_scan_summary").value)
        self.scan_sector_count = max(4, int(self.get_parameter("scan_sector_count").value))
        self.min_valid_scan_distance_m = float(
            self.get_parameter("min_valid_scan_distance_m").value
        )
        self.scan_summary_output_file = str(
            self.get_parameter("scan_summary_output_file").value
        )
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 20)
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.have_odom = False
        self.have_scan = False
        self.active = False
        self.finished = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.accumulated_rotation = 0.0
        self.last_yaw = None
        self.scan_global_min = float("inf")
        self.scan_samples_used = 0
        self.scan_sector_mins = [float("inf")] * self.scan_sector_count

        if self.auto_start:
            self.start_timer = self.create_timer(self.startup_delay_sec, self._start_once)
        else:
            self.start_timer = None

        self.get_logger().info(
            f"Calibration motion ready: mode={self.mode}, "
            f"target_distance_m={self.target_distance_m:.3f}, "
            f"target_rotation_deg={math.degrees(self.target_rotation_rad):.1f}, "
            f"angular_speed={self.angular_speed:.3f}, "
            f"min_valid_scan_distance_m={self.min_valid_scan_distance_m:.3f}"
        )

    def _odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_x = float(pose.position.x)
        self.current_y = float(pose.position.y)
        self.current_yaw = yaw_from_quaternion(pose.orientation)
        self.have_odom = True

        if self.active:
            if self.last_yaw is None:
                self.last_yaw = self.current_yaw
            delta = normalize_angle(self.current_yaw - self.last_yaw)
            self.accumulated_rotation += delta
            self.last_yaw = self.current_yaw

    def _scan_cb(self, msg: LaserScan):
        self.have_scan = True
        if not self.active or not self.record_scan_summary:
            return

        bin_width = 2.0 * math.pi / self.scan_sector_count
        local_valid_sample = False

        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            min_valid_distance = max(float(msg.range_min), self.min_valid_scan_distance_m)
            if distance < min_valid_distance or distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            normalized = normalize_angle(angle)
            sector_index = int(((normalized + math.pi) % (2.0 * math.pi)) / bin_width)
            sector_index = min(max(sector_index, 0), self.scan_sector_count - 1)

            self.scan_global_min = min(self.scan_global_min, float(distance))
            self.scan_sector_mins[sector_index] = min(
                self.scan_sector_mins[sector_index], float(distance)
            )
            local_valid_sample = True

        if local_valid_sample:
            self.scan_samples_used += 1

    def _start_once(self):
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

        if not self.have_odom:
            self.get_logger().warning("Waiting for /odom before starting motion")
            self.start_timer = self.create_timer(0.5, self._start_once)
            return

        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw
        self.last_yaw = self.current_yaw
        self.accumulated_rotation = 0.0
        self.scan_global_min = float("inf")
        self.scan_samples_used = 0
        self.scan_sector_mins = [float("inf")] * self.scan_sector_count
        self.active = True

        self.get_logger().info(
            f"Starting mode={self.mode} from x={self.start_x:.3f}, y={self.start_y:.3f}, "
            f"yaw={self.start_yaw:.3f}"
        )
        if self.mode == "rotate" and self.record_scan_summary:
            self.get_logger().info(
                f"Recording scan summary from {self.scan_topic} into {self.scan_summary_output_file}"
            )

    def _publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def _stop_and_finish(self, result_text: str):
        self._publish_cmd(0.0, 0.0)
        self.active = False
        self.finished = True
        self.get_logger().info(result_text)
        if self.mode == "rotate" and self.record_scan_summary:
            self._write_scan_summary()
        self.get_logger().info("Calibration motion complete")
        self.create_timer(0.5, self._shutdown_once)

    def _sector_label(self, index: int) -> str:
        if self.scan_sector_count == 8:
            labels = [
                "back",
                "back_right",
                "right",
                "front_right",
                "front",
                "front_left",
                "left",
                "back_left",
            ]
            return labels[index]
        return f"sector_{index}"

    def _write_scan_summary(self):
        if not self.have_scan or self.scan_samples_used == 0:
            self.get_logger().warning("No valid scan samples were captured during rotation")
            return

        sector_summary = {}
        for index, value in enumerate(self.scan_sector_mins):
            sector_summary[self._sector_label(index)] = None if math.isinf(value) else round(value, 3)

        summary = {
            "mode": self.mode,
            "target_rotation_deg": round(math.degrees(self.target_rotation_rad), 3),
            "actual_rotation_deg": round(math.degrees(abs(self.accumulated_rotation)), 3),
            "angular_speed_command": round(self.angular_speed, 3),
            "scan_samples_used": self.scan_samples_used,
            "global_min_range_m": None if math.isinf(self.scan_global_min) else round(self.scan_global_min, 3),
            "sector_min_ranges_m": sector_summary,
            "notes": (
                "This is an observed clearance snapshot during in-place rotation. "
                "Use it as a tuning reference, not as a guaranteed universal safe distance."
            ),
        }

        output_path = Path(self.scan_summary_output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(summary, handle, sort_keys=False)

        self.get_logger().info(f"Saved rotation scan summary to {output_path}")
        self.get_logger().info(
            "Rotation clearance summary: "
            f"global_min_range_m={summary['global_min_range_m']}, "
            f"front={sector_summary.get('front')}, "
            f"left={sector_summary.get('left')}, "
            f"right={sector_summary.get('right')}, "
            f"back={sector_summary.get('back')}"
        )

    def _shutdown_once(self):
        if rclpy.ok():
            self.destroy_node()
            rclpy.shutdown()

    def _control_loop(self):
        if not self.active or self.finished:
            return

        if self.mode == "straight":
            distance = math.hypot(self.current_x - self.start_x, self.current_y - self.start_y)
            remaining = self.target_distance_m - distance
            if remaining <= self.distance_tolerance_m:
                self._stop_and_finish(
                    f"Straight test done: target={self.target_distance_m:.3f}m, actual={distance:.3f}m, "
                    f"dx={self.current_x - self.start_x:.3f}, dy={self.current_y - self.start_y:.3f}"
                )
                return
            self._publish_cmd(self.linear_speed, 0.0)
            return

        if self.mode == "rotate":
            rotation = abs(self.accumulated_rotation)
            remaining = abs(self.target_rotation_rad) - rotation
            if remaining <= self.angle_tolerance_rad:
                self._stop_and_finish(
                    f"Rotation test done: target={math.degrees(self.target_rotation_rad):.1f}deg, "
                    f"actual={math.degrees(rotation):.1f}deg"
                )
                return
            angular = self.angular_speed if self.target_rotation_rad >= 0.0 else -self.angular_speed
            self._publish_cmd(0.0, angular)
            return

        self.get_logger().error(f"Unsupported mode '{self.mode}', expected 'straight' or 'rotate'")
        self._stop_and_finish("Stopped due to invalid mode")


def main():
    rclpy.init()
    node = CalibrationMotionRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
