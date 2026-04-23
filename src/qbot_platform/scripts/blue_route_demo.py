#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class BlueRouteDemo(Node):
    def __init__(self):
        super().__init__("blue_route_demo")

        self.declare_parameter("command_topic", "/trash_target")
        self.declare_parameter("status_topic", "/trash_mission_status")
        self.declare_parameter("dump_topic", "/dump_trash_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/qbot_imu")

        self.declare_parameter("forward_1_sec", 26.0)
        self.declare_parameter("forward_2_sec", 18.0)
        self.declare_parameter("wait_at_blue_sec", 5.0)
        self.declare_parameter("return_1_sec", 18.0)
        self.declare_parameter("return_2_sec", 26.0)

        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("turn_left_angle_deg", 90.0)
        self.declare_parameter("turn_back_angle_deg", 180.0)
        self.declare_parameter("turn_right_home_angle_deg", 90.0)
        self.declare_parameter("turn_angle_tolerance_deg", 4.0)
        self.declare_parameter("turn_kp", 1.2)
        self.declare_parameter("turn_min_speed", 0.12)
        self.declare_parameter("obstacle_distance", 0.9)
        self.declare_parameter("front_arc_degrees", 60.0)
        self.declare_parameter("front_sector_count", 5)
        self.declare_parameter("avoid_backup_sec", 0.8)
        self.declare_parameter("avoid_turn_sec", 1.6)
        self.declare_parameter("avoid_forward_sec", 1.4)
        self.declare_parameter("avoid_realign_sec", 1.2)

        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.dump_topic = self.get_parameter("dump_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value

        self.forward_1_sec = float(self.get_parameter("forward_1_sec").value)
        self.forward_2_sec = float(self.get_parameter("forward_2_sec").value)
        self.wait_at_blue_sec = float(self.get_parameter("wait_at_blue_sec").value)
        self.return_1_sec = float(self.get_parameter("return_1_sec").value)
        self.return_2_sec = float(self.get_parameter("return_2_sec").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.turn_left_angle = math.radians(float(self.get_parameter("turn_left_angle_deg").value))
        self.turn_back_angle = math.radians(float(self.get_parameter("turn_back_angle_deg").value))
        self.turn_right_home_angle = math.radians(float(self.get_parameter("turn_right_home_angle_deg").value))
        self.turn_angle_tolerance = math.radians(
            float(self.get_parameter("turn_angle_tolerance_deg").value)
        )
        self.turn_kp = float(self.get_parameter("turn_kp").value)
        self.turn_min_speed = float(self.get_parameter("turn_min_speed").value)
        self.obstacle_distance = float(self.get_parameter("obstacle_distance").value)
        self.front_arc_degrees = float(self.get_parameter("front_arc_degrees").value)
        self.front_sector_count = int(self.get_parameter("front_sector_count").value)
        self.avoid_backup_sec = float(self.get_parameter("avoid_backup_sec").value)
        self.avoid_turn_sec = float(self.get_parameter("avoid_turn_sec").value)
        self.avoid_forward_sec = float(self.get_parameter("avoid_forward_sec").value)
        self.avoid_realign_sec = float(self.get_parameter("avoid_realign_sec").value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.dump_pub = self.create_publisher(String, self.dump_topic, 10)
        self.command_sub = self.create_subscription(String, self.command_topic, self._command_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self._imu_cb, 20)

        self.timer = self.create_timer(0.05, self._control_loop)

        self.state = "idle"
        self.deadline = None
        self.latest_scan = None
        self.resume_state = None
        self.resume_duration = None
        self.avoid_turn_sign = 1.0
        self.have_imu = False
        self.integrated_yaw = 0.0
        self.last_imu_time_sec = None
        self.turn_start_yaw = 0.0
        self.turn_target_delta = 0.0
        self._publish_status("idle")

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def _publish_dump(self):
        msg = String()
        msg.data = "start_blue"
        self.dump_pub.publish(msg)
        self.get_logger().info("dump_command=start_blue")

    def _set_state(self, state: str, duration_sec: Optional[float], status: str):
        self.state = state
        if duration_sec is None:
            self.deadline = None
        else:
            self.deadline = self.get_clock().now().nanoseconds / 1e9 + duration_sec
        self._publish_status(status)

    def _command_cb(self, msg: String):
        command = msg.data.strip().lower()
        if command != "blue":
            return
        if self.state != "idle":
            self.get_logger().warning(f"Busy in state {self.state}, ignoring blue command")
            return

        self._set_state("forward_1", self.forward_1_sec, "route_forward_1")

    def _send_cmd(self, linear: float = 0.0, angular: float = 0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def _expired(self) -> bool:
        return self.deadline is not None and (self.get_clock().now().nanoseconds / 1e9) >= self.deadline

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _imu_cb(self, msg: Imu):
        now_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        if self.last_imu_time_sec is not None:
            dt = now_sec - self.last_imu_time_sec
            if 0.0 < dt < 0.2:
                # The driver publishes angular velocity in rad/s; integrate it
                # to get a relative turn angle for closed-loop turns.
                self.integrated_yaw += msg.angular_velocity.z * dt
        self.last_imu_time_sec = now_sec
        self.have_imu = True

    def _front_sector_mins(self):
        if self.latest_scan is None:
            return None

        scan = self.latest_scan
        sector_count = max(1, self.front_sector_count)
        sectors = [[] for _ in range(sector_count)]
        half_arc = math.radians(self.front_arc_degrees) / 2.0
        sector_width = (2.0 * half_arc) / sector_count

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            if angle < -half_arc or angle > half_arc:
                continue

            shifted = angle + half_arc
            index = min(int(shifted / sector_width), sector_count - 1)
            sectors[index].append(distance)

        return [min(values) if values else float("inf") for values in sectors]

    def _front_blocked(self) -> bool:
        sector_mins = self._front_sector_mins()
        if sector_mins is None:
            return False
        return min(sector_mins) < self.obstacle_distance

    def _choose_turn_sign(self) -> float:
        sector_mins = self._front_sector_mins()
        if sector_mins is None:
            return 1.0
        left_clearance = min(sector_mins[: max(1, len(sector_mins) // 2)])
        right_clearance = min(sector_mins[max(1, len(sector_mins) // 2):])
        # Positive angular z is the successful counter-clockwise turn for this robot.
        return 1.0 if left_clearance >= right_clearance else -1.0

    def _enter_avoidance(self, resume_state: str):
        if self.deadline is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.resume_state = resume_state
        self.resume_duration = max(0.0, self.deadline - now_sec)
        self.avoid_turn_sign = self._choose_turn_sign()
        self._set_state("avoid_backup", self.avoid_backup_sec, f"avoid_backup_{resume_state}")

    def _begin_turn(self, state: str, target_delta: float, status: str):
        if not self.have_imu or self.last_imu_time_sec is None:
            self.get_logger().warning("No IMU data yet; cannot start closed-loop turn")
            self.state = "idle"
            self.deadline = None
            self._publish_status("idle")
            return
        self.state = state
        self.deadline = None
        self.turn_start_yaw = self.integrated_yaw
        self.turn_target_delta = target_delta
        self._publish_status(status)

    def _turn_error(self) -> float:
        return math.atan2(
            math.sin(self.turn_target_delta - self._turned_angle()),
            math.cos(self.turn_target_delta - self._turned_angle()),
        )

    def _turned_angle(self) -> float:
        return math.atan2(
            math.sin(self.integrated_yaw - self.turn_start_yaw),
            math.cos(self.integrated_yaw - self.turn_start_yaw),
        )

    def _execute_turn(self, next_state: str, next_duration: Optional[float], next_status: str):
        if not self.have_imu:
            self._send_cmd(0.0, 0.0)
            return

        error = self._turn_error()
        if abs(error) <= self.turn_angle_tolerance:
            self._send_cmd(0.0, 0.0)
            self._set_state(next_state, next_duration, next_status)
            return

        angular_mag = min(self.angular_speed, max(self.turn_min_speed, self.turn_kp * abs(error)))
        angular = angular_mag if error > 0.0 else -angular_mag
        self._send_cmd(0.0, angular)

    def _control_loop(self):
        if self.state == "idle":
            return

        if self.state == "forward_1":
            if self._front_blocked():
                self._send_cmd(0.0, 0.0)
                self._enter_avoidance("forward_1")
                return
            if self._expired():
                self._send_cmd(0.0, 0.0)
                self._begin_turn("turn_left_1", -self.turn_left_angle, "route_turn_left_1")
                return
            self._send_cmd(self.linear_speed, 0.0)
            return

        if self.state == "turn_left_1":
            self._execute_turn("forward_2", self.forward_2_sec, "route_forward_2")
            return

        if self.state == "forward_2":
            if self._front_blocked():
                self._send_cmd(0.0, 0.0)
                self._enter_avoidance("forward_2")
                return
            if self._expired():
                self._send_cmd(0.0, 0.0)
                self._publish_dump()
                self._set_state("wait_blue", self.wait_at_blue_sec, "route_wait_blue")
                return
            self._send_cmd(self.linear_speed, 0.0)
            return

        if self.state == "wait_blue":
            self._send_cmd(0.0, 0.0)
            if self._expired():
                self._begin_turn("turn_back", self.turn_back_angle, "route_turn_back")
            return

        if self.state == "turn_back":
            self._execute_turn("return_1", self.return_1_sec, "route_return_1")
            return

        if self.state == "return_1":
            if self._front_blocked():
                self._send_cmd(0.0, 0.0)
                self._enter_avoidance("return_1")
                return
            if self._expired():
                self._send_cmd(0.0, 0.0)
                self._begin_turn(
                    "turn_right_home", self.turn_right_home_angle, "route_turn_right_home"
                )
                return
            self._send_cmd(self.linear_speed, 0.0)
            return

        if self.state == "turn_right_home":
            self._execute_turn("return_2", self.return_2_sec, "route_return_2")
            return

        if self.state == "return_2":
            if self._front_blocked():
                self._send_cmd(0.0, 0.0)
                self._enter_avoidance("return_2")
                return
            if self._expired():
                self._send_cmd(0.0, 0.0)
                self.state = "idle"
                self.deadline = None
                self._publish_status("route_complete")
                return
            self._send_cmd(self.linear_speed, 0.0)
            return

        if self.state == "avoid_backup":
            if self._expired():
                self._set_state("avoid_turn_out", self.avoid_turn_sec, "avoid_turn_out")
                return
            self._send_cmd(-0.10, 0.0)
            return

        if self.state == "avoid_turn_out":
            if self._expired():
                self._set_state("avoid_forward", self.avoid_forward_sec, "avoid_forward")
                return
            self._send_cmd(0.0, self.avoid_turn_sign * self.angular_speed)
            return

        if self.state == "avoid_forward":
            if self._expired():
                self._set_state("avoid_realign", self.avoid_realign_sec, "avoid_realign")
                return
            self._send_cmd(self.linear_speed, 0.0)
            return

        if self.state == "avoid_realign":
            if self._expired():
                if self.resume_state is None:
                    self.state = "idle"
                    self.deadline = None
                    self._publish_status("idle")
                    return
                self._set_state(
                    self.resume_state,
                    self.resume_duration,
                    f"resume_{self.resume_state}",
                )
                self.resume_state = None
                self.resume_duration = None
                return
            self._send_cmd(0.0, -self.avoid_turn_sign * self.angular_speed)
            return


def main():
    rclpy.init()
    node = BlueRouteDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
