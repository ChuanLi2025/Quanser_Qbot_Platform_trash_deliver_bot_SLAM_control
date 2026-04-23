#!/usr/bin/env python3

import math
from pathlib import Path
from threading import Lock

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node


class TrashMissionTestNode(Node):
    def __init__(self):
        super().__init__("trash_mission_test_node")

        default_targets = f"{get_package_share_directory('qbot_platform')}/config/trash_targets.yaml"
        self.declare_parameter("targets_file", default_targets)
        self.declare_parameter("command_topic", "/trash_target")
        self.declare_parameter("status_topic", "/trash_mission_status")
        self.declare_parameter("dump_command_topic", "/dump_trash_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("task_wait_seconds", 20.0)
        self.declare_parameter("home_name", "home")
        self.declare_parameter("forward_speed", 0.12)
        self.declare_parameter("backup_speed", -0.08)
        self.declare_parameter("turn_speed", 0.45)
        self.declare_parameter("obstacle_distance", 0.75)
        self.declare_parameter("travel_time_blue", 18.0)
        self.declare_parameter("travel_time_black", 20.0)
        self.declare_parameter("travel_time_home", 16.0)
        self.declare_parameter("backup_time", 1.2)
        self.declare_parameter("turn_time", 1.5)
        self.declare_parameter("front_sector_degrees", 180.0)
        self.declare_parameter("front_sector_count", 5)

        self.targets_file = self.get_parameter("targets_file").value
        self.command_topic = self.get_parameter("command_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.dump_command_topic = self.get_parameter("dump_command_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.task_wait_seconds = float(self.get_parameter("task_wait_seconds").value)
        self.home_name = self.get_parameter("home_name").value
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.backup_speed = float(self.get_parameter("backup_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.obstacle_distance = float(self.get_parameter("obstacle_distance").value)
        self.backup_time = float(self.get_parameter("backup_time").value)
        self.turn_time = float(self.get_parameter("turn_time").value)
        self.front_sector_degrees = float(self.get_parameter("front_sector_degrees").value)
        self.front_sector_count = int(self.get_parameter("front_sector_count").value)

        self.targets = self._load_targets(self.targets_file)
        self.travel_times = {
            "blue": float(self.get_parameter("travel_time_blue").value),
            "black": float(self.get_parameter("travel_time_black").value),
            self.home_name: float(self.get_parameter("travel_time_home").value),
        }

        self.command_sub = self.create_subscription(String, self.command_topic, self._command_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.dump_pub = self.create_publisher(String, self.dump_command_topic, 10)

        self._lock = Lock()
        self._state = "idle"
        self._active_target = None
        self._pending_return = False
        self._travel_deadline = None
        self._phase_deadline = None
        self._current_motion = Twist()
        self._turn_direction = 1.0
        self._control_timer = self.create_timer(0.1, self._control_loop)

        self._publish_status("idle")
        self.get_logger().info("Trash mission test node ready")

    def _load_targets(self, filename):
        path = Path(filename)
        if not path.exists():
            raise FileNotFoundError(f"Targets file not found: {filename}")
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        required = {self.home_name, "blue", "black"}
        missing = [name for name in required if name not in data]
        if missing:
            raise RuntimeError(f"Missing targets in {filename}: {', '.join(missing)}")
        return data

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"status={text}")

    def _publish_dump_command(self, target_name: str):
        msg = String()
        msg.data = f"start_{target_name}"
        self.dump_pub.publish(msg)
        self.get_logger().info(f"dump_command={msg.data}")

    def _command_cb(self, msg: String):
        command = msg.data.strip().lower()
        if command not in self.targets:
            self.get_logger().warning(f"Ignoring unknown target '{command}'")
            return

        with self._lock:
            if self._state != "idle":
                self.get_logger().warning(f"Busy in state '{self._state}', ignoring '{command}'")
                return
            self._active_target = command
            self._pending_return = command != self.home_name
            self._state = "navigating"
            self._travel_deadline = self.get_clock().now().nanoseconds / 1e9 + self.travel_times[command]

        self._publish_status(f"sim_navigating_to_{command}")

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            if self._state != "navigating":
                return

        sector_ranges = self._front_sector_ranges(msg)
        if not sector_ranges:
            return

        sector_mins = [min(ranges) if ranges else float("inf") for ranges in sector_ranges]
        if min(sector_mins) >= self.obstacle_distance:
            return

        left_clearance = min(sector_mins[0:2])
        right_clearance = min(sector_mins[-2:])
        turn_direction = 1.0 if left_clearance >= right_clearance else -1.0

        now_sec = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            if self._state != "navigating":
                return
            self._state = "avoiding_backup"
            self._phase_deadline = now_sec + self.backup_time
            self._turn_direction = turn_direction

        self._publish_status("sim_avoiding_backup")

    def _front_sector_ranges(self, scan: LaserScan):
        total_angle = math.radians(self.front_sector_degrees)
        half_angle = total_angle / 2.0
        sector_count = max(1, self.front_sector_count)
        sector_width = total_angle / sector_count
        sectors = [[] for _ in range(sector_count)]

        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            wrapped = math.atan2(math.sin(angle), math.cos(angle))
            if wrapped < -half_angle or wrapped > half_angle:
                continue

            normalized = wrapped + half_angle
            sector_index = min(int(normalized / sector_width), sector_count - 1)
            sectors[sector_index].append(value)

        return sectors

    def _control_loop(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        twist = Twist()
        next_status = None
        dump_target = None

        with self._lock:
            if self._state == "idle":
                self._current_motion = twist
            elif self._state == "navigating":
                twist.linear.x = self.forward_speed
                if now_sec >= self._travel_deadline:
                    reached_target = self._active_target
                    if self._pending_return:
                        self._state = "working"
                        self._phase_deadline = now_sec + self.task_wait_seconds
                        dump_target = reached_target
                        next_status = f"sim_working_at_{reached_target}"
                    else:
                        self._state = "idle"
                        self._active_target = None
                        self._pending_return = False
                        next_status = "idle"
            elif self._state == "avoiding_backup":
                twist.linear.x = self.backup_speed
                if now_sec >= self._phase_deadline:
                    self._state = "avoiding_turn"
                    self._phase_deadline = now_sec + self.turn_time
                    next_status = "sim_avoiding_turn"
            elif self._state == "avoiding_turn":
                twist.angular.z = self.turn_speed * self._turn_direction
                if now_sec >= self._phase_deadline:
                    self._state = "navigating"
                    next_status = f"sim_navigating_to_{self._active_target}"
            elif self._state == "working":
                if now_sec >= self._phase_deadline:
                    self._active_target = self.home_name
                    self._pending_return = False
                    self._state = "navigating"
                    self._travel_deadline = now_sec + self.travel_times[self.home_name]
                    next_status = "sim_returning_home"

            self._current_motion = twist

        self.cmd_pub.publish(twist)

        if dump_target is not None:
            self._publish_dump_command(dump_target)
        if next_status is not None:
            self._publish_status(next_status)


def main():
    rclpy.init()
    node = None
    try:
        node = TrashMissionTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
