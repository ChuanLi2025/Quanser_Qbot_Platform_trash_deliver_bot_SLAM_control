#!/usr/bin/env python3

import math
from pathlib import Path
from threading import Lock

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class TrashMissionNode(Node):
    def __init__(self):
        super().__init__("trash_mission_node")

        default_targets = f"{get_package_share_directory('qbot_platform')}/config/trash_targets.yaml"
        self.declare_parameter("targets_file", default_targets)
        self.declare_parameter("command_topic", "/trash_target")
        self.declare_parameter("status_topic", "/trash_mission_status")
        self.declare_parameter("dump_command_topic", "/dump_trash_cmd")
        self.declare_parameter("home_name", "home")
        self.declare_parameter("task_wait_seconds", 20.0)
        self.declare_parameter("auto_go_home_on_startup", True)
        self.declare_parameter("startup_delay_sec", 8.0)
        self.declare_parameter("startup_mission_target", "")
        self.declare_parameter("startup_mission_delay_sec", 1.0)

        self.targets_file = self.get_parameter("targets_file").get_parameter_value().string_value
        self.command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.dump_command_topic = self.get_parameter("dump_command_topic").get_parameter_value().string_value
        self.home_name = self.get_parameter("home_name").get_parameter_value().string_value
        self.task_wait_seconds = (
            self.get_parameter("task_wait_seconds").get_parameter_value().double_value
        )
        self.auto_go_home_on_startup = (
            self.get_parameter("auto_go_home_on_startup").get_parameter_value().bool_value
        )
        self.startup_delay_sec = (
            self.get_parameter("startup_delay_sec").get_parameter_value().double_value
        )
        self.startup_mission_target = (
            self.get_parameter("startup_mission_target").get_parameter_value().string_value.strip().lower()
        )
        self.startup_mission_delay_sec = (
            self.get_parameter("startup_mission_delay_sec").get_parameter_value().double_value
        )

        self.targets = self._load_targets(self.targets_file)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.command_sub = self.create_subscription(String, self.command_topic, self._command_cb, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.dump_pub = self.create_publisher(String, self.dump_command_topic, 10)

        self._lock = Lock()
        self._busy = False
        self._pending_return = False
        self._active_target = None
        self._task_timer = None
        self._startup_timer = None
        self._startup_mission_timer = None
        self._startup_go_home_in_progress = False
        self._startup_mission_triggered = False

        self._publish_status("idle")
        self.get_logger().info(f"Trash mission node ready. Commands on {self.command_topic}")

        if self.auto_go_home_on_startup:
            self._startup_timer = self.create_timer(self.startup_delay_sec, self._startup_go_home)

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

    def _command_cb(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return

        if command not in self.targets:
            self.get_logger().warning(f"Ignoring unknown target '{command}'")
            return

        with self._lock:
            if self._busy:
                self.get_logger().warning(
                    f"Busy with target '{self._active_target}'. Ignoring new command '{command}'"
                )
                return
            self._busy = True
            self._pending_return = command != self.home_name
            self._active_target = command

        self._publish_status(f"navigating_to_{command}")
        self._send_goal(command)

    def _startup_go_home(self):
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

        with self._lock:
            if self._busy:
                return
            self._busy = True
            self._pending_return = False
            self._active_target = self.home_name
            self._startup_go_home_in_progress = True

        self._publish_status("startup_navigating_to_home")
        self._send_goal(self.home_name)

    def _startup_send_target(self):
        if self._startup_mission_timer is not None:
            self._startup_mission_timer.cancel()
            self._startup_mission_timer = None

        target = self.startup_mission_target
        if not target or target not in self.targets:
            return

        with self._lock:
            if self._busy:
                return
            self._busy = True
            self._pending_return = target != self.home_name
            self._active_target = target
            self._startup_mission_triggered = True

        self._publish_status(f"startup_navigating_to_{target}")
        self._send_goal(target)

    def _send_goal(self, target_name: str):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server is not available")
            with self._lock:
                self._busy = False
                self._pending_return = False
                self._active_target = None
            self._publish_status("error_no_nav_server")
            return

        target = self.targets[target_name]
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
            self.get_logger().error(f"Goal to '{self._active_target}' was rejected")
            with self._lock:
                self._busy = False
                self._pending_return = False
                self._active_target = None
            self._publish_status("goal_rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f"Navigation to '{self._active_target}' ended with status {status}")
            with self._lock:
                self._busy = False
                self._pending_return = False
                self._active_target = None
            self._publish_status(f"navigation_failed_{status}")
            return

        target_reached = self._active_target
        startup_go_home_finished = self._startup_go_home_in_progress and target_reached == self.home_name
        if startup_go_home_finished:
            self._startup_go_home_in_progress = False
        self._publish_status(f"arrived_{target_reached}")

        if self._pending_return:
            self._publish_dump_command(target_reached)
            self._publish_status(f"working_at_{target_reached}")
            self._task_timer = self.create_timer(self.task_wait_seconds, self._return_home_once)
        else:
            with self._lock:
                self._busy = False
                self._pending_return = False
                self._active_target = None
            self._publish_status("idle")
            if (
                startup_go_home_finished
                and self.startup_mission_target
                and not self._startup_mission_triggered
                and self.startup_mission_target in self.targets
                and self.startup_mission_target != self.home_name
            ):
                self._startup_mission_timer = self.create_timer(
                    self.startup_mission_delay_sec, self._startup_send_target
                )

    def _publish_dump_command(self, target_name: str):
        msg = String()
        msg.data = f"start_{target_name}"
        self.dump_pub.publish(msg)
        self.get_logger().info(f"dump_command={msg.data}")

    def _return_home_once(self):
        if self._task_timer is not None:
            self._task_timer.cancel()
            self._task_timer = None

        with self._lock:
            self._pending_return = False
            self._active_target = self.home_name

        self._publish_status("returning_home")
        self._send_goal(self.home_name)


def main():
    rclpy.init()
    node = None
    try:
        node = TrashMissionNode()
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
