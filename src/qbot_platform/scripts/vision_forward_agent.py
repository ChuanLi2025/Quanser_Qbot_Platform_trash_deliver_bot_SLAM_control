#!/usr/bin/env python3
"""Vision-gated forward motion agent for QBot Platform."""

import base64
import hashlib
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


OPENROUTER_CHAT_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_MODEL = "nvidia/nemotron-3-nano-omni-30b-a3b-reasoning:free"


class VisionForwardAgent(Node):
    def __init__(self):
        super().__init__("vision_forward_agent")

        self._declare_parameters()
        self._load_parameters()

        self.api_key = os.environ.get(self.api_key_env, "")
        if not self.api_key:
            self.get_logger().warning(
                f"{self.api_key_env} is not set; OpenRouter requests will fail safely."
            )

        self.latest_image = None
        self.latest_image_count = 0
        self.latest_image_time = 0.0
        self.latest_odom = None
        self.latest_odom_time = 0.0
        self.lock = threading.Lock()
        self.request_thread = None

        self.attempt_index = 0
        self.state = "waiting"
        self.next_attempt_time = time.monotonic()
        self.start_x = 0.0
        self.start_y = 0.0
        self.move_started_at = 0.0
        self.last_dependency_log = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self._image_cb,
            qos,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_cb,
            20,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(0.05, self._control_loop)

        self._publish_status(
            "started",
            attempts=self.attempts,
            dry_run=self.dry_run,
            image_topic=self.image_topic,
            odom_topic=self.odom_topic,
            cmd_vel_topic=self.cmd_vel_topic,
            model=self.model,
        )

    def _declare_parameters(self):
        self.declare_parameter("image_topic", "/camera/color_image/compressed")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/vision_forward_agent/status")
        self.declare_parameter("model", DEFAULT_MODEL)
        self.declare_parameter("attempts", 3)
        self.declare_parameter("target_distance_m", 0.30)
        self.declare_parameter("linear_speed", 0.10)
        self.declare_parameter("distance_tolerance_m", 0.01)
        self.declare_parameter("inter_attempt_delay_sec", 10.0)
        self.declare_parameter("confidence_threshold", 0.65)
        self.declare_parameter("max_frame_age_sec", 3.0)
        self.declare_parameter("max_odom_age_sec", 1.0)
        self.declare_parameter("max_move_time_sec", 6.0)
        self.declare_parameter("max_width", 640)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("max_tokens", 2048)
        self.declare_parameter("reasoning_max_tokens", 256)
        self.declare_parameter("reasoning_effort", "low")
        self.declare_parameter("exclude_reasoning", True)
        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("http_timeout_sec", 60.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("api_key_env", "OPENROUTER_API_KEY")
        self.declare_parameter("http_referer", os.environ.get("OPENROUTER_HTTP_REFERER", ""))
        self.declare_parameter("app_title", os.environ.get("OPENROUTER_APP_TITLE", "QBot Vision Forward Agent"))

    def _load_parameters(self):
        self.image_topic = self.get_parameter("image_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.model = self.get_parameter("model").value
        self.attempts = int(self.get_parameter("attempts").value)
        self.target_distance_m = float(self.get_parameter("target_distance_m").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.distance_tolerance_m = float(self.get_parameter("distance_tolerance_m").value)
        self.inter_attempt_delay_sec = float(self.get_parameter("inter_attempt_delay_sec").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.max_frame_age_sec = float(self.get_parameter("max_frame_age_sec").value)
        self.max_odom_age_sec = float(self.get_parameter("max_odom_age_sec").value)
        self.max_move_time_sec = float(self.get_parameter("max_move_time_sec").value)
        self.max_width = int(self.get_parameter("max_width").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.max_tokens = int(self.get_parameter("max_tokens").value)
        self.reasoning_max_tokens = int(self.get_parameter("reasoning_max_tokens").value)
        self.reasoning_effort = self.get_parameter("reasoning_effort").value
        self.exclude_reasoning = bool(self.get_parameter("exclude_reasoning").value)
        self.temperature = float(self.get_parameter("temperature").value)
        self.http_timeout_sec = float(self.get_parameter("http_timeout_sec").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.api_key_env = self.get_parameter("api_key_env").value
        self.http_referer = self.get_parameter("http_referer").value
        self.app_title = self.get_parameter("app_title").value

    def _image_cb(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_count += 1
            self.latest_image_time = time.monotonic()

    def _odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg
            self.latest_odom_time = time.monotonic()

    def _control_loop(self):
        if self.state == "done":
            return

        if self.state == "moving":
            self._continue_motion()
            return

        if self.state == "requesting":
            return

        if self.attempt_index >= self.attempts:
            self._stop_robot()
            self.state = "done"
            self._publish_status("finished", attempts=self.attempt_index)
            self.create_timer(0.5, self._shutdown_once)
            return

        if time.monotonic() < self.next_attempt_time:
            return

        self._start_attempt()

    def _start_attempt(self):
        snapshot = self._get_snapshot()
        if snapshot is None:
            self._wait_for_dependency(reason="missing_image_or_odom")
            return

        image_msg, image_count, image_age, odom_msg, odom_age = snapshot
        if image_age > self.max_frame_age_sec:
            self._wait_for_dependency(
                reason="stale_image",
                image_age=round(image_age, 3),
            )
            return
        if odom_age > self.max_odom_age_sec:
            self._wait_for_dependency(
                reason="stale_odom",
                odom_age=round(odom_age, 3),
            )
            return

        self.attempt_index += 1
        self.state = "requesting"
        self._publish_status(
            "attempt_started",
            attempt=self.attempt_index,
            image_count=image_count,
            image_stamp=format_msg_stamp(image_msg),
            image_age=round(image_age, 3),
            odom_age=round(odom_age, 3),
        )
        self.request_thread = threading.Thread(
            target=self._request_worker,
            args=(image_msg, image_count),
            daemon=True,
        )
        self.request_thread.start()

    def _wait_for_dependency(self, **fields):
        now = time.monotonic()
        if now - self.last_dependency_log < 5.0:
            return
        fields.setdefault("attempts_completed", self.attempt_index)
        fields.setdefault("attempts_total", self.attempts)
        fields.setdefault("image_publishers", self.count_publishers(self.image_topic))
        fields.setdefault("odom_publishers", self.count_publishers(self.odom_topic))
        self._publish_status("waiting_for_fresh_inputs", **fields)
        self.last_dependency_log = now

    def _get_snapshot(self):
        with self.lock:
            if self.latest_image is None or self.latest_odom is None:
                return None
            now = time.monotonic()
            return (
                self.latest_image,
                self.latest_image_count,
                now - self.latest_image_time,
                self.latest_odom,
                now - self.latest_odom_time,
            )

    def _request_worker(self, image_msg, image_count):
        try:
            image_data_url, frame_info = self._image_msg_to_data_url(image_msg, image_count)
            self._publish_status("image_selected", **frame_info)
            response = self._call_openrouter(image_data_url)
            verdict = parse_verdict(response)
        except Exception as exc:  # pylint: disable=broad-except
            self._finish_attempt("blocked", reason="request_failed", detail=str(exc))
            return

        clear = bool(verdict["path_clear"])
        confidence = float(verdict["confidence"])
        reason = str(verdict["reason"])
        self._publish_status(
            "verdict",
            attempt=self.attempt_index,
            path_clear=clear,
            confidence=round(confidence, 3),
            reason=reason,
        )

        if not clear:
            self._finish_attempt("blocked", reason=reason, confidence=round(confidence, 3))
            return
        if confidence < self.confidence_threshold:
            self._finish_attempt(
                "blocked",
                reason="low_confidence",
                confidence=round(confidence, 3),
                threshold=self.confidence_threshold,
            )
            return

        if self.dry_run:
            self._finish_attempt(
                "dry_run_clear",
                reason=reason,
                confidence=round(confidence, 3),
            )
            return

        with self.lock:
            odom_msg = self.latest_odom
            odom_age = time.monotonic() - self.latest_odom_time if odom_msg else float("inf")
        if odom_msg is None or odom_age > self.max_odom_age_sec:
            self._finish_attempt(
                "blocked",
                reason="stale_odom_before_move",
                odom_age=round(odom_age, 3),
            )
            return

        pose = odom_msg.pose.pose.position
        self.start_x = float(pose.x)
        self.start_y = float(pose.y)
        self.move_started_at = time.monotonic()
        self.state = "moving"
        self._publish_status(
            "moving",
            attempt=self.attempt_index,
            target_distance_m=self.target_distance_m,
            linear_speed=self.linear_speed,
        )

    def _finish_attempt(self, event, **fields):
        self._stop_robot()
        self._publish_status(event, attempt=self.attempt_index, **fields)
        self.next_attempt_time = time.monotonic() + self.inter_attempt_delay_sec
        self.state = "waiting"

    def _continue_motion(self):
        with self.lock:
            odom_msg = self.latest_odom
            odom_age = time.monotonic() - self.latest_odom_time if odom_msg else float("inf")

        if odom_msg is None or odom_age > self.max_odom_age_sec:
            self._finish_attempt(
                "move_aborted",
                reason="stale_odom",
                odom_age=round(odom_age, 3),
            )
            return

        pose = odom_msg.pose.pose.position
        distance = math.hypot(float(pose.x) - self.start_x, float(pose.y) - self.start_y)
        if distance >= max(0.0, self.target_distance_m - self.distance_tolerance_m):
            self._finish_attempt(
                "move_complete",
                distance_m=round(distance, 3),
            )
            return

        elapsed = time.monotonic() - self.move_started_at
        if elapsed > self.max_move_time_sec:
            self._finish_attempt(
                "move_aborted",
                reason="move_timeout",
                distance_m=round(distance, 3),
                elapsed_sec=round(elapsed, 3),
            )
            return

        self._publish_cmd(self.linear_speed, 0.0)

    def _image_msg_to_data_url(self, msg, image_count):
        frame = compressed_image_to_bgr(msg)
        input_hash = short_hash(msg.data)
        height, width = frame.shape[:2]
        if self.max_width > 0 and width > self.max_width:
            scale = self.max_width / float(width)
            frame = cv2.resize(
                frame,
                (self.max_width, max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("OpenCV failed to JPEG-encode the camera frame.")

        encoded_bytes = encoded.tobytes()
        sent_hash = short_hash(encoded_bytes)
        encoded_b64 = base64.b64encode(encoded_bytes).decode("ascii")
        frame_info = {
            "attempt": self.attempt_index,
            "image_count": image_count,
            "stamp": format_msg_stamp(msg),
            "width": int(frame.shape[1]),
            "height": int(frame.shape[0]),
            "input_hash": input_hash,
            "sent_hash": sent_hash,
        }
        return f"data:image/jpeg;base64,{encoded_b64}", frame_info

    def _call_openrouter(self, image_data_url):
        prompt = (
            "Look at this forward-facing robot camera image. Decide whether the "
            "robot can safely drive straight forward 30 centimeters right now. "
            "Return only strict JSON with exactly these keys: "
            '{"path_clear": true|false, "confidence": 0.0-1.0, "reason": "short reason"}. '
            "Do not wrap the JSON in markdown or a code fence. The first character "
            "of your response must be { and the last character must be }. "
            "Set path_clear false if there is any person, obstacle, wall, furniture, "
            "drop-off, narrow gap, or uncertainty in the immediate forward path. "
            "However, IGNORE small cracks, lines, or seams in the ground surface. "
            "These do not constitute obstacles and should not prevent forward motion."
        )
        messages = [
            {
                "role": "system",
                "content": (
                    "You are a conservative robot safety vision checker. "
                    "Only return valid JSON. Do not include markdown."
                ),
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {"url": image_data_url}},
                ],
            },
        ]
        payload = {
            "model": self.model,
            "messages": messages,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
            "reasoning": self._reasoning_config(),
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "X-OpenRouter-Title": self.app_title,
        }
        if self.http_referer:
            headers["HTTP-Referer"] = self.http_referer

        request = urllib.request.Request(
            OPENROUTER_CHAT_URL,
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            headers=headers,
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.http_timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"HTTP {exc.code} from OpenRouter: {detail}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"Could not reach OpenRouter: {exc}") from exc

    def _reasoning_config(self):
        config = {"exclude": self.exclude_reasoning}
        if self.reasoning_max_tokens > 0:
            config["max_tokens"] = self.reasoning_max_tokens
        else:
            config["effort"] = self.reasoning_effort
        return config

    def _publish_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _stop_robot(self):
        for _ in range(5):
            self._publish_cmd(0.0, 0.0)

    def _publish_status(self, event, **fields):
        payload = {"event": event, "time": round(time.time(), 3)}
        payload.update(fields)
        text = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(String(data=text))
        if event in {"blocked", "request_failed", "move_aborted"}:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)

    def _shutdown_once(self):
        if rclpy.ok():
            rclpy.shutdown()

    def shutdown(self):
        if not rclpy.ok():
            return
        self._stop_robot()
        self._publish_status("shutdown")


def compressed_image_to_bgr(msg):
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(f"Could not decode compressed image format '{msg.format}'.")
    return frame


def parse_verdict(response):
    try:
        choice = response["choices"][0]
        message = choice["message"]
        content = message.get("content")
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(f"Unexpected OpenRouter response: {response}") from exc

    if content is None:
        reasoning = message.get("reasoning")
        reasoning_chars = len(reasoning) if isinstance(reasoning, str) else 0
        raise RuntimeError(
            "Model response content was empty: "
            f"finish_reason={choice.get('finish_reason')}, "
            f"reasoning_chars={reasoning_chars}"
        )

    if isinstance(content, list):
        content = "".join(
            item.get("text", "")
            for item in content
            if isinstance(item, dict) and item.get("type") == "text"
        )
    if not isinstance(content, str):
        raise RuntimeError(f"Model response was not text: {content}")

    text = content.strip()
    try:
        verdict = json.loads(text)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Model did not return strict JSON: {text}") from exc

    if not isinstance(verdict, dict):
        raise RuntimeError(f"Verdict JSON is not an object: {verdict}")
    if "path_clear" not in verdict or "confidence" not in verdict or "reason" not in verdict:
        raise RuntimeError(f"Verdict JSON missing required keys: {verdict}")
    if not isinstance(verdict["path_clear"], bool):
        raise RuntimeError(f"path_clear must be boolean: {verdict}")
    try:
        verdict["confidence"] = float(verdict["confidence"])
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"confidence must be numeric: {verdict}") from exc
    if not 0.0 <= verdict["confidence"] <= 1.0:
        raise RuntimeError(f"confidence must be between 0 and 1: {verdict}")
    verdict["reason"] = str(verdict["reason"])
    return verdict


def short_hash(data):
    return hashlib.sha256(data).hexdigest()[:12]


def format_msg_stamp(msg):
    stamp = msg.header.stamp
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def main():
    rclpy.init()
    node = VisionForwardAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
