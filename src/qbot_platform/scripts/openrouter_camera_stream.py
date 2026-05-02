#!/usr/bin/env python3
"""Sample a ROS 2 camera topic and send frames to OpenRouter."""

import argparse
import base64
import hashlib
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


OPENROUTER_CHAT_URL = "https://openrouter.ai/api/v1/chat/completions"
DEFAULT_MODEL = "nvidia/nemotron-3-nano-omni-30b-a3b-reasoning:free"
DEFAULT_TOPIC = "/camera/color_image"


class OpenRouterCameraStream(Node):
    def __init__(self, args):
        super().__init__("openrouter_camera_stream")
        self.args = args
        self.bridge = CvBridge()
        self.api_key = os.environ.get(args.api_key_env)
        self.latest_msg = None
        self.latest_msg_count = 0
        self.latest_msg_time = 0.0
        self.latest_msg_lock = threading.Lock()
        self.last_sent_msg_count = 0
        self.last_stale_log = 0.0
        self.request_lock = threading.Lock()
        self.last_waiting_log = 0.0

        if not args.diagnose_only and not self.api_key:
            raise RuntimeError(
                f"Set {args.api_key_env} to your OpenRouter API key before running."
            )

        self.input_msg_type = self._select_input_msg_type()
        self.subscription = self.create_subscription(
            self.input_msg_type,
            args.topic,
            self._image_callback,
            build_image_qos(args.qos),
        )
        self.description_pub = self.create_publisher(String, args.output_topic, 10)
        self.timer = self.create_timer(args.interval, self._timer_callback)

        self.get_logger().info(
            f"Watching {args.topic} ({self.input_msg_type.__name__}); "
            f"sending one frame every {args.interval:.1f}s "
            f"to {args.model}; publishing descriptions on {args.output_topic}"
        )

    def _select_input_msg_type(self):
        if self.args.input_type == "raw":
            return Image
        if self.args.input_type == "compressed":
            return CompressedImage

        topic_types = dict(self.get_topic_names_and_types()).get(self.args.topic, [])
        if "sensor_msgs/msg/CompressedImage" in topic_types:
            return CompressedImage
        if self.args.topic.endswith("/compressed"):
            return CompressedImage
        return Image

    def _image_callback(self, msg):
        with self.latest_msg_lock:
            self.latest_msg = msg
            self.latest_msg_count += 1
            self.latest_msg_time = time.monotonic()

    def _timer_callback(self):
        with self.latest_msg_lock:
            msg = self.latest_msg
            msg_count = self.latest_msg_count
            received_at = self.latest_msg_time

        if msg is None:
            now = time.monotonic()
            if now - self.last_waiting_log > 5.0:
                topic_types = dict(self.get_topic_names_and_types()).get(
                    self.args.topic, []
                )
                type_summary = ", ".join(topic_types) if topic_types else "unknown"
                self.get_logger().warning(
                    f"No image received yet on {self.args.topic}. "
                    f"publishers={self.count_publishers(self.args.topic)}, "
                    f"types={type_summary}, qos={self.args.qos}. "
                    "If publishers=0, start the camera node first, for example "
                    "`ros2 run qbot_platform rgbd --ros-args -r __node:=rgbd_camera`. "
                    "Run `ros2 topic hz <topic>` to confirm frames are flowing."
                )
                self.last_waiting_log = now
            return

        now = time.monotonic()
        frame_age = now - received_at
        if msg_count == self.last_sent_msg_count:
            self._warn_stale_frame(
                f"No newer frame has arrived since frame #{msg_count}; skipping."
            )
            return
        if self.args.max_frame_age > 0.0 and frame_age > self.args.max_frame_age:
            self._warn_stale_frame(
                f"Latest frame #{msg_count} is {frame_age:.1f}s old; skipping."
            )
            return

        if not self.request_lock.acquire(blocking=False):
            self.get_logger().debug("Previous OpenRouter request is still running.")
            return

        self.last_sent_msg_count = msg_count
        worker = threading.Thread(
            target=self._send_frame_worker,
            args=(msg, msg_count, received_at),
            daemon=True,
        )
        worker.start()

    def _warn_stale_frame(self, message):
        now = time.monotonic()
        if now - self.last_stale_log > 5.0:
            self.get_logger().warning(
                f"{message} The camera stream may be stalled; "
                "not sending stale pixels to OpenRouter."
            )
            self.last_stale_log = now

    def _send_frame_worker(self, msg, msg_count, received_at):
        started_at = time.monotonic()
        try:
            image_data_url, frame_info = self._image_msg_to_data_url(msg, msg_count)
            self.get_logger().info(
                "Sending frame "
                f"#{frame_info['count']} stamp={frame_info['stamp']} "
                f"size={frame_info['width']}x{frame_info['height']} "
                f"input_hash={frame_info['input_hash']} "
                f"sent_hash={frame_info['sent_hash']}"
            )
            if self.args.diagnose_only:
                return
            response = self._call_openrouter(image_data_url)
            answer = extract_response_text(response)
            self.description_pub.publish(String(data=answer))
            frame_age = started_at - received_at
            elapsed = time.monotonic() - started_at
            self.get_logger().info(
                f"What the model sees ({elapsed:.1f}s, frame age {frame_age:.1f}s):\n"
                f"{answer}"
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"OpenRouter camera request failed: {exc}")
        finally:
            self.request_lock.release()
            if self.args.once and rclpy.ok():
                rclpy.shutdown()

    def _image_msg_to_data_url(self, msg, msg_count):
        try:
            if isinstance(msg, CompressedImage):
                input_hash = short_hash(msg.data)
                frame = compressed_image_to_bgr(msg)
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                input_hash = short_hash(frame.tobytes())
        except CvBridgeError as exc:
            raise RuntimeError(f"Could not convert ROS image to OpenCV: {exc}") from exc

        height, width = frame.shape[:2]
        if self.args.max_width > 0 and width > self.args.max_width:
            scale = self.args.max_width / float(width)
            frame = cv2.resize(
                frame,
                (self.args.max_width, max(1, int(height * scale))),
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.args.jpeg_quality],
        )
        if not ok:
            raise RuntimeError("OpenCV failed to JPEG-encode the camera frame.")

        encoded_bytes = encoded.tobytes()
        sent_hash = short_hash(encoded_bytes)
        if self.args.save_debug_frames:
            self._save_debug_frame(encoded_bytes, msg, msg_count, sent_hash)

        encoded_b64 = base64.b64encode(encoded_bytes).decode("ascii")
        frame_info = {
            "count": msg_count,
            "stamp": format_msg_stamp(msg),
            "width": frame.shape[1],
            "height": frame.shape[0],
            "input_hash": input_hash,
            "sent_hash": sent_hash,
        }
        return f"data:image/jpeg;base64,{encoded_b64}", frame_info

    def _save_debug_frame(self, encoded_bytes, msg, msg_count, sent_hash):
        os.makedirs(self.args.save_debug_frames, exist_ok=True)
        stamp = format_msg_stamp(msg).replace(".", "_")
        filename = f"frame_{msg_count:06d}_{stamp}_{sent_hash}.jpg"
        path = os.path.join(self.args.save_debug_frames, filename)
        with open(path, "wb") as debug_file:
            debug_file.write(encoded_bytes)
        self.get_logger().info(f"Saved sent frame to {path}")

    def _call_openrouter(self, image_data_url):
        messages = [
            {
                "role": "system",
                "content": self.args.system_prompt,
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": self.args.prompt,
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image_data_url,
                        },
                    },
                ],
            },
        ]

        payload = {
            "model": self.args.model,
            "messages": messages,
            "max_tokens": self.args.max_tokens,
            "temperature": self.args.temperature,
        }

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
            "X-OpenRouter-Title": self.args.app_title,
        }
        if self.args.http_referer:
            headers["HTTP-Referer"] = self.args.http_referer

        body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        request = urllib.request.Request(
            OPENROUTER_CHAT_URL,
            data=body,
            headers=headers,
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.args.timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            details = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"HTTP {exc.code} from OpenRouter: {details}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"Could not reach OpenRouter: {exc}") from exc


def extract_response_text(response):
    try:
        content = response["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(f"Unexpected OpenRouter response: {response}") from exc

    if isinstance(content, str):
        return content.strip()

    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict) and item.get("type") == "text":
                parts.append(item.get("text", ""))
        text = "".join(parts).strip()
        if text:
            return text

    raise RuntimeError(f"OpenRouter response did not contain text: {response}")


def compressed_image_to_bgr(msg):
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError(
            f"OpenCV failed to decode compressed image with format '{msg.format}'."
        )
    return frame


def short_hash(data):
    return hashlib.sha256(data).hexdigest()[:12]


def format_msg_stamp(msg):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return "no_stamp"
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def build_image_qos(qos_name):
    if qos_name == "sensor_data":
        return qos_profile_sensor_data

    profile = QoSProfile(depth=10)
    if qos_name == "best_effort":
        profile.reliability = ReliabilityPolicy.BEST_EFFORT
    elif qos_name == "reliable":
        profile.reliability = ReliabilityPolicy.RELIABLE
    return profile


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe to a ROS 2 sensor_msgs/Image topic and periodically send "
            "the latest frame to OpenRouter."
        )
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=(
            "Camera image topic. This qbot_platform package's RGBD node "
            "publishes /camera/color_image. The RealSense ROS 2 wrapper default "
            "is /camera/camera/color/image_raw."
        ),
    )
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument(
        "--input-type",
        choices=["auto", "raw", "compressed"],
        default="auto",
        help="Input message type. Auto selects compressed for /compressed topics.",
    )
    parser.add_argument(
        "--qos",
        choices=["default", "reliable", "best_effort", "sensor_data"],
        default="default",
        help=(
            "Subscriber QoS. Use default/reliable for this qbot_platform RGBD "
            "publisher; try sensor_data or best_effort for RealSense topics."
        ),
    )
    parser.add_argument(
        "--api-key-env",
        default="OPENROUTER_API_KEY",
        help="Environment variable containing the OpenRouter API key.",
    )
    parser.add_argument(
        "--prompt",
        default=(
            "Say what you see in this live robot camera frame. Be concise and "
            "mention visible obstacles, people, notable objects, and anything "
            "relevant for safe navigation."
        ),
    )
    parser.add_argument(
        "--system-prompt",
        default=(
            "You are a concise robot vision assistant. Focus on visible facts "
            "from the image. Output only what you see and avoid guessing beyond "
            "the frame."
        ),
    )
    parser.add_argument(
        "--output-topic",
        default="/openrouter/camera_description",
        help="ROS topic where the model's text description is published.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=10.0,
        help="Seconds between OpenRouter requests.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Send one frame and exit.",
    )
    parser.add_argument(
        "--diagnose-only",
        action="store_true",
        help="Log frame stamps/hashes without calling OpenRouter.",
    )
    parser.add_argument(
        "--max-frame-age",
        type=float,
        default=5.0,
        help="Skip frames older than this many seconds. Use 0 to disable.",
    )
    parser.add_argument(
        "--save-debug-frames",
        default="",
        help="Optional directory where the exact JPEG frames sent to OpenRouter are saved.",
    )
    parser.add_argument(
        "--max-width",
        type=int,
        default=960,
        help="Downscale frames wider than this before upload. Use 0 to disable.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=80,
        help="JPEG quality from 1 to 100.",
    )
    parser.add_argument(
        "--max-tokens",
        type=int,
        default=256,
        help="Maximum response tokens.",
    )
    parser.add_argument(
        "--temperature",
        type=float,
        default=0.2,
        help="Model sampling temperature.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="OpenRouter HTTP timeout in seconds.",
    )
    parser.add_argument(
        "--http-referer",
        default=os.environ.get("OPENROUTER_HTTP_REFERER", ""),
        help="Optional OpenRouter attribution URL.",
    )
    parser.add_argument(
        "--app-title",
        default=os.environ.get("OPENROUTER_APP_TITLE", "QBot Camera Stream"),
        help="Optional OpenRouter attribution title.",
    )
    return parser


def validate_args(args):
    if not args.diagnose_only and not os.environ.get(args.api_key_env):
        raise ValueError(f"set {args.api_key_env} to your OpenRouter API key")
    if args.interval <= 0.0:
        raise ValueError("--interval must be greater than 0")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")
    if args.max_tokens <= 0:
        raise ValueError("--max-tokens must be greater than 0")
    if args.timeout <= 0.0:
        raise ValueError("--timeout must be greater than 0")


def main():
    parser = build_arg_parser()
    cli_args = remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(cli_args)
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    rclpy.init(args=sys.argv)
    node = OpenRouterCameraStream(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
