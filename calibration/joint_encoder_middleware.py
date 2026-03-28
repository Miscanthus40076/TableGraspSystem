#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


@dataclass(frozen=True)
class JointSample:
    stamp_sec: int
    stamp_nanosec: int
    joint_names: list[str]
    joint_positions: list[float]
    approx_servo_angles_deg: list[float]

    def to_dict(self) -> dict[str, object]:
        return {
            "stamp": {
                "sec": self.stamp_sec,
                "nanosec": self.stamp_nanosec,
            },
            "joint_names": self.joint_names,
            "joint_positions": self.joint_positions,
            "approx_servo_angles_deg": self.approx_servo_angles_deg,
        }


def joint_position_to_servo_angle_deg(joint_name: str, joint_position: float) -> float:
    """Invert the conversion used in robo_driver as closely as possible.

    For joint1~joint6, the driver publishes radians converted from servo angle degrees.
    For joint7_left, the driver publishes a gripper opening value in meters based on:
      meters = ((degrees - 100) / 100) * 0.032
    """

    if joint_name == "joint7_left":
        return (joint_position / 0.032) * 100.0 + 100.0
    return math.degrees(joint_position)


class JointEncoderMiddleware(Node):
    def __init__(self, topic_name: str, jsonl_path: Path | None = None) -> None:
        super().__init__("joint_encoder_middleware")
        self.topic_name = topic_name
        self.jsonl_path = jsonl_path
        self.subscription = self.create_subscription(
            JointState,
            self.topic_name,
            self._joint_state_callback,
            10,
        )
        self._last_payload: str | None = None
        self.get_logger().info(f"Listening to {self.topic_name}")
        if self.jsonl_path is not None:
            self.jsonl_path.parent.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Logging samples to {self.jsonl_path}")

    def _joint_state_callback(self, msg: JointState) -> None:
        joint_names = list(msg.name)
        joint_positions = [float(v) for v in msg.position]
        approx_servo_angles_deg = [
            round(joint_position_to_servo_angle_deg(name, pos), 6)
            for name, pos in zip(joint_names, joint_positions)
        ]
        sample = JointSample(
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
            joint_names=joint_names,
            joint_positions=[round(v, 9) for v in joint_positions],
            approx_servo_angles_deg=approx_servo_angles_deg,
        )
        payload = json.dumps(sample.to_dict(), ensure_ascii=True)

        # Avoid spamming identical frames on screen while still allowing logging.
        if payload != self._last_payload:
            print(payload, flush=True)
            self._last_payload = payload

        if self.jsonl_path is not None:
            with self.jsonl_path.open("a", encoding="utf-8") as fp:
                fp.write(payload + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Realtime joint-state middleware for calibration."
    )
    parser.add_argument(
        "--topic",
        default="/joint_states",
        help="JointState topic to subscribe to.",
    )
    parser.add_argument(
        "--jsonl-path",
        type=Path,
        default=None,
        help="Optional path to append JSONL samples.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        topic_check = subprocess.run(
            ["bash", "-lc", f"source /opt/ros/humble/setup.bash >/dev/null 2>&1 && source /home/misca/starai_ws/install/setup.bash >/dev/null 2>&1 && ros2 topic list"],
            check=False,
            text=True,
            capture_output=True,
        )
        if args.topic not in topic_check.stdout.splitlines():
            print(
                f"[joint_encoder_middleware] topic {args.topic} not found. "
                "Start the robot driver first, for example: "
                "`ros2 launch cello_moveit_config driver.launch.py`",
                flush=True,
            )
    except Exception:
        pass

    rclpy.init()
    node = JointEncoderMiddleware(topic_name=args.topic, jsonl_path=args.jsonl_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
