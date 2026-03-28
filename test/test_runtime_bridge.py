from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.runtime_bridge_client import RuntimeBridgeClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Runtime bridge integration test.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--start-driver", action="store_true", help="Ask the bridge to start driver.launch.py")
    parser.add_argument("--start-full-stack", action="store_true", help="Ask the bridge to start driver + MoveIt stack")
    parser.add_argument("--send-open", action="store_true", help="Send one gripper open command")
    parser.add_argument("--send-close", action="store_true", help="Send one gripper close command")
    parser.add_argument("--toggle-gripper", action="store_true", help="Send open then close")
    parser.add_argument("--stop-all", action="store_true", help="Stop all ROS processes started by the bridge")
    parser.add_argument("--wait", type=float, default=1.0, help="Wait time between commands in seconds")
    return parser.parse_args()


def print_json(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=True, indent=2))


def main() -> None:
    args = parse_args()
    client = RuntimeBridgeClient(host=args.host, port=args.port, timeout_sec=10.0)

    print("Bridge status:")
    print_json(client.status())

    if args.start_driver:
        print("Starting driver through bridge...")
        print_json(client.start_driver())
        time.sleep(max(args.wait, 0.0))

    if args.start_full_stack:
        print("Starting full stack through bridge...")
        print_json(client.start_full_stack())
        time.sleep(max(args.wait, 0.0))

    print("Latest joint state:")
    print_json(client.get_joint_state())

    if args.send_open:
        print("Sending gripper open...")
        print_json(client.send_gripper("open"))
        time.sleep(max(args.wait, 0.0))
        print("Latest joint state after gripper command:")
        print_json(client.get_joint_state())

    if args.send_close:
        print("Sending gripper close...")
        print_json(client.send_gripper("close"))
        time.sleep(max(args.wait, 0.0))
        print("Latest joint state after gripper command:")
        print_json(client.get_joint_state())

    if args.toggle_gripper:
        print("Sending gripper open...")
        print_json(client.send_gripper("open"))
        time.sleep(max(args.wait, 0.0))
        print("Joint state after open:")
        print_json(client.get_joint_state())
        print("Sending gripper close...")
        print_json(client.send_gripper("close"))
        time.sleep(max(args.wait, 0.0))
        print("Joint state after close:")
        print_json(client.get_joint_state())

    if args.stop_all:
        print("Stopping all bridge-managed ROS processes...")
        print_json(client.stop_all())


if __name__ == "__main__":
    main()
