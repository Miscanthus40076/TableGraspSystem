from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.adapter import RobotAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Simple gripper command test through the ROS2 bridge.")
    parser.add_argument("--command", choices=["open", "close"], default="open", help="Single gripper command to send.")
    parser.add_argument("--toggle", action="store_true", help="Send open then close in sequence.")
    parser.add_argument("--delay", type=float, default=2.0, help="Delay in seconds between toggle commands.")
    parser.add_argument("--dry-run", action="store_true", help="Print commands without sending ROS2 messages.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    adapter = RobotAdapter(dry_run=args.dry_run)

    if args.toggle:
        print("Sending gripper open...")
        print(adapter.send_gripper("open"))
        time.sleep(max(args.delay, 0.0))
        print("Sending gripper close...")
        print(adapter.send_gripper("close"))
        return

    print(f"Sending gripper {args.command}...")
    print(adapter.send_gripper(args.command))


if __name__ == "__main__":
    main()
