from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.runtime_bridge_client import RuntimeBridgeClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Healthcheck for the runtime bridge.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--start-driver", action="store_true")
    parser.add_argument("--wait", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = RuntimeBridgeClient(host=args.host, port=args.port, timeout_sec=10.0)

    try:
        status = client.status()
    except Exception as exc:  # noqa: BLE001
        print(f"bridge unavailable: {exc}")
        return 2

    if not status.get("ok"):
        print(f"bridge status failed: {status}")
        return 3

    if args.start_driver:
        result = client.start_driver()
        if not result.get("ok"):
            print(f"failed to start driver: {result}")
            return 4
        time.sleep(max(args.wait, 0.0))

    joint_state = client.get_joint_state()
    if not joint_state.get("ok"):
        print(f"joint state unavailable: {joint_state}")
        return 5

    names = joint_state["joint_state"].get("joint_names", [])
    positions = joint_state["joint_state"].get("joint_positions", [])
    servo_angles = joint_state["joint_state"].get("approx_servo_angles_deg", [])

    if len(names) != 7 or len(positions) != 7 or len(servo_angles) != 7:
        print(f"unexpected joint payload: {joint_state}")
        return 6

    print("runtime bridge healthcheck passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
