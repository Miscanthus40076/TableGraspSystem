from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.adapter import RobotAdapter


POSE_PRESETS = {
    "cello_safe_high": {
        "position_xyz_m": (-0.278, 0.0, 0.438),
        "orientation_xyzw": (0.707, 0.0, -0.707, 0.0),
        "description": "Official cello tutorial pose, higher and safer.",
    },
    "cello_safe_low": {
        "position_xyz_m": (-0.479, 0.0, 0.369),
        "orientation_xyzw": (0.707, 0.0, -0.707, 0.0),
        "description": "Official cello tutorial pose, lower and closer.",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a fixed end-effector pose through the Python ROS2 bridge."
    )
    parser.add_argument(
        "--preset",
        choices=sorted(POSE_PRESETS.keys()),
        default="cello_safe_high",
        help="Named pose preset to send.",
    )
    parser.add_argument("--x", type=float, help="Override X position in meters.")
    parser.add_argument("--y", type=float, help="Override Y position in meters.")
    parser.add_argument("--z", type=float, help="Override Z position in meters.")
    parser.add_argument("--qx", type=float, help="Override quaternion x.")
    parser.add_argument("--qy", type=float, help="Override quaternion y.")
    parser.add_argument("--qz", type=float, help="Override quaternion z.")
    parser.add_argument("--qw", type=float, help="Override quaternion w.")
    parser.add_argument(
        "--list-presets",
        action="store_true",
        help="Print available presets and exit.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the ROS command without actually sending it.",
    )
    return parser.parse_args()


def build_pose(args: argparse.Namespace) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    preset = POSE_PRESETS[args.preset]
    px, py, pz = preset["position_xyz_m"]
    qx, qy, qz, qw = preset["orientation_xyzw"]

    position = (
        px if args.x is None else args.x,
        py if args.y is None else args.y,
        pz if args.z is None else args.z,
    )
    orientation = (
        qx if args.qx is None else args.qx,
        qy if args.qy is None else args.qy,
        qz if args.qz is None else args.qz,
        qw if args.qw is None else args.qw,
    )
    return position, orientation


def main() -> None:
    args = parse_args()

    if args.list_presets:
        for name, preset in POSE_PRESETS.items():
            print(
                f"{name}: position={preset['position_xyz_m']}, "
                f"orientation={preset['orientation_xyzw']} | {preset['description']}"
            )
        return

    position, orientation = build_pose(args)
    adapter = RobotAdapter(dry_run=args.dry_run)

    print(f"Sending pose preset: {args.preset}")
    print(f"position_xyz_m={position}")
    print(f"orientation_xyzw={orientation}")
    result = adapter.send_pose(position_xyz_m=position, orientation_xyzw=orientation)
    print(result)


if __name__ == "__main__":
    main()
