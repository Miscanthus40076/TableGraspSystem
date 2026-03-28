#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.robot_fk import compute_tcp_point_in_base


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Backfill base_point_xyz_m into calibration samples using forward kinematics.")
    parser.add_argument(
        "--samples-dir",
        type=Path,
        default=Path("/home/misca/TabletopGraspSystem/calibration/samples"),
        help="Root directory containing sample.json files.",
    )
    parser.add_argument(
        "--urdf-path",
        type=Path,
        default=Path("/home/misca/starai_ws/src/fashionstar-starai-arm-ros2/src/cello_description/urdf/cello_description.urdf"),
        help="URDF used for forward kinematics.",
    )
    parser.add_argument("--tcp-link", type=str, default="link7_left", help="Link name used as the TCP reference.")
    parser.add_argument("--tcp-offset-x-mm", type=float, default=0.0)
    parser.add_argument("--tcp-offset-y-mm", type=float, default=0.0)
    parser.add_argument("--tcp-offset-z-mm", type=float, default=0.0)
    parser.add_argument("--force", action="store_true", help="Overwrite existing base_point_xyz_m if present.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    tcp_offset_xyz_m = [
        float(args.tcp_offset_x_mm) / 1000.0,
        float(args.tcp_offset_y_mm) / 1000.0,
        float(args.tcp_offset_z_mm) / 1000.0,
    ]
    updated = 0
    skipped = 0
    for sample_path in sorted(args.samples_dir.rglob("sample.json")):
        sample = json.loads(sample_path.read_text(encoding="utf-8"))
        if sample.get("base_point_xyz_m") is not None and not args.force:
            skipped += 1
            continue
        joint_state = sample.get("joint_state", {}).get("joint_state")
        if not joint_state:
            skipped += 1
            continue
        joint_names = joint_state.get("joint_names") or []
        joint_positions = joint_state.get("joint_positions") or []
        if not joint_names or not joint_positions:
            skipped += 1
            continue
        joint_map = {str(name): float(pos) for name, pos in zip(joint_names, joint_positions)}
        base_point = compute_tcp_point_in_base(
            urdf_path=args.urdf_path,
            joint_positions=joint_map,
            tcp_link=args.tcp_link,
            tcp_offset_xyz_m=tcp_offset_xyz_m,
            root_link="base_link",
        )
        sample["base_point_xyz_m"] = [float(v) for v in base_point.tolist()]
        sample["tcp_reference"] = {
            "urdf_path": str(args.urdf_path),
            "tcp_link": args.tcp_link,
            "tcp_offset_xyz_m": tcp_offset_xyz_m,
        }
        sample_path.write_text(json.dumps(sample, ensure_ascii=True, indent=2), encoding="utf-8")
        updated += 1

    print(
        json.dumps(
            {
                "ok": True,
                "updated": updated,
                "skipped": skipped,
                "tcp_link": args.tcp_link,
                "tcp_offset_xyz_m": tcp_offset_xyz_m,
            },
            ensure_ascii=True,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
