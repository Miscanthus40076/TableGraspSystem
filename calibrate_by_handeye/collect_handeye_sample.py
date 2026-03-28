#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.detect_charuco import (  # noqa: E402
    board_center_offset_m,
    build_board,
    detect_charuco,
    intrinsics_to_camera_matrix,
)
from calibrate_by_handeye.start_handeye_session import (  # noqa: E402
    board_is_valid_6x6,
    board_validity_message,
    build_manual_runtime_with_fallback,
    build_ee_pose_summary,
    choose_device,
    get_manual_frame_bundle,
    start_control_stack,
    stop_manual_runtime,
    wait_for_joint_state,
    write_sample,
)
from robot.runtime_bridge_client import RuntimeBridgeClient  # noqa: E402


def parse_args() -> argparse.Namespace:
    """单样本采集参数。

    这个脚本面向“拍一条就退出”的场景，因此保留了和连续采样脚本同样的
    相机、板定义和 bridge 参数，方便两边数据结构完全一致。
    """
    parser = argparse.ArgumentParser(description="Collect one manual eye-to-hand sample.")
    parser.add_argument("--serial", required=True)
    parser.add_argument("--color-width", type=int, default=1280)
    parser.add_argument("--color-height", type=int, default=720)
    parser.add_argument("--color-fps", type=int, default=15)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--dictionary", type=str, default="DICT_4X4_50")
    parser.add_argument("--squares-x", type=int, default=11)
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-length-mm", type=float, default=15.0)
    parser.add_argument("--marker-length-mm", type=float, default=11.0)
    parser.add_argument("--legacy-pattern", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    parser.add_argument("--samples-root", type=Path, default=PROJECT_ROOT / "calibrate_by_handeye" / "samples")
    parser.add_argument("--sample-name", type=str, default=None)
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=8765)
    parser.add_argument("--start-driver", action="store_true")
    parser.add_argument("--start-full-stack", action="store_true")
    parser.add_argument("--disable-servos", action="store_true")
    parser.add_argument("--bridge-wait", type=float, default=2.0)
    parser.add_argument(
        "--urdf-path",
        type=Path,
        default=Path("/home/misca/starai_ws/src/fashionstar-starai-arm-ros2/src/cello_description/urdf/cello_description.urdf"),
    )
    parser.add_argument("--ee-link", type=str, default="link6")
    return parser.parse_args()


def main() -> None:
    """单条样本采集入口。

    它复用了 start_handeye_session.py 里的运行时/保存逻辑，只是把交互式循环
    收缩成“取 joint_state -> 抓一帧 -> 检测板 -> 保存一条 sample.json”。
    """
    args = parse_args()
    sample_name = args.sample_name or datetime.now().strftime("sample_%Y%m%d_%H%M%S")
    bridge = RuntimeBridgeClient(host=args.bridge_host, port=args.bridge_port, timeout_sec=10.0)
    if args.start_driver or args.start_full_stack or args.disable_servos:
        start_control_stack(bridge, args.bridge_wait, disable_servos=bool(args.disable_servos))

    joint_state = wait_for_joint_state(bridge, timeout_sec=max(args.bridge_wait, 0.0) + 8.0)
    ee_pose = build_ee_pose_summary(args, joint_state)
    if ee_pose is None:
        raise RuntimeError("No valid joint state / ee pose available.")

    device = choose_device(args.serial)
    runtime, stream_spec = build_manual_runtime_with_fallback(device, args)
    dictionary, board = build_board(args)

    try:
        for _ in range(max(int(args.warmup), 0)):
            try:
                _ = get_manual_frame_bundle(runtime, stream_spec)
            except RuntimeError:
                pass
        bundle = get_manual_frame_bundle(runtime, stream_spec)
    finally:
        stop_manual_runtime(runtime)

    camera_matrix, dist_coeffs = intrinsics_to_camera_matrix(bundle["color_intrinsics"])
    overlay, detection = detect_charuco(
        bundle["color"],
        camera_matrix,
        dist_coeffs,
        dictionary,
        board,
        args.square_length_mm / 1000.0,
        board_center_offset_m(args.squares_x, args.squares_y, args.square_length_mm / 1000.0),
    )
    # 单次采样同样坚持只接收完整 25/25 的板位姿，保证和连续采样目录里的样本质量标准一致。
    if not board_is_valid_6x6(detection):
        raise RuntimeError(
            "The full 6x6 board is not detected in this frame. "
            f"reason={board_validity_message(detection)}"
        )

    sample_dir = args.samples_root / sample_name
    sample = write_sample(
        sample_dir=sample_dir,
        sample_name=sample_name,
        bundle=bundle,
        detection=detection,
        ee_pose=ee_pose,
        joint_state=joint_state,
        args=args,
        overlay=overlay,
    )
    print(json.dumps(sample, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
