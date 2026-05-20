#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import select
import sys
import termios
import time
import tty
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.detect_charuco import (  # noqa: E402
    board_center_offset_m,
    build_board,
    detect_charuco,
    enumerate_devices,
    intrinsics_to_camera_matrix,
)
from calibration.robot_fk import compute_link_transform  # noqa: E402
from robot.runtime_bridge_client import RuntimeBridgeClient  # noqa: E402


WINDOW_NAME = "handeye_manual_session"
REQUIRED_CHARUCO_6X6_CORNERS = 25


def ensure_qt_fonts() -> None:
    """Mirror the official preview fix so OpenCV Qt windows can render text reliably."""

    source_dir = Path("/usr/share/fonts/truetype/dejavu")
    target_dir = (
        Path(sys.prefix)
        / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages/cv2/qt/fonts"
    )
    if not source_dir.is_dir():
        return
    target_dir.mkdir(parents=True, exist_ok=True)
    for font_path in source_dir.glob("*.ttf"):
        link_path = target_dir / font_path.name
        if not link_path.exists():
            link_path.symlink_to(font_path)
    os.environ.setdefault("QT_QPA_FONTDIR", str(target_dir))


ensure_qt_fonts()


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    r = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(r))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    else:
        if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
    quat = np.asarray([x, y, z, w], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-12)
    return quat.tolist()


class TerminalKeyReader:
    """Read single-key terminal commands without forcing the user to type full words."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._old_attrs: list[Any] | None = None

    def __enter__(self) -> "TerminalKeyReader":
        if sys.stdin.isatty():
            self._fd = sys.stdin.fileno()
            self._old_attrs = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        if self._fd is not None and self._old_attrs is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_attrs)

    def poll(self) -> str | None:
        if self._fd is None:
            return None
        readable, _, _ = select.select([self._fd], [], [], 0.0)
        if not readable:
            return None
        try:
            return sys.stdin.read(1)
        except (OSError, EOFError):
            return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual eye-to-hand calibration session.")
    parser.add_argument("--list-devices", action="store_true", help="List connected RealSense devices and exit.")
    parser.add_argument("--serial", type=str, default=None, help="Target RealSense serial number.")
    parser.add_argument("--color-width", type=int, default=1280)
    parser.add_argument("--color-height", type=int, default=720)
    parser.add_argument("--color-fps", type=int, default=15)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--dictionary", type=str, default="DICT_4X4_50")
    parser.add_argument("--squares-x", type=int, default=11)
    parser.add_argument("--squares-y", type=int, default=8)
    parser.add_argument("--square-length-mm", type=float, default=15.0)
    parser.add_argument("--marker-length-mm", type=float, default=11.0)
    parser.add_argument("--legacy-pattern", action="store_true")
    parser.add_argument("--save-overlay", action="store_true")
    parser.add_argument("--samples-root", type=Path, default=PROJECT_ROOT / "calibrate_by_handeye" / "samples")
    parser.add_argument("--target-samples", type=int, default=0, help="Optional stop condition. 0 means unlimited.")
    parser.add_argument("--bridge-host", default="127.0.0.1")
    parser.add_argument("--bridge-port", type=int, default=8765)
    parser.add_argument("--start-driver", action="store_true")
    parser.add_argument("--start-full-stack", action="store_true")
    parser.add_argument("--disable-servos", action="store_true", help="Start calibration in free-drive mode by disabling servo lock.")
    parser.add_argument("--bridge-wait", type=float, default=2.0)
    parser.add_argument(
        "--urdf-path",
        type=Path,
        default=Path("/home/misca/starai_ws/src/fashionstar-starai-arm-ros2/src/cello_description/urdf/cello_description.urdf"),
    )
    parser.add_argument("--ee-link", type=str, default="link6")
    return parser.parse_args()


def board_is_valid_6x6(detection: dict[str, Any]) -> bool:
    return (
        bool(detection.get("pose_ok"))
        and str(detection.get("pose_source")) == "charuco"
        and int(detection.get("charuco_corner_count", 0)) == REQUIRED_CHARUCO_6X6_CORNERS
    )


def board_validity_message(detection: dict[str, Any]) -> str:
    corner_count = int(detection.get("charuco_corner_count", 0))
    pose_source = str(detection.get("pose_source"))
    if corner_count != REQUIRED_CHARUCO_6X6_CORNERS:
        return f"charuco_{corner_count}_of_{REQUIRED_CHARUCO_6X6_CORNERS}"
    if pose_source != "charuco":
        return f"pose_source_{pose_source}"
    if not detection.get("pose_ok"):
        return "pose_not_ok"
    return "ok"


def choose_device(serial: str | None) -> Any:
    devices = enumerate_devices()
    if serial is None:
        if not devices:
            raise RuntimeError("No RealSense devices detected.")
        return devices[0]
    for dev in devices:
        if dev.serial_number == serial:
            return dev
    raise RuntimeError(f"Target serial not found: {serial}")


def build_manual_runtime_with_fallback(device: Any, args: argparse.Namespace) -> tuple[Any, dict[str, Any]]:
    requested = {
        "mode": "official_aligned",
        "color": (int(args.color_width), int(args.color_height), int(args.color_fps)),
        "depth": (int(args.depth_width), int(args.depth_height), int(args.depth_fps)),
    }
    official_default = {
        "mode": "official_aligned",
        "color": (640, 480, 30),
        "depth": (640, 480, 30),
    }
    fallbacks = [requested, official_default]
    seen: set[tuple[str, tuple[int, int, int], tuple[int, int, int]]] = set()
    last_error: Exception | None = None
    for spec in fallbacks:
        combo = (spec["mode"], spec["color"], spec["depth"])
        if combo in seen:
            continue
        seen.add(combo)
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(device.serial_number)
        config.enable_stream(
            rs.stream.depth,
            int(spec["depth"][0]),
            int(spec["depth"][1]),
            rs.format.z16,
            int(spec["depth"][2]),
        )
        config.enable_stream(
            rs.stream.color,
            int(spec["color"][0]),
            int(spec["color"][1]),
            rs.format.bgr8,
            int(spec["color"][2]),
        )
        try:
            pipeline.start(config)
            runtime = SimpleNamespace(
                info=device,
                pipeline=pipeline,
                align=rs.align(rs.stream.color),
            )
            print(
                "[handeye] Using official RealSense stream "
                f"color={spec['color'][0]}x{spec['color'][1]}@{spec['color'][2]} "
                f"depth={spec['depth'][0]}x{spec['depth'][1]}@{spec['depth'][2]} "
                "(aligned depth transport, color used for calibration).",
                flush=True,
            )
            return runtime, spec
        except RuntimeError as exc:
            last_error = exc
            try:
                pipeline.stop()
            except Exception:
                pass
    raise RuntimeError(f"Failed to start RealSense manual calibration stream: {last_error}")


def get_manual_frame_bundle(runtime: Any, stream_spec: dict[str, Any]) -> dict[str, Any]:
    try:
        frames = runtime.pipeline.wait_for_frames(10000)
    except RuntimeError as exc:
        message = str(exc)
        if "Frame didn't arrive" in message:
            raise RuntimeError(
                "D435i stream timed out. Check whether another process is using the camera, "
                "then replug it and try again."
            ) from exc
        raise

    aligned_frames = runtime.align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame:
        raise RuntimeError("Missing aligned color or depth frame from RealSense pipeline.")

    color = np.asanyarray(color_frame.get_data())
    color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
    return {
        "serial_number": runtime.info.serial_number,
        "device_name": runtime.info.name,
        "firmware_version": runtime.info.firmware_version,
        "product_line": runtime.info.product_line,
        "usb_type_descriptor": runtime.info.usb_type_descriptor,
        "color": color,
        "timestamp_ms": aligned_frames.get_timestamp(),
        "frame_number": aligned_frames.get_frame_number(),
        "color_intrinsics": {
            "width": color_intrinsics.width,
            "height": color_intrinsics.height,
            "fx": color_intrinsics.fx,
            "fy": color_intrinsics.fy,
            "ppx": color_intrinsics.ppx,
            "ppy": color_intrinsics.ppy,
            "model": str(color_intrinsics.model),
            "coeffs": list(color_intrinsics.coeffs),
        },
    }


def stop_manual_runtime(runtime: Any | None) -> None:
    if runtime is None:
        return
    try:
        runtime.pipeline.stop()
    except RuntimeError:
        pass


def start_control_stack(bridge: RuntimeBridgeClient, wait_sec: float, *, disable_servos: bool = False) -> None:
    """准备标定所需的控制链。

    这里的语义和抓取脚本不同：
    - 手眼标定只需要能稳定读到 joint_state
    - 如果传入 disable_servos，则切到 driver_free，方便人工拖动机械臂采样
    - 如果 bridge 下已经有进程在跑，默认复用，避免把现场已有会话直接打断
    """
    status = bridge.status()
    if not status.get("ok"):
        raise RuntimeError(f"Failed to query runtime bridge status: {status}")
    running_names = {
        str(item.get("name"))
        for item in status.get("processes", [])
        if item.get("running")
    }
    if running_names:
        print(
            "[handeye] Runtime bridge already has running processes: "
            + ",".join(sorted(running_names)),
            flush=True,
        )
        if disable_servos:
            print("[handeye] Switching runtime bridge to driver_free mode for calibration.", flush=True)
            free_result = bridge.start_driver_free()
            if not free_result.get("ok"):
                raise RuntimeError(f"Failed to start driver_free mode through runtime bridge: {free_result}")
        else:
            print("[handeye] Reusing the current control stack without restarting it.", flush=True)
    elif disable_servos:
        free_result = bridge.start_driver_free()
        if not free_result.get("ok"):
            raise RuntimeError(f"Failed to start driver_free mode through runtime bridge: {free_result}")
    elif bridge.start_full_stack().get("ok") is False:
        raise RuntimeError("Failed to start full stack through runtime bridge.")
    time.sleep(max(wait_sec, 0.0))


def get_joint_position_map(joint_state_response: dict[str, Any]) -> dict[str, float]:
    state = joint_state_response.get("joint_state") or {}
    names = state.get("joint_names") or []
    positions = state.get("joint_positions") or []
    return {str(name): float(pos) for name, pos in zip(names, positions)}


def wait_for_joint_state(bridge: RuntimeBridgeClient, timeout_sec: float = 10.0) -> dict[str, Any]:
    """等待 bridge 至少返回一条 joint_state。

    这里故意只要求“有值”，不额外做 freshness 判定。
    标定阶段更关心人工拖动时能否持续读到当前位置，而不是 MoveIt 风格的严格新鲜度。
    """
    deadline = time.time() + timeout_sec
    last = {"ok": False, "message": "No joint state received yet"}
    while time.time() < deadline:
        last = bridge.get_joint_state()
        if last.get("ok"):
            return last
        time.sleep(0.25)
    return last


def build_ee_pose_summary(args: argparse.Namespace, joint_state: dict[str, Any]) -> dict[str, Any] | None:
    """把当前 joint_state 转成采样时刻的末端参考位姿。

    采样文件里最终保存的是 T_base_ee，而不是原始关节角本身。
    这样后续 solve/validate 都能直接按统一坐标约定消费样本。
    """
    if not joint_state.get("ok"):
        return None
    joint_map = get_joint_position_map(joint_state)
    if not joint_map:
        return None
    t_base_ee = compute_link_transform(
        urdf_path=args.urdf_path,
        joint_positions=joint_map,
        root_link="base_link",
        target_link=args.ee_link,
    )
    return {
        "T_base_ee": t_base_ee,
        "translation_xyz_m": t_base_ee[:3, 3].astype(float).tolist(),
        "quaternion_xyzw": rotation_matrix_to_quaternion_xyzw(t_base_ee[:3, :3]),
        "tool_axis_base_xyz": t_base_ee[:3, 2].astype(float).tolist(),
    }


def six_joint_servo_angles_deg(joint_state_response: dict[str, Any]) -> list[float | None]:
    payload = joint_state_response.get("joint_state") or {}
    names = payload.get("joint_names") or []
    approx = payload.get("approx_servo_angles_deg") or []
    mapping = {str(name): float(value) for name, value in zip(names, approx)}
    ordered = []
    for idx in range(1, 7):
        ordered.append(mapping.get(f"joint{idx}"))
    return ordered


def render_terminal_status(
    *,
    joint_state: dict[str, Any],
    detection: dict[str, Any],
    saved_count: int,
    target_samples: int,
) -> None:
    """用终端整屏刷新显示采样状态。

    这里是人工采样的主要反馈入口：
    - 板子是否完整识别到 25/25
    - 当前 6 个关节近似角度
    - 当前已经保存了多少条样本
    """
    progress_text = f"{saved_count}" if target_samples <= 0 else f"{saved_count}/{target_samples}"
    servo_angles = six_joint_servo_angles_deg(joint_state)
    lines = [
        "[handeye] Manual calibration status",
        f"progress: {progress_text}",
        f"board: charuco={int(detection.get('charuco_corner_count', 0))}/{REQUIRED_CHARUCO_6X6_CORNERS} "
        f"valid_6x6={board_is_valid_6x6(detection)} reason={board_validity_message(detection)}",
        "",
        "joint servo angles (deg):",
    ]
    for idx, value in enumerate(servo_angles, start=1):
        if value is None:
            lines.append(f"  joint{idx}: --")
        else:
            lines.append(f"  joint{idx}: {value:7.2f}")
    lines.extend(
        [
            "",
            "controls:",
            "  s = save current sample",
            "  q = quit session",
        ]
    )
    sys.stdout.write("\x1b[2J\x1b[H")
    sys.stdout.write("\n".join(lines) + "\n")
    sys.stdout.flush()


def draw_manual_overlay(
    image: np.ndarray,
    detection: dict[str, Any],
    saved_count: int,
    target_samples: int,
) -> np.ndarray:
    overlay = image.copy()
    valid = board_is_valid_6x6(detection)
    status_color = (0, 255, 0) if valid else (0, 0, 255)
    progress_text = f"{saved_count}" if target_samples <= 0 else f"{saved_count}/{target_samples}"
    lines = [
        f"manual_handeye progress={progress_text}",
        f"charuco={int(detection.get('charuco_corner_count', 0))}/{REQUIRED_CHARUCO_6X6_CORNERS}",
        f"valid_6x6={valid} reason={board_validity_message(detection)}",
        "terminal/window: s=save q=quit",
    ]
    for idx, line in enumerate(lines):
        cv2.putText(
            overlay,
            line,
            (16, 32 + idx * 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            status_color if idx < 3 else (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return overlay


def write_sample(
    *,
    sample_dir: Path,
    sample_name: str,
    bundle: dict[str, Any],
    detection: dict[str, Any],
    ee_pose: dict[str, Any],
    joint_state: dict[str, Any],
    args: argparse.Namespace,
    overlay: np.ndarray | None,
) -> dict[str, Any]:
    """把当前一帧完整落盘为 hand-eye 样本。

    sample.json 会同时保存三类信息：
    - 图像与板检测结果
    - 当时的 joint_state
    - 通过 FK 算出的 T_base_ee
    后续 solve/validate 都只依赖这份结构化样本，不回头访问现场状态。
    """
    sample_dir.mkdir(parents=True, exist_ok=True)
    color_path = sample_dir / "color.png"
    cv2.imwrite(str(color_path), bundle["color"])
    if args.save_overlay and overlay is not None:
        cv2.imwrite(str(sample_dir / "overlay.png"), overlay)

    sample = {
        "sample_name": sample_name,
        "serial_number": bundle["serial_number"],
        "image_path": str(color_path),
        "frame_number": int(bundle["frame_number"]),
        "timestamp_ms": float(bundle["timestamp_ms"]),
        "color_intrinsics": bundle["color_intrinsics"],
        "board_definition": {
            "dictionary": args.dictionary,
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_mm": args.square_length_mm,
            "marker_length_mm": args.marker_length_mm,
            "legacy_pattern": bool(args.legacy_pattern),
        },
        "board_detection": detection,
        "board_valid_6x6": True,
        "joint_state": joint_state,
        "ee_reference": {
            "urdf_path": str(args.urdf_path),
            "ee_link": args.ee_link,
            "T_base_ee": ee_pose["T_base_ee"].tolist(),
            "translation_xyz_m": ee_pose["translation_xyz_m"],
            "quaternion_xyzw": ee_pose["quaternion_xyzw"],
            "tool_axis_base_xyz": ee_pose["tool_axis_base_xyz"],
        },
    }
    (sample_dir / "sample.json").write_text(json.dumps(sample, ensure_ascii=True, indent=2), encoding="utf-8")
    return sample


def main() -> None:
    """人工连续采样入口。

    运行流程是：
    1. 可选启动控制栈或 free-drive
    2. 打开相机与窗口
    3. 持续检测 ChArUco 板，并在终端打印当前状态
    4. 用户按 s 时，仅在“关节状态可读 + 板完整 25/25”时保存样本
    """
    args = parse_args()
    if args.list_devices:
        devices = enumerate_devices()
        if not devices:
            print("No RealSense devices detected.")
            return
        print("Connected RealSense devices:")
        for dev in devices:
            print(f"- {dev.name} | serial={dev.serial_number}")
        return

    if not args.serial:
        raise RuntimeError("--serial is required for manual handeye calibration.")

    bridge = RuntimeBridgeClient(host=args.bridge_host, port=args.bridge_port, timeout_sec=10.0)
    if args.start_driver or args.start_full_stack or args.disable_servos:
        start_control_stack(bridge, args.bridge_wait, disable_servos=bool(args.disable_servos))

    input("[handeye] Make sure the calibration board is already clamped securely, then press Enter to open the manual calibration session...")

    device = choose_device(args.serial)
    dictionary, board = build_board(args)
    runtime, stream_spec = build_manual_runtime_with_fallback(device, args)
    session_root = args.samples_root / datetime.now().strftime("session_%Y%m%d_%H%M%S")
    saved_count = 0
    last_log_at = 0.0
    last_frame_warn_at = 0.0
    last_status_draw_at = 0.0
    last_overlay: np.ndarray | None = None
    last_detection: dict[str, Any] = {
        "charuco_corner_count": 0,
        "pose_ok": False,
        "pose_source": None,
    }
    last_bundle: dict[str, Any] | None = None
    last_joint_state: dict[str, Any] = {"ok": False, "message": "No joint state received yet"}

    print("[handeye] Manual session started.", flush=True)
    print("[handeye] Use your normal robot controls to move the board to different poses.", flush=True)
    print("[handeye] Save a sample only when the full 6x6 board is detected (25/25 corners).", flush=True)
    print("[handeye] Controls: `s` save current valid sample | `q` quit", flush=True)

    try:
        for idx in range(max(int(args.warmup), 0)):
            try:
                _ = get_manual_frame_bundle(runtime, stream_spec)
                print(f"[handeye] Camera warmup {idx + 1}/{max(int(args.warmup), 0)}...", flush=True)
            except RuntimeError as exc:
                print(
                    f"[handeye][warning] Warmup frame {idx + 1}/{max(int(args.warmup), 0)} failed: {exc}",
                    flush=True,
                )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        placeholder = np.zeros((max(int(args.color_height), 240), max(int(args.color_width), 320), 3), dtype=np.uint8)
        cv2.putText(
            placeholder,
            "Starting manual calibration session...",
            (40, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            placeholder,
            "Waiting for the first camera frame...",
            (40, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(WINDOW_NAME, placeholder)
        cv2.waitKey(1)
        with TerminalKeyReader() as key_reader:
            while True:
                try:
                    bundle = get_manual_frame_bundle(runtime, stream_spec)
                    last_bundle = bundle
                except RuntimeError as exc:
                    now = time.time()
                    if now - last_frame_warn_at >= 2.0:
                        print(f"[handeye][warning] Frame grab failed: {exc}", flush=True)
                        last_frame_warn_at = now
                    fresh_joint_state = bridge.get_joint_state()
                    if fresh_joint_state.get("ok"):
                        last_joint_state = fresh_joint_state
                    if last_overlay is not None:
                        overlay = draw_manual_overlay(last_overlay, last_detection, saved_count, int(args.target_samples))
                        cv2.imshow(WINDOW_NAME, overlay)
                    if now - last_status_draw_at >= 0.25:
                        render_terminal_status(
                            joint_state=last_joint_state,
                            detection=last_detection,
                            saved_count=saved_count,
                            target_samples=int(args.target_samples),
                        )
                        last_status_draw_at = now
                    window_key = cv2.waitKey(1) & 0xFF
                    if window_key in (ord("q"), 27):
                        print("[handeye] Quit requested while recovering from frame timeout.", flush=True)
                        break
                    term_key = key_reader.poll()
                    if term_key and term_key.lower() == "q":
                        print("[handeye] Quit requested while recovering from frame timeout.", flush=True)
                        break
                    time.sleep(0.2)
                    continue
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
                overlay = draw_manual_overlay(overlay, detection, saved_count, int(args.target_samples))
                last_overlay = overlay.copy()
                last_detection = detection
                cv2.imshow(WINDOW_NAME, overlay)
                fresh_joint_state = bridge.get_joint_state()
                if fresh_joint_state.get("ok"):
                    last_joint_state = fresh_joint_state

                now = time.time()
                if now - last_log_at >= 1.0 or now - last_status_draw_at >= 0.25:
                    render_terminal_status(
                        joint_state=last_joint_state,
                        detection=detection,
                        saved_count=saved_count,
                        target_samples=int(args.target_samples),
                    )
                    last_log_at = now
                    last_status_draw_at = now

                window_key = cv2.waitKey(1) & 0xFF
                term_key = key_reader.poll()
                key = None
                if term_key:
                    key = term_key.lower()
                elif window_key in (ord("s"), ord("q"), 27):
                    key = chr(window_key).lower() if window_key != 27 else "q"

                if key == "q":
                    print("[handeye] Quit requested. Ending manual session.", flush=True)
                    break

                if key == "s":
                    # 保存时重新取一次 joint_state，避免用户按键和上一轮显示之间存在时间差。
                    joint_state = wait_for_joint_state(bridge, timeout_sec=max(args.bridge_wait, 0.0) + 3.0)
                    if joint_state.get("ok"):
                        last_joint_state = joint_state
                    ee_pose = build_ee_pose_summary(args, joint_state)
                    if ee_pose is None:
                        print("[handeye][warning] Save ignored: no valid joint state / ee pose available.", flush=True)
                        continue
                    # 只允许完整 6x6/25 角点样本进入数据集，避免后面 solve 时混入几何约束不足的帧。
                    if not board_is_valid_6x6(detection):
                        print(
                            "[handeye][warning] Save ignored: the full 6x6 board is not detected. "
                            f"reason={board_validity_message(detection)}",
                            flush=True,
                        )
                        continue

                    sample_name = f"sample_{saved_count:03d}"
                    sample_dir = session_root / sample_name
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
                    saved_count += 1
                    print(
                        f"[handeye] Saved {sample_name} | charuco=25/25 | frame={sample['frame_number']} | dir={sample_dir}",
                        flush=True,
                    )
                    if int(args.target_samples) > 0 and saved_count >= int(args.target_samples):
                        print(f"[handeye] Target sample count reached: {saved_count}/{int(args.target_samples)}", flush=True)
                        break
    finally:
        stop_manual_runtime(runtime)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
