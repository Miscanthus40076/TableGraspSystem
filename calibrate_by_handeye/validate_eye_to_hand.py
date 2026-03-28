#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate an eye-to-hand result by checking whether the inferred ee->board mount stays consistent across samples."
    )
    parser.add_argument(
        "--samples-dir",
        type=Path,
        default=PROJECT_ROOT / "calibrate_by_handeye" / "samples" / "session_20260327_142345",
    )
    parser.add_argument(
        "--extrinsics",
        type=Path,
        default=PROJECT_ROOT / "calibrate_by_handeye" / "extrinsics_eye_to_hand.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=PROJECT_ROOT / "calibrate_by_handeye" / "validation_eye_to_hand.json",
    )
    return parser.parse_args()


def inverse_transform(tf: np.ndarray) -> np.ndarray:
    """对齐 solve/validate 两侧的齐次变换求逆逻辑。"""
    rot = tf[:3, :3]
    trans = tf[:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ trans
    return out


def rvec_tvec_to_transform(rvec: list[float], tvec: list[float]) -> np.ndarray:
    """把 board_detection 里的 rvec/tvec 还原为 T_camera_board。"""
    rot, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    tf = np.eye(4, dtype=np.float64)
    tf[:3, :3] = rot
    tf[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return tf


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    m = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:
        diag = np.diag(m)
        idx = int(np.argmax(diag))
        if idx == 0:
            s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-12)
    return quat


def quaternion_to_rotation(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_xyzw.tolist()
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def average_quaternions_xyzw(quaternions: list[np.ndarray]) -> np.ndarray:
    if not quaternions:
        raise RuntimeError("Cannot average zero quaternions.")
    ref = quaternions[0]
    aligned = []
    for quat in quaternions:
        q = quat.copy()
        if float(np.dot(q, ref)) < 0.0:
            q = -q
        aligned.append(q)
    avg = np.mean(np.vstack(aligned), axis=0)
    avg /= max(float(np.linalg.norm(avg)), 1e-12)
    return avg


def rotation_angle_deg(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    rel = rot_a.T @ rot_b
    trace = float(np.trace(rel))
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    return math.degrees(math.acos(cos_theta))


def load_valid_mount_samples(samples_dir: Path) -> list[dict[str, object]]:
    """只加载“板完整识别”的样本。

    validate 的意义是检查外参是否自洽，因此这里故意过滤掉：
    - pose_ok 为 false
    - 不是完整 25/25 ChArUco 的样本
    避免用低质量样本把 mount 一致性统计污染掉。
    """
    out: list[dict[str, object]] = []
    for path in sorted(samples_dir.rglob("sample.json")):
        sample = json.loads(path.read_text(encoding="utf-8"))
        det = sample.get("board_detection", {})
        if not det.get("pose_ok"):
            continue
        if int(det.get("charuco_corner_count", 0)) != 25:
            continue
        t_base_ee = np.asarray(sample["ee_reference"]["T_base_ee"], dtype=np.float64)
        # solvePnP / detect_charuco returns board->camera directly:
        # p_camera = T_camera_board * p_board
        t_camera_board = rvec_tvec_to_transform(det["rvec"], det["tvec_m"])
        out.append(
            {
                "sample_name": str(sample.get("sample_name", path.parent.name)),
                "T_base_ee": t_base_ee,
                "T_camera_board": t_camera_board,
            }
        )
    return out


def main() -> None:
    """验证当前 T_base_camera 是否能让 T_ee_board 在整批样本里近似恒定。"""
    args = parse_args()
    extrinsics = json.loads(args.extrinsics.read_text(encoding="utf-8"))
    t_base_camera = np.asarray(extrinsics["T_base_camera"], dtype=np.float64)
    valid_samples = load_valid_mount_samples(args.samples_dir)
    if len(valid_samples) < 3:
        raise RuntimeError("Need at least 3 valid 25/25 samples to validate eye-to-hand extrinsics.")

    mount_transforms = []
    quats = []
    for item in valid_samples:
        t_ee_base = inverse_transform(item["T_base_ee"])
        # 这里的核心检查就是：
        # T_ee_board = inv(T_base_ee) @ T_base_camera @ T_camera_board
        # 如果外参和样本坐标约定都正确，板相对末端的安装关系应在所有样本中近似不变。
        t_ee_board = t_ee_base @ t_base_camera @ item["T_camera_board"]
        mount_transforms.append((item["sample_name"], t_ee_board))
        quats.append(rotation_to_quaternion_xyzw(t_ee_board[:3, :3]))

    mean_translation = np.mean([tf[:3, 3] for _, tf in mount_transforms], axis=0)
    mean_quat = average_quaternions_xyzw(quats)
    mean_rotation = quaternion_to_rotation(mean_quat)

    per_sample = []
    translation_errors = []
    rotation_errors = []
    for sample_name, tf in mount_transforms:
        trans = tf[:3, 3]
        rot = tf[:3, :3]
        trans_err = float(np.linalg.norm(trans - mean_translation))
        rot_err = rotation_angle_deg(mean_rotation, rot)
        translation_errors.append(trans_err)
        rotation_errors.append(rot_err)
        per_sample.append(
            {
                "sample_name": sample_name,
                "T_ee_board": tf.tolist(),
                "translation_xyz_m": trans.tolist(),
                "quaternion_xyzw": rotation_to_quaternion_xyzw(rot).tolist(),
                "translation_error_m": trans_err,
                "rotation_error_deg": rot_err,
            }
        )

    result = {
        "sample_count": len(valid_samples),
        "extrinsics_file": str(args.extrinsics),
        "samples_dir": str(args.samples_dir),
        "mean_mount_translation_xyz_m": mean_translation.tolist(),
        "mean_mount_quaternion_xyzw": mean_quat.tolist(),
        "mean_mount_rotation_matrix": mean_rotation.tolist(),
        "translation_error_stats_m": {
            "mean": float(np.mean(translation_errors)),
            "median": float(np.median(translation_errors)),
            "max": float(np.max(translation_errors)),
        },
        "rotation_error_stats_deg": {
            "mean": float(np.mean(rotation_errors)),
            "median": float(np.median(rotation_errors)),
            "max": float(np.max(rotation_errors)),
        },
        "per_sample": per_sample,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=True, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
