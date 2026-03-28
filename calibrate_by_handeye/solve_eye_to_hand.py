#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Solve eye-to-hand calibration from board-on-hand samples.")
    parser.add_argument("--samples-dir", type=Path, default=PROJECT_ROOT / "calibrate_by_handeye" / "samples")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "calibrate_by_handeye" / "extrinsics_eye_to_hand.json")
    parser.add_argument(
        "--method",
        type=str,
        default="auto",
        choices=["auto", "tsai", "park", "horaud", "andreff", "daniilidis"],
    )
    return parser.parse_args()


def resolve_samples_dir(samples_dir: Path) -> Path:
    """把输入解析成真正要用于求解的 session 目录。

    约定上：
    - 直接传入 session_xxx 就按该目录求解
    - 传入 samples/ 总目录时，默认选修改时间最新的 session
    """
    samples_dir = samples_dir.resolve()
    if samples_dir.name.startswith("session_"):
        return samples_dir
    sessions = sorted(
        [path for path in samples_dir.iterdir() if path.is_dir() and path.name.startswith("session_")],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    if not sessions:
        return samples_dir
    return sessions[0]


# Invert a homogeneous transform so OpenCV receives the base->gripper convention it expects.
def inverse_transform(tf: np.ndarray) -> np.ndarray:
    rot = tf[:3, :3]
    trans = tf[:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T @ trans
    return out


# Map the user-facing method name onto OpenCV's calibrateHandEye enum.
def method_to_cv(name: str) -> int:
    return {
        "tsai": cv2.CALIB_HAND_EYE_TSAI,
        "park": cv2.CALIB_HAND_EYE_PARK,
        "horaud": cv2.CALIB_HAND_EYE_HORAUD,
        "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
        "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }[name]


def method_order(requested: str) -> list[str]:
    if requested != "auto":
        return [requested]
    # Daniilidis tends to be the most forgiving when rotations are not very informative.
    return ["daniilidis", "horaud", "park", "tsai", "andreff"]


# Serialize the solved camera rotation into the quaternion form used elsewhere in the project.
def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> list[float]:
    m = rotation
    trace = float(np.trace(m))
    if trace > 0.0:
        s = (trace + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    else:
        diag = np.diag(m)
        idx = int(np.argmax(diag))
        if idx == 0:
            s = (1.0 + m[0, 0] - m[1, 1] - m[2, 2]) ** 0.5 * 2.0
            qw = (m[2, 1] - m[1, 2]) / s
            qx = 0.25 * s
            qy = (m[0, 1] + m[1, 0]) / s
            qz = (m[0, 2] + m[2, 0]) / s
        elif idx == 1:
            s = (1.0 + m[1, 1] - m[0, 0] - m[2, 2]) ** 0.5 * 2.0
            qw = (m[0, 2] - m[2, 0]) / s
            qx = (m[0, 1] + m[1, 0]) / s
            qy = 0.25 * s
            qz = (m[1, 2] + m[2, 1]) / s
        else:
            s = (1.0 + m[2, 2] - m[0, 0] - m[1, 1]) ** 0.5 * 2.0
            qw = (m[1, 0] - m[0, 1]) / s
            qx = (m[0, 2] + m[2, 0]) / s
            qy = (m[1, 2] + m[2, 1]) / s
            qz = 0.25 * s
    return [float(qx), float(qy), float(qz), float(qw)]


def validate_solution(rotation: np.ndarray, translation: np.ndarray) -> tuple[bool, str]:
    """做最低限度的几何合法性检查。

    这里不判断“是不是最准”，只排除明显坏解：
    - 旋转矩阵非正交
    - det(R) 明显不等于 1
    - 平移大到离谱
    """
    if rotation is None or translation is None:
        return False, "missing_solution"
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        return False, "non_finite"
    det = float(np.linalg.det(rotation))
    if abs(det - 1.0) > 1e-2:
        return False, f"bad_rotation_det:{det:.6f}"
    ortho_err = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord="fro"))
    if ortho_err > 1e-1:
        return False, f"bad_rotation_orthogonality:{ortho_err:.6f}"
    tnorm = float(np.linalg.norm(np.asarray(translation, dtype=np.float64).reshape(3)))
    if tnorm > 5.0:
        return False, f"translation_too_large:{tnorm:.6f}"
    return True, "ok"


# Load all valid board-on-hand samples and solve one fixed camera-to-base transform for eye-to-hand calibration.
def main() -> None:
    args = parse_args()
    resolved_samples_dir = resolve_samples_dir(args.samples_dir)
    sample_paths = sorted(resolved_samples_dir.rglob("sample.json"))
    if len(sample_paths) < 3:
        raise RuntimeError("Need at least 3 samples to solve eye-to-hand calibration.")

    # 当前 eye-to-hand 假设：
    # - 标定板固定在末端
    # - 相机固定在环境中
    #
    # 这里喂给 OpenCV 的方向约定非常关键。虽然 calibrateHandEye 文档常用
    # gripper2base / target2cam 来描述，但在本项目里我们最终想要的是
    # camera->base 的外参，因此实际传入的是：
    # - base->gripper：通过 inv(T_base_ee) 得到
    # - cam->target：直接使用 solvePnP / detect_charuco 的 board->camera 结果
    #
    # 这一套喂法与 validate_eye_to_hand.py 中
    # inv(T_base_ee) @ T_base_camera @ T_camera_board
    # 的约定保持一致，随意改方向会让 validate 直接失去意义。
    r_base2gripper = []
    t_base2gripper = []
    r_cam2target = []
    t_cam2target = []
    used = []

    for path in sample_paths:
        sample = json.loads(path.read_text(encoding="utf-8"))
        det = sample.get("board_detection", {})
        if not det.get("pose_ok"):
            continue
        # 保存样本里记录的是 T_base_ee，这里先求逆得到 OpenCV 需要的 base->gripper。
        t_base_ee = np.asarray(sample["ee_reference"]["T_base_ee"], dtype=np.float64)
        t_ee_base = inverse_transform(t_base_ee)
        r_base2gripper.append(t_ee_base[:3, :3])
        t_base2gripper.append(t_ee_base[:3, 3].reshape(3, 1))

        rvec = np.asarray(det["rvec"], dtype=np.float64).reshape(3, 1)
        rot_cam_target, _ = cv2.Rodrigues(rvec)
        tvec = np.asarray(det["tvec_m"], dtype=np.float64).reshape(3, 1)
        # board_detection 内的 rvec/tvec 在本项目里统一按 board->camera 使用。
        r_cam2target.append(rot_cam_target)
        t_cam2target.append(tvec)
        used.append(sample.get("sample_name", path.parent.name))

    if len(used) < 3:
        raise RuntimeError("Need at least 3 valid pose-detected samples.")

    attempted: list[dict[str, object]] = []
    rot_cam2base = None
    trans_cam2base = None
    chosen_method = None
    last_error = None
    for method_name in method_order(args.method):
        try:
            candidate_rot, candidate_trans = cv2.calibrateHandEye(
                r_base2gripper,
                t_base2gripper,
                r_cam2target,
                t_cam2target,
                method=method_to_cv(method_name),
            )
            ok, reason = validate_solution(candidate_rot, candidate_trans)
            attempted.append(
                {
                    "method": method_name,
                    "ok": ok,
                    "reason": reason,
                    "translation_xyz_m": None if candidate_trans is None else np.asarray(candidate_trans).reshape(3).tolist(),
                }
            )
            if ok:
                rot_cam2base = candidate_rot
                trans_cam2base = candidate_trans
                chosen_method = method_name
                break
        except cv2.error as exc:
            last_error = exc
            attempted.append({"method": method_name, "ok": False, "reason": f"opencv_error:{exc}"})

    if rot_cam2base is None or trans_cam2base is None or chosen_method is None:
        raise RuntimeError(
            "No valid hand-eye solution found. Attempts: "
            + json.dumps(attempted, ensure_ascii=True)
            + ("" if last_error is None else f" | last_error={last_error}")
        )

    t_base_camera = np.eye(4, dtype=np.float64)
    t_base_camera[:3, :3] = rot_cam2base
    t_base_camera[:3, 3] = trans_cam2base.reshape(3)
    # 输出文件中的 T_base_camera 语义是“camera 坐标到 base 坐标”。

    result = {
        "sample_count": len(used),
        "samples_dir": str(resolved_samples_dir),
        "method": chosen_method,
        "requested_method": args.method,
        "attempts": attempted,
        "samples": used,
        "T_base_camera": t_base_camera.tolist(),
        "rotation_matrix": rot_cam2base.tolist(),
        "translation_xyz_m": trans_cam2base.reshape(3).tolist(),
        "quaternion_xyzw": rotation_to_quaternion_xyzw(rot_cam2base),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=True, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
