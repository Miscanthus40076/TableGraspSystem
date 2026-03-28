from __future__ import annotations

from typing import Any

import cv2
import numpy as np


REQUIRED_CHARUCO_6X6_CORNERS = 25


# Treat only a fully observed 6x6 cut board as "save-ready" for final handeye samples.
def board_is_valid_6x6(detection: dict[str, Any]) -> bool:
    return bool(
        detection.get("pose_ok")
        and detection.get("pose_source") == "charuco"
        and int(detection.get("charuco_corner_count", 0)) == REQUIRED_CHARUCO_6X6_CORNERS
    )


# Return a short machine-readable reason so the session state machine can explain why a frame is unusable.
def board_validity_message(detection: dict[str, Any]) -> str:
    charuco_count = int(detection.get("charuco_corner_count", 0))
    if charuco_count != REQUIRED_CHARUCO_6X6_CORNERS:
        return f"board_not_detected_6x6:charuco_{charuco_count}_of_{REQUIRED_CHARUCO_6X6_CORNERS}"
    if not detection.get("pose_ok"):
        return "board_not_detected_6x6:pose_not_ok"
    if detection.get("pose_source") != "charuco":
        return f"board_not_detected_6x6:pose_source_{detection.get('pose_source')}"
    return "ok"


# Partial board detections can still be used for depth alignment as long as a real detected footprint exists.
def board_has_detected_footprint(detection: dict[str, Any]) -> bool:
    return bool(_flatten_detection_points(detection).shape[0] >= 3)


# Build one consistent "invalid depth result" payload so callers do not need special-case branches.
def build_invalid_board_depth_result(
    detection: dict[str, Any],
    *,
    message: str,
    board_depth_mask_summary: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "board_depth_alignable": False,
        "board_valid_6x6": board_is_valid_6x6(detection),
        "board_valid_message": board_validity_message(detection),
        "board_plane_fit_ok": False,
        "board_depth_mask_summary": board_depth_mask_summary
        or {
            "point_count": 0,
            "hull_area_px": 0.0,
            "bbox_xyxy": None,
            "dilate_px": 0,
            "mask_pixel_count": 0,
            "valid_depth_pixel_count": 0,
        },
        "board_center_xyz_camera_m": None,
        "board_pose_center_xyz_camera_m": detection.get("tvec_m"),
        "board_normal_camera_xyz": None,
        "board_plane_centroid_xyz_camera_m": None,
        "board_plane_inlier_count": 0,
        "normal_alignment_cosine": None,
        "message": message,
    }


# Merge detected ChArUco and marker image points into one board footprint source for masking depth.
def _flatten_detection_points(detection: dict[str, Any]) -> np.ndarray:
    point_groups: list[np.ndarray] = []

    charuco = detection.get("charuco_corners_px") or []
    if charuco:
        point_groups.append(np.asarray(charuco, dtype=np.float32).reshape(-1, 2))

    marker_groups = detection.get("marker_corners_px") or []
    for marker in marker_groups:
        point_groups.append(np.asarray(marker, dtype=np.float32).reshape(-1, 2))

    if not point_groups:
        return np.empty((0, 2), dtype=np.float32)
    return np.vstack(point_groups).astype(np.float32)


# Use only the real detected board footprint to crop depth; never infer a full board outline from pose.
def build_board_mask(
    image_shape: tuple[int, int] | tuple[int, int, int],
    detection: dict[str, Any],
    dilate_px: int = 8,
) -> tuple[np.ndarray, dict[str, Any]]:
    height, width = image_shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    points = _flatten_detection_points(detection)
    summary = {
        "point_count": int(points.shape[0]),
        "hull_area_px": 0.0,
        "bbox_xyxy": None,
        "dilate_px": int(max(dilate_px, 0)),
    }
    if points.shape[0] < 3:
        return mask, summary

    hull = cv2.convexHull(points.reshape(-1, 1, 2)).reshape(-1, 2)
    hull_int = np.round(hull).astype(np.int32)
    cv2.fillConvexPoly(mask, hull_int, 255)
    if dilate_px > 0:
        kernel = np.ones((dilate_px * 2 + 1, dilate_px * 2 + 1), dtype=np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

    xs = hull[:, 0]
    ys = hull[:, 1]
    summary["hull_area_px"] = float(cv2.contourArea(hull.reshape(-1, 1, 2)))
    summary["bbox_xyxy"] = [
        int(np.clip(np.floor(xs.min()), 0, width - 1)),
        int(np.clip(np.floor(ys.min()), 0, height - 1)),
        int(np.clip(np.ceil(xs.max()), 0, width - 1)),
        int(np.clip(np.ceil(ys.max()), 0, height - 1)),
    ]
    return mask, summary


def build_mask_from_points(
    image_shape: tuple[int, int] | tuple[int, int, int],
    points_px: np.ndarray,
    dilate_px: int = 8,
) -> tuple[np.ndarray, dict[str, Any]]:
    height, width = image_shape[:2]
    mask = np.zeros((height, width), dtype=np.uint8)
    points = np.asarray(points_px, dtype=np.float32).reshape(-1, 2)
    summary = {
        "point_count": int(points.shape[0]),
        "hull_area_px": 0.0,
        "bbox_xyxy": None,
        "dilate_px": int(max(dilate_px, 0)),
    }
    if points.shape[0] < 3:
        return mask, summary

    hull = cv2.convexHull(points.reshape(-1, 1, 2)).reshape(-1, 2)
    hull_int = np.round(hull).astype(np.int32)
    cv2.fillConvexPoly(mask, hull_int, 255)
    if dilate_px > 0:
        kernel = np.ones((dilate_px * 2 + 1, dilate_px * 2 + 1), dtype=np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

    xs = hull[:, 0]
    ys = hull[:, 1]
    summary["hull_area_px"] = float(cv2.contourArea(hull.reshape(-1, 1, 2)))
    summary["bbox_xyxy"] = [
        int(np.clip(np.floor(xs.min()), 0, width - 1)),
        int(np.clip(np.floor(ys.min()), 0, height - 1)),
        int(np.clip(np.ceil(xs.max()), 0, width - 1)),
        int(np.clip(np.ceil(ys.max()), 0, height - 1)),
    ]
    return mask, summary


# Reproject only masked depth pixels into camera-frame 3D points measured in meters.
def depth_mask_to_points(
    depth_m: np.ndarray,
    intrinsics: dict[str, Any],
    mask: np.ndarray,
    min_depth_m: float,
    max_depth_m: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    valid = (
        (mask > 0)
        & np.isfinite(depth_m)
        & (depth_m >= float(min_depth_m))
        & (depth_m <= float(max_depth_m))
    )
    ys, xs = np.nonzero(valid)
    z = depth_m[ys, xs].astype(np.float64)

    if z.size == 0:
        return np.empty((0, 3), dtype=np.float64), {
            "mask_pixel_count": int(np.count_nonzero(mask)),
            "valid_depth_pixel_count": 0,
            "point_count": 0,
        }

    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])

    x = (xs.astype(np.float64) - ppx) * z / fx
    y = (ys.astype(np.float64) - ppy) * z / fy
    points = np.column_stack((x, y, z))
    return points, {
        "mask_pixel_count": int(np.count_nonzero(mask)),
        "valid_depth_pixel_count": int(z.size),
        "point_count": int(points.shape[0]),
    }


# Fit a plane robustly enough for board-normal estimation while rejecting obvious board-mask outliers.
def fit_plane_from_points(
    points_xyz_m: np.ndarray,
    distance_threshold_m: float,
    refinement_rounds: int = 3,
) -> dict[str, Any]:
    if points_xyz_m.shape[0] < 3:
        return {
            "ok": False,
            "message": "not_enough_points",
            "point_count": int(points_xyz_m.shape[0]),
        }

    points = np.asarray(points_xyz_m, dtype=np.float64)
    inlier_mask = np.ones(points.shape[0], dtype=bool)
    normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    centroid = points.mean(axis=0)

    for _ in range(max(int(refinement_rounds), 1)):
        working = points[inlier_mask]
        if working.shape[0] < 3:
            break
        centroid = working.mean(axis=0)
        centered = working - centroid
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        normal = vh[-1]
        normal /= max(float(np.linalg.norm(normal)), 1e-12)
        distances = np.abs((points - centroid) @ normal)
        inlier_mask = distances <= float(distance_threshold_m)

    inliers = points[inlier_mask]
    if inliers.shape[0] < 3:
        return {
            "ok": False,
            "message": "too_few_inliers",
            "point_count": int(points.shape[0]),
            "inlier_count": int(inliers.shape[0]),
        }

    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= max(float(np.linalg.norm(normal)), 1e-12)
    distances = np.abs((inliers - centroid) @ normal)

    return {
        "ok": True,
        "point_count": int(points.shape[0]),
        "inlier_count": int(inliers.shape[0]),
        "centroid_xyz_m": centroid.astype(float).tolist(),
        "normal_xyz": normal.astype(float).tolist(),
        "mean_distance_m": float(np.mean(distances)),
        "max_distance_m": float(np.max(distances)),
        "inlier_mask": inlier_mask,
    }


# Convert one aligned color+depth frame into board-center / board-normal geometry for closed-loop alignment.
def analyze_board_depth(
    bundle: dict[str, Any],
    detection: dict[str, Any],
    *,
    min_depth_m: float,
    max_depth_m: float,
    mask_dilate_px: int,
    min_board_depth_points: int,
    plane_distance_threshold_m: float,
) -> dict[str, Any]:
    if not board_has_detected_footprint(detection):
        return build_invalid_board_depth_result(
            detection,
            message="board_footprint_not_detected",
        )

    depth_raw = np.asarray(bundle["depth"], dtype=np.float32)
    depth_m = depth_raw * float(bundle["depth_scale"])
    mask, mask_summary = build_board_mask(bundle["color"].shape, detection, dilate_px=mask_dilate_px)
    points_xyz_m, depth_summary = depth_mask_to_points(
        depth_m,
        bundle["color_intrinsics"],
        mask,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    summary = {**mask_summary, **depth_summary}
    if points_xyz_m.shape[0] < int(min_board_depth_points):
        result = build_invalid_board_depth_result(
            detection,
            message="not_enough_board_depth_points",
            board_depth_mask_summary=summary,
        )
        result["board_depth_alignable"] = False
        return result

    plane = fit_plane_from_points(points_xyz_m, distance_threshold_m=plane_distance_threshold_m)
    if not plane.get("ok"):
        result = build_invalid_board_depth_result(
            detection,
            message=str(plane.get("message", "plane_fit_failed")),
            board_depth_mask_summary=summary,
        )
        result["board_plane_centroid_xyz_camera_m"] = plane.get("centroid_xyz_m")
        result["board_plane_inlier_count"] = int(plane.get("inlier_count", 0))
        return result

    plane_centroid = np.asarray(plane["centroid_xyz_m"], dtype=np.float64).reshape(3)
    pose_center = None
    if detection.get("tvec_m") is not None:
        pose_center = np.asarray(detection["tvec_m"], dtype=np.float64).reshape(3)
    board_center = plane_centroid

    normal = np.asarray(plane["normal_xyz"], dtype=np.float64).reshape(3)
    center_norm = float(np.linalg.norm(board_center))
    if center_norm < 1e-9:
        n_target = np.zeros(3, dtype=np.float64)
        alignment = None
    else:
        n_target = -board_center / center_norm
        if float(np.dot(normal, n_target)) < 0.0:
            normal = -normal
        alignment = float(np.dot(normal, n_target))

    return {
        "board_depth_alignable": True,
        "board_valid_6x6": board_is_valid_6x6(detection),
        "board_valid_message": board_validity_message(detection),
        "board_plane_fit_ok": True,
        "board_depth_mask_summary": summary,
        "board_center_xyz_camera_m": board_center.astype(float).tolist(),
        "board_pose_center_xyz_camera_m": None if pose_center is None else pose_center.astype(float).tolist(),
        "board_normal_camera_xyz": normal.astype(float).tolist(),
        "board_plane_centroid_xyz_camera_m": plane["centroid_xyz_m"],
        "board_plane_inlier_count": int(plane["inlier_count"]),
        "board_plane_mean_distance_m": float(plane["mean_distance_m"]),
        "board_plane_max_distance_m": float(plane["max_distance_m"]),
        "normal_alignment_cosine": alignment,
        "message": "ok",
        "board_mask": mask,
        "board_depth_points_xyz_m": points_xyz_m,
    }


def analyze_board_depth_from_points(
    bundle: dict[str, Any],
    boundary_points_px: np.ndarray,
    *,
    min_depth_m: float,
    max_depth_m: float,
    mask_dilate_px: int,
    min_board_depth_points: int,
    plane_distance_threshold_m: float,
) -> dict[str, Any]:
    depth_raw = np.asarray(bundle["depth"], dtype=np.float32)
    depth_m = depth_raw * float(bundle["depth_scale"])
    mask, mask_summary = build_mask_from_points(bundle["color"].shape, boundary_points_px, dilate_px=mask_dilate_px)
    points_xyz_m, depth_summary = depth_mask_to_points(
        depth_m,
        bundle["color_intrinsics"],
        mask,
        min_depth_m=min_depth_m,
        max_depth_m=max_depth_m,
    )
    summary = {**mask_summary, **depth_summary}
    if points_xyz_m.shape[0] < int(min_board_depth_points):
        return {
            "board_depth_alignable": False,
            "board_plane_fit_ok": False,
            "board_depth_mask_summary": summary,
            "board_center_xyz_camera_m": None,
            "board_normal_camera_xyz": None,
            "board_plane_centroid_xyz_camera_m": None,
            "board_plane_inlier_count": 0,
            "normal_alignment_cosine": None,
            "message": "not_enough_board_depth_points",
            "board_mask": mask,
            "board_depth_points_xyz_m": points_xyz_m,
        }

    plane = fit_plane_from_points(points_xyz_m, distance_threshold_m=plane_distance_threshold_m)
    if not plane.get("ok"):
        return {
            "board_depth_alignable": False,
            "board_plane_fit_ok": False,
            "board_depth_mask_summary": summary,
            "board_center_xyz_camera_m": plane.get("centroid_xyz_m"),
            "board_normal_camera_xyz": None,
            "board_plane_centroid_xyz_camera_m": plane.get("centroid_xyz_m"),
            "board_plane_inlier_count": int(plane.get("inlier_count", 0)),
            "normal_alignment_cosine": None,
            "message": str(plane.get("message", "plane_fit_failed")),
            "board_mask": mask,
            "board_depth_points_xyz_m": points_xyz_m,
        }

    board_center = np.asarray(plane["centroid_xyz_m"], dtype=np.float64).reshape(3)
    normal = np.asarray(plane["normal_xyz"], dtype=np.float64).reshape(3)
    center_norm = float(np.linalg.norm(board_center))
    if center_norm < 1e-9:
        alignment = None
    else:
        n_target = -board_center / center_norm
        if float(np.dot(normal, n_target)) < 0.0:
            normal = -normal
        alignment = float(np.dot(normal, n_target))

    return {
        "board_depth_alignable": True,
        "board_plane_fit_ok": True,
        "board_depth_mask_summary": summary,
        "board_center_xyz_camera_m": board_center.astype(float).tolist(),
        "board_normal_camera_xyz": normal.astype(float).tolist(),
        "board_plane_centroid_xyz_camera_m": plane["centroid_xyz_m"],
        "board_plane_inlier_count": int(plane["inlier_count"]),
        "board_plane_mean_distance_m": float(plane["mean_distance_m"]),
        "board_plane_max_distance_m": float(plane["max_distance_m"]),
        "normal_alignment_cosine": alignment,
        "message": "ok",
        "board_mask": mask,
        "board_depth_points_xyz_m": points_xyz_m,
    }


# Keep the overlay lightweight: show only the depth-derived board region and whether plane fitting is ready.
def draw_board_depth_debug(
    overlay: np.ndarray,
    analysis: dict[str, Any],
    color_hull_bgr: tuple[int, int, int] = (255, 255, 0),
    color_center_bgr: tuple[int, int, int] = (0, 255, 255),
) -> np.ndarray:
    out = overlay.copy()
    bbox = (analysis.get("board_depth_mask_summary") or {}).get("bbox_xyxy")
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(out, (x1, y1), (x2, y2), color_hull_bgr, 2, cv2.LINE_AA)
    center = analysis.get("board_center_xyz_camera_m")
    if center is not None:
        text = "depth-plane=ok" if analysis.get("board_plane_fit_ok") else "depth-plane=wait"
        cv2.putText(out, text, (12, out.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_center_bgr, 2, cv2.LINE_AA)
    return out
