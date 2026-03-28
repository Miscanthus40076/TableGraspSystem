"""Helpers that keep the Open3D rendering code small and readable."""

from __future__ import annotations

from typing import Any

import numpy as np

from segmentation.runtime import Detection3D

BOX_LINES = np.array(
    [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ],
    dtype=np.int32,
)

BACKGROUND_COLOR_RGB = np.array([0.12, 0.14, 0.16], dtype=np.float64)
BACKGROUND_COLOR_RGBA = np.array([0.12, 0.14, 0.16, 1.0], dtype=np.float32)


def color_for_index(index: int) -> np.ndarray:
    palette = np.array(
        [
            [1.0, 0.35, 0.35],
            [0.35, 1.0, 0.55],
            [0.35, 0.7, 1.0],
            [1.0, 0.82, 0.35],
            [0.82, 0.35, 1.0],
            [0.35, 1.0, 1.0],
        ],
        dtype=np.float64,
    )
    return palette[index % len(palette)]


def update_line_set(line_set: Any, corners_xyz: np.ndarray | None, color: np.ndarray, o3d: Any) -> None:
    if corners_xyz is None or len(corners_xyz) != 8:
        line_set.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        line_set.lines = o3d.utility.Vector2iVector(np.empty((0, 2), dtype=np.int32))
        line_set.colors = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        return

    line_set.points = o3d.utility.Vector3dVector(np.asarray(corners_xyz, dtype=np.float64))
    line_set.lines = o3d.utility.Vector2iVector(BOX_LINES)
    line_set.colors = o3d.utility.Vector3dVector(np.tile(color[None, :], (len(BOX_LINES), 1)))


def scene_center(points_xyz: np.ndarray) -> np.ndarray:
    if len(points_xyz) == 0:
        return np.array([0.0, 0.0, 0.5], dtype=np.float64)
    return points_xyz.mean(axis=0).astype(np.float64)


def configure_view(vis: Any, center_xyz: np.ndarray) -> None:
    view = vis.get_view_control()
    view.set_lookat(center_xyz.tolist())
    view.set_front([0.0, 0.0, -1.0])
    view.set_up([0.0, -1.0, 0.0])
    view.set_zoom(0.7)


def scene_extent(points_xyz: np.ndarray) -> float:
    if len(points_xyz) == 0:
        return 1.0
    mins = points_xyz.min(axis=0)
    maxs = points_xyz.max(axis=0)
    return float(max(np.linalg.norm(maxs - mins), 0.5))


def scene_eye(points_xyz: np.ndarray, center_xyz: np.ndarray) -> np.ndarray:
    distance = scene_extent(points_xyz) * 1.2
    return center_xyz + np.array([0.0, 0.0, -distance], dtype=np.float32)


def label_anchor(detection: Detection3D) -> np.ndarray:
    if detection.box_corners_xyz is not None:
        corners = np.asarray(detection.box_corners_xyz, dtype=np.float32)
        return corners[corners[:, 1].argmin()]
    if detection.center_xyz is not None:
        return np.asarray(detection.center_xyz, dtype=np.float32)
    return np.zeros(3, dtype=np.float32)


def format_detection_label(detection: Detection3D) -> str:
    if detection.center_xyz is None:
        return (
            f"{detection.class_name} {detection.confidence:.2f}\n"
            f"pts={detection.point_count}"
        )

    cx, cy, cz = detection.center_xyz
    yaw_text = "n/a" if detection.yaw_deg is None else f"{detection.yaw_deg:.1f} deg"
    return (
        f"{detection.class_name} {detection.confidence:.2f}\n"
        f"xyz=({cx:.3f}, {cy:.3f}, {cz:.3f}) m\n"
        f"yaw={yaw_text}\n"
        f"pts={detection.point_count}"
    )


def build_legacy_point_cloud(o3d: Any, scene_points: np.ndarray, scene_colors: np.ndarray) -> Any:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(scene_points.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(scene_colors.astype(np.float64))
    return pcd


def update_labels(vis: Any, detections_3d: list[Detection3D]) -> None:
    vis.clear_3d_labels()
    for detection in detections_3d:
        if detection.center_xyz is None:
            continue
        vis.add_3d_label(label_anchor(detection), format_detection_label(detection))
