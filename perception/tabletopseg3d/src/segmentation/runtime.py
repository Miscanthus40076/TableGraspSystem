"""Segmentation and 3D lifting helpers for the realtime tabletop pipeline."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Any

import numpy as np

from geometry.pointcloud import (
    filter_points_by_depth_band,
    project_mask_to_points,
    tabletop_aligned_obb,
)


@dataclass
class Detection3D:
    class_name: str
    confidence: float
    bbox_xyxy: list[int]
    mask: np.ndarray
    center_xyz: list[float] | None
    extent_xyz: list[float] | None
    yaw_rad: float | None
    yaw_deg: float | None
    rotation_matrix: np.ndarray | None
    box_corners_xyz: np.ndarray | None
    bbox_min_xyz: list[float] | None
    bbox_max_xyz: list[float] | None
    point_count: int


def load_model(model_name: str):
    from ultralytics import YOLO

    return YOLO(model_name)


def run_inference(model, color_image: np.ndarray, args: argparse.Namespace) -> list[dict[str, Any]]:
    """Run YOLO segmentation and resize masks back to the aligned camera image."""

    import cv2

    results = model.predict(
        source=color_image,
        task="segment",
        device=args.device,
        imgsz=args.imgsz,
        conf=args.conf,
        iou=args.iou,
        max_det=args.max_det,
        verbose=False,
    )
    result = results[0]
    if result.masks is None or result.boxes is None:
        return []

    masks = result.masks.data.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)
    confidences = result.boxes.conf.cpu().numpy().astype(float)
    bboxes = result.boxes.xyxy.cpu().numpy().astype(int)
    image_h, image_w = color_image.shape[:2]

    detections: list[dict[str, Any]] = []
    for idx, mask_arr in enumerate(masks):
        class_id = int(class_ids[idx])
        class_name = result.names.get(class_id, str(class_id))
        if args.target_class and class_name != args.target_class:
            continue

        # Ultralytics masks are produced at inference resolution. Resize them
        # back to the aligned color frame so each mask pixel matches depth.
        mask_resized = cv2.resize(mask_arr, (image_w, image_h), interpolation=cv2.INTER_NEAREST) > 0.5
        detections.append(
            {
                "class_name": class_name,
                "confidence": float(confidences[idx]),
                "bbox_xyxy": [int(v) for v in bboxes[idx].tolist()],
                "mask": mask_resized,
            }
        )
    return detections


def build_detection_3d(
    detection: dict[str, Any],
    depth_m: np.ndarray,
    intrinsics: dict[str, Any],
    table_normal: np.ndarray,
    args: argparse.Namespace,
) -> Detection3D:
    """Lift one segmentation mask into a tabletop-aligned 3D oriented box."""

    raw_points, _ = project_mask_to_points(
        mask=detection["mask"],
        depth_m=depth_m,
        intrinsics=intrinsics,
        min_depth_m=args.min_depth,
        max_depth_m=args.max_depth,
    )

    # Mask boundaries often include a few unstable depth pixels. A lightweight
    # depth-band filter removes most of that noise before fitting the box.
    filtered_points = filter_points_by_depth_band(raw_points)
    point_count = int(len(filtered_points))
    if point_count < args.min_points:
        return Detection3D(
            class_name=detection["class_name"],
            confidence=detection["confidence"],
            bbox_xyxy=detection["bbox_xyxy"],
            mask=detection["mask"],
            center_xyz=None,
            extent_xyz=None,
            yaw_rad=None,
            yaw_deg=None,
            rotation_matrix=None,
            box_corners_xyz=None,
            bbox_min_xyz=None,
            bbox_max_xyz=None,
            point_count=point_count,
        )

    obb = tabletop_aligned_obb(filtered_points, plane_normal=table_normal)
    return Detection3D(
        class_name=detection["class_name"],
        confidence=detection["confidence"],
        bbox_xyxy=detection["bbox_xyxy"],
        mask=detection["mask"],
        center_xyz=obb["center_xyz"].tolist(),
        extent_xyz=obb["extent_xyz"].tolist(),
        yaw_rad=obb["yaw_rad"],
        yaw_deg=obb["yaw_deg"],
        rotation_matrix=obb["rotation_matrix"],
        box_corners_xyz=obb["corners_xyz"],
        bbox_min_xyz=obb["bbox_min_xyz"].tolist(),
        bbox_max_xyz=obb["bbox_max_xyz"].tolist(),
        point_count=point_count,
    )


def build_scene_point_cloud(
    color_image: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray]:
    """Back-project the full scene into a subsampled colored point cloud."""

    height, width = depth_m.shape
    stride = max(1, int(args.point_stride))
    ys = np.arange(0, height, stride, dtype=np.int32)
    xs = np.arange(0, width, stride, dtype=np.int32)
    grid_x, grid_y = np.meshgrid(xs, ys)

    sampled_depth = depth_m[grid_y, grid_x]
    valid = np.isfinite(sampled_depth) & (sampled_depth > args.min_depth) & (sampled_depth < args.max_depth)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float64)

    z = sampled_depth[valid].astype(np.float32)
    u = grid_x[valid].astype(np.float32)
    v = grid_y[valid].astype(np.float32)
    fx = float(intrinsics["fx"])
    fy = float(intrinsics["fy"])
    ppx = float(intrinsics["ppx"])
    ppy = float(intrinsics["ppy"])

    x = (u - ppx) * z / fx
    y = (v - ppy) * z / fy
    points = np.stack([x, y, z], axis=1)
    colors = color_image[grid_y[valid], grid_x[valid]][:, ::-1].astype(np.float64) / 255.0

    max_points = int(args.scene_max_points)
    if max_points > 0 and len(points) > max_points:
        # Even spacing keeps the visualization stable without allocating a full
        # random shuffle on every frame.
        keep = np.linspace(0, len(points) - 1, max_points, dtype=np.int32)
        points = points[keep]
        colors = colors[keep]

    return points, colors


def highlight_object_points(
    scene_points: np.ndarray,
    scene_colors: np.ndarray,
    detections_3d: list[Detection3D],
) -> np.ndarray:
    """Blend object colors into the scene cloud using fitted 3D boxes."""

    if len(scene_points) == 0:
        return scene_colors

    colors = scene_colors.copy()
    for idx, detection in enumerate(detections_3d):
        if detection.center_xyz is None or detection.extent_xyz is None or detection.rotation_matrix is None:
            continue

        center = np.asarray(detection.center_xyz, dtype=np.float32)
        half_extent = 0.5 * np.asarray(detection.extent_xyz, dtype=np.float32)
        local = (scene_points.astype(np.float32) - center[None, :]) @ detection.rotation_matrix
        inside = np.all(np.abs(local) <= (half_extent[None, :] + 1e-4), axis=1)
        if np.any(inside):
            colors[inside] = 0.55 * colors[inside] + 0.45 * color_for_index(idx)
    return colors


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


def estimate_table_normal(scene_points: np.ndarray, o3d: Any) -> np.ndarray:
    """Estimate a dominant tabletop normal from the full-scene point cloud."""

    default_normal = np.array([0.0, -1.0, 0.0], dtype=np.float32)
    if len(scene_points) < 128:
        return default_normal

    sampled_points = scene_points
    max_plane_points = 12000
    if len(sampled_points) > max_plane_points:
        keep = np.linspace(0, len(sampled_points) - 1, max_plane_points, dtype=np.int32)
        sampled_points = sampled_points[keep]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(sampled_points.astype(np.float64))
    plane_model, _ = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=120)
    normal = np.asarray(plane_model[:3], dtype=np.float32)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-6:
        return default_normal

    normal = normal / norm
    if float(np.dot(normal, default_normal)) < 0.0:
        normal = -normal
    return normal.astype(np.float32)


def frame_output_record(
    frame_index: int,
    fps_value: float,
    infer_ms: float,
    geom_ms: float,
    scene_points: np.ndarray,
    table_normal: np.ndarray,
    detections_3d: list[Detection3D],
) -> dict[str, Any]:
    return {
        "frame_index": frame_index,
        "fps": round(float(fps_value), 4),
        "infer_ms": round(float(infer_ms), 4),
        "geom_ms": round(float(geom_ms), 4),
        "scene_point_count": int(len(scene_points)),
        "table_normal_xyz": [round(float(v), 6) for v in table_normal.tolist()],
        "detections": [
            {
                "class_name": det.class_name,
                "confidence": round(float(det.confidence), 6),
                "center_camera_xyz_m": None if det.center_xyz is None else [round(float(v), 6) for v in det.center_xyz],
                "extent_xyz_m": None if det.extent_xyz is None else [round(float(v), 6) for v in det.extent_xyz],
                "yaw_rad": None if det.yaw_rad is None else round(float(det.yaw_rad), 6),
                "yaw_deg": None if det.yaw_deg is None else round(float(det.yaw_deg), 4),
                "point_count": int(det.point_count),
            }
            for det in detections_3d
            if det.center_xyz is not None
        ],
    }
