#!/usr/bin/env python3
"""Realtime Open3D scene viewer with YOLO segmentation and 3D boxes."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    # Allow running this file directly without installing the package first.
    sys.path.insert(0, str(SRC_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Realtime Open3D point-cloud scene viewer.")
    parser.add_argument("--list-devices", action="store_true", help="List connected RealSense devices and exit.")
    parser.add_argument("--serial", type=str, default="419522072950", help="RealSense serial to use.")
    parser.add_argument("--model", type=str, default="yolo11n-seg.pt", help="Ultralytics segmentation model.")
    parser.add_argument("--device", type=str, default="cpu", help="Inference device.")
    parser.add_argument("--width", type=int, default=640, help="Camera stream width.")
    parser.add_argument("--height", type=int, default=480, help="Camera stream height.")
    parser.add_argument("--fps", type=int, default=30, help="Camera stream FPS.")
    parser.add_argument("--imgsz", type=int, default=448, help="Inference image size.")
    parser.add_argument("--conf", type=float, default=0.25, help="Detection confidence threshold.")
    parser.add_argument("--iou", type=float, default=0.45, help="NMS IoU threshold.")
    parser.add_argument("--max-det", type=int, default=10, help="Maximum detections per frame.")
    parser.add_argument("--target-class", type=str, default="", help="Optional class filter.")
    parser.add_argument("--min-depth", type=float, default=0.10, help="Minimum valid depth in meters.")
    parser.add_argument("--max-depth", type=float, default=1.50, help="Maximum valid depth in meters.")
    parser.add_argument("--min-points", type=int, default=500, help="Minimum object point count for a 3D box.")
    parser.add_argument("--warmup-frames", type=int, default=5, help="Camera warm-up frames.")
    parser.add_argument("--frames", type=int, default=0, help="Run a fixed number of frames then exit.")
    parser.add_argument("--point-stride", type=int, default=2, help="Subsample stride for full-scene point cloud.")
    parser.add_argument("--scene-max-points", type=int, default=80000, help="Maximum full-scene points to keep after stride.")
    parser.add_argument("--show-object-points", action="store_true", help="Also color object mask points in the full-scene point cloud.")
    parser.add_argument("--show-labels", action="store_true", help="Show 3D labels for detected objects in the Open3D scene.")
    parser.add_argument("--no-display", action="store_true", help="Disable Open3D window and print timing only.")
    return parser.parse_args()


def print_connected_devices(devices: list[Any]) -> None:
    if not devices:
        print("No RealSense devices found.")
        return

    print("Connected RealSense devices:")
    for device in devices:
        print(f"- {device.name} | serial={device.serial_number}")


def build_initial_state(model, warm_bundle: dict[str, Any], table_normal: np.ndarray, args: argparse.Namespace):
    """Prepare the first scene update so window creation and headless mode share one path."""

    from segmentation.runtime import (  # noqa: E402
        build_detection_3d,
        build_scene_point_cloud,
        highlight_object_points,
        run_inference,
    )

    initial_depth_m = warm_bundle["depth"].astype(np.float32) * float(warm_bundle["depth_scale"])
    initial_scene_points, initial_scene_colors = build_scene_point_cloud(
        color_image=warm_bundle["color"],
        depth_m=initial_depth_m,
        intrinsics=warm_bundle["color_intrinsics"],
        args=args,
    )
    initial_detections = run_inference(model, warm_bundle["color"], args)
    initial_detections_3d = [
        build_detection_3d(det, initial_depth_m, warm_bundle["color_intrinsics"], table_normal, args)
        for det in initial_detections
    ]
    if args.show_object_points:
        initial_scene_colors = highlight_object_points(initial_scene_points, initial_scene_colors, initial_detections_3d)
    return initial_scene_points, initial_scene_colors, initial_detections_3d


def main() -> int:
    args = parse_args()
    import open3d as o3d
    from camera.realsense_capture import (
        build_runtime,
        enumerate_devices,
        get_aligned_frame_bundle,
        select_serials,
        stop_runtimes,
    )
    from segmentation.runtime import (
        build_detection_3d,
        build_scene_point_cloud,
        estimate_table_normal,
        frame_output_record,
        highlight_object_points,
        load_model,
        run_inference,
    )
    from visualization.open3d_scene import (
        BACKGROUND_COLOR_RGB,
        BACKGROUND_COLOR_RGBA,
        build_legacy_point_cloud,
        color_for_index,
        configure_view,
        scene_center,
        scene_eye,
        update_labels,
        update_line_set,
    )

    devices = enumerate_devices()
    if args.list_devices:
        print_connected_devices(devices)
        return 0

    serial = select_serials(devices, [args.serial], expected_count=1)[0]
    device_map = {device.serial_number: device for device in devices}

    model = load_model(args.model)
    runtime = build_runtime(device_map[serial], args.width, args.height, args.fps)
    frame_times: list[float] = []
    frame_counter = 0

    vis = None
    gui_app = None
    scene_pcd = o3d.geometry.PointCloud()
    scene_material = None
    box_sets: list[Any] = [o3d.geometry.LineSet() for _ in range(args.max_det)]
    box_added = [False for _ in range(args.max_det)]
    center_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
    label_mode = bool(args.show_labels and not args.no_display)
    window_closed = False
    table_normal = np.array([0.0, -1.0, 0.0], dtype=np.float32)

    try:
        for _ in range(args.warmup_frames):
            get_aligned_frame_bundle(runtime, args.min_depth, args.max_depth)

        warm_bundle = get_aligned_frame_bundle(runtime, args.min_depth, args.max_depth)

        # Prime model weights and kernels before the measured realtime loop starts.
        model.predict(
            source=warm_bundle["color"],
            task="segment",
            device=args.device,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            max_det=args.max_det,
            verbose=False,
        )

        warm_depth_m = warm_bundle["depth"].astype(np.float32) * float(warm_bundle["depth_scale"])
        warm_scene_points, _ = build_scene_point_cloud(
            color_image=warm_bundle["color"],
            depth_m=warm_depth_m,
            intrinsics=warm_bundle["color_intrinsics"],
            args=args,
        )
        table_normal = estimate_table_normal(warm_scene_points, o3d)
        initial_scene_points, initial_scene_colors, initial_detections_3d = build_initial_state(
            model,
            warm_bundle,
            table_normal,
            args,
        )

        if not args.no_display:
            if label_mode:
                import open3d.visualization.gui as gui
                import open3d.visualization.rendering as rendering

                gui_app = gui.Application.instance
                gui_app.initialize()
                vis = o3d.visualization.O3DVisualizer("Realtime Open3D Scene", 1280, 800)
                vis.show_axes = True
                vis.show_settings = False
                vis.show_ground = False
                vis.show_skybox(False)
                vis.set_background(BACKGROUND_COLOR_RGBA, None)

                def on_window_close() -> bool:
                    nonlocal window_closed
                    window_closed = True
                    return True

                vis.set_on_close(on_window_close)

                scene_material = rendering.MaterialRecord()
                scene_material.shader = "defaultUnlit"
                scene_material.point_size = 2.0

                vis.add_geometry(
                    "scene",
                    build_legacy_point_cloud(o3d, initial_scene_points, initial_scene_colors),
                    scene_material,
                )

                for idx, detection in enumerate(initial_detections_3d):
                    if idx >= args.max_det or detection.box_corners_xyz is None:
                        continue
                    update_line_set(box_sets[idx], detection.box_corners_xyz, color_for_index(idx), o3d)
                    vis.add_geometry(f"box_{idx}", box_sets[idx])
                    box_added[idx] = True

                update_labels(vis, initial_detections_3d)
                gui_app.add_window(vis)
                center = scene_center(initial_scene_points).astype(np.float32)
                vis.setup_camera(
                    45.0,
                    center,
                    scene_eye(initial_scene_points, center).astype(np.float32),
                    np.array([0.0, -1.0, 0.0], dtype=np.float32),
                )
                gui_app.run_one_tick()
            else:
                vis = o3d.visualization.Visualizer()
                vis.create_window(window_name="Realtime Open3D Scene", width=1280, height=800)
                scene_pcd.points = o3d.utility.Vector3dVector(initial_scene_points.astype(np.float64))
                scene_pcd.colors = o3d.utility.Vector3dVector(initial_scene_colors.astype(np.float64))
                vis.add_geometry(scene_pcd)

                first_center = None
                for idx, detection in enumerate(initial_detections_3d):
                    if idx >= len(box_sets):
                        break
                    if detection.box_corners_xyz is None:
                        continue
                    update_line_set(box_sets[idx], detection.box_corners_xyz, color_for_index(idx), o3d)
                    vis.add_geometry(box_sets[idx], reset_bounding_box=False)
                    box_added[idx] = True
                    if first_center is None and detection.center_xyz is not None:
                        first_center = np.array(detection.center_xyz, dtype=np.float64)

                if first_center is not None:
                    center_frame.translate(first_center, relative=True)
                vis.add_geometry(center_frame)
                render_option = vis.get_render_option()
                render_option.background_color = BACKGROUND_COLOR_RGB
                render_option.point_size = 2.0
                configure_view(vis, scene_center(initial_scene_points))

        while True:
            loop_start = time.perf_counter()
            bundle = get_aligned_frame_bundle(runtime, args.min_depth, args.max_depth)
            depth_m = bundle["depth"].astype(np.float32) * float(bundle["depth_scale"])

            infer_start = time.perf_counter()
            detections = run_inference(model, bundle["color"], args)
            infer_ms = (time.perf_counter() - infer_start) * 1000.0

            geom_start = time.perf_counter()
            detections_3d = [
                build_detection_3d(det, depth_m, bundle["color_intrinsics"], table_normal, args)
                for det in detections
            ]
            scene_points, scene_colors = build_scene_point_cloud(
                color_image=bundle["color"],
                depth_m=depth_m,
                intrinsics=bundle["color_intrinsics"],
                args=args,
            )
            if args.show_object_points:
                scene_colors = highlight_object_points(scene_points, scene_colors, detections_3d)
            geom_ms = (time.perf_counter() - geom_start) * 1000.0

            loop_time = time.perf_counter() - loop_start
            frame_times.append(loop_time)
            if len(frame_times) > 30:
                frame_times.pop(0)
            fps_value = len(frame_times) / sum(frame_times)

            if args.no_display:
                print(
                    json.dumps(
                        frame_output_record(
                            frame_index=frame_counter,
                            fps_value=fps_value,
                            infer_ms=infer_ms,
                            geom_ms=geom_ms,
                            scene_points=scene_points,
                            table_normal=table_normal,
                            detections_3d=detections_3d,
                        ),
                        ensure_ascii=False,
                    )
                )
            else:
                if label_mode:
                    vis.remove_geometry("scene")
                    vis.add_geometry(
                        "scene",
                        build_legacy_point_cloud(o3d, scene_points, scene_colors),
                        scene_material,
                    )

                    for idx in range(args.max_det):
                        box_name = f"box_{idx}"
                        if idx < len(detections_3d) and detections_3d[idx].box_corners_xyz is not None:
                            update_line_set(
                                box_sets[idx],
                                detections_3d[idx].box_corners_xyz,
                                color_for_index(idx),
                                o3d,
                            )
                            if box_added[idx]:
                                vis.remove_geometry(box_name)
                            vis.add_geometry(box_name, box_sets[idx])
                            box_added[idx] = True
                        elif box_added[idx]:
                            vis.remove_geometry(box_name)
                            box_added[idx] = False

                    update_labels(vis, detections_3d)
                    vis.post_redraw()
                    if window_closed or not gui_app.run_one_tick():
                        break
                else:
                    scene_pcd.points = o3d.utility.Vector3dVector(scene_points.astype(np.float64))
                    scene_pcd.colors = o3d.utility.Vector3dVector(scene_colors.astype(np.float64))
                    vis.update_geometry(scene_pcd)

                    first_center = None
                    for idx, box_set in enumerate(box_sets):
                        if idx < len(detections_3d):
                            detection = detections_3d[idx]
                            if detection.box_corners_xyz is not None:
                                update_line_set(
                                    box_set,
                                    detection.box_corners_xyz,
                                    color_for_index(idx),
                                    o3d,
                                )
                                if not box_added[idx]:
                                    vis.add_geometry(box_set, reset_bounding_box=False)
                                    box_added[idx] = True
                            elif box_added[idx]:
                                vis.remove_geometry(box_set, reset_bounding_box=False)
                                box_added[idx] = False
                            if first_center is None and detection.center_xyz is not None:
                                first_center = np.array(detection.center_xyz, dtype=np.float64)
                        elif box_added[idx]:
                            vis.remove_geometry(box_set, reset_bounding_box=False)
                            box_added[idx] = False

                        if box_added[idx]:
                            vis.update_geometry(box_set)

                    if first_center is None:
                        center_frame.translate(-np.asarray(center_frame.get_center()), relative=True)
                    else:
                        center_frame.translate(first_center - np.asarray(center_frame.get_center()), relative=True)
                    vis.update_geometry(center_frame)

                    if not vis.poll_events():
                        break
                    vis.update_renderer()

            frame_counter += 1
            if args.frames > 0 and frame_counter >= args.frames:
                break

    finally:
        if vis is not None:
            if hasattr(vis, "destroy_window"):
                vis.destroy_window()
            elif hasattr(vis, "close"):
                vis.close()
        if gui_app is not None and hasattr(gui_app, "quit"):
            gui_app.quit()
        stop_runtimes([runtime])

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        raise SystemExit(130)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
