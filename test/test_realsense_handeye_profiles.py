#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, asdict
from typing import Any

import pyrealsense2 as rs


@dataclass(frozen=True)
class StreamSpec:
    width: int
    height: int
    fps: int
    fmt: Any


@dataclass(frozen=True)
class ProfileCombo:
    color: StreamSpec
    depth: StreamSpec


def stream_spec_to_dict(spec: StreamSpec) -> dict[str, Any]:
    return {
        "width": int(spec.width),
        "height": int(spec.height),
        "fps": int(spec.fps),
        "format": str(spec.fmt),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Probe RealSense color+depth profile combinations for handeye calibration."
    )
    parser.add_argument("--serial", required=True, help="Target RealSense serial number.")
    parser.add_argument("--warmup", type=int, default=5, help="Frames to read after pipeline start.")
    parser.add_argument("--list-only", action="store_true", help="Only list sensor-advertised profiles.")
    return parser.parse_args()


def find_device(serial: str) -> rs.device:
    ctx = rs.context()
    for dev in ctx.query_devices():
        if dev.get_info(rs.camera_info.serial_number) == serial:
            return dev
    raise RuntimeError(f"RealSense serial not found: {serial}")


def list_sensor_profiles(dev: rs.device) -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for sensor in dev.query_sensors():
        sensor_name = sensor.get_info(rs.camera_info.name)
        rows: list[dict[str, Any]] = []
        seen: set[tuple[str, int, int, int, str]] = set()
        for profile in sensor.get_stream_profiles():
            try:
                vp = profile.as_video_stream_profile()
                item = (
                    profile.stream_name(),
                    vp.width(),
                    vp.height(),
                    profile.fps(),
                    str(profile.format()),
                )
            except Exception:
                continue
            if item in seen:
                continue
            seen.add(item)
            rows.append(
                {
                    "stream": item[0],
                    "width": item[1],
                    "height": item[2],
                    "fps": item[3],
                    "format": item[4],
                }
            )
        result[sensor_name] = rows
    return result


def candidate_combos() -> list[ProfileCombo]:
    bgr8 = rs.format.bgr8
    z16 = rs.format.z16
    return [
        ProfileCombo(StreamSpec(1920, 1080, 8, bgr8), StreamSpec(640, 480, 6, z16)),
        ProfileCombo(StreamSpec(1920, 1080, 8, bgr8), StreamSpec(640, 480, 15, z16)),
        ProfileCombo(StreamSpec(1920, 1080, 8, bgr8), StreamSpec(640, 480, 30, z16)),
        ProfileCombo(StreamSpec(1280, 720, 15, bgr8), StreamSpec(640, 480, 15, z16)),
        ProfileCombo(StreamSpec(1280, 720, 15, bgr8), StreamSpec(640, 480, 30, z16)),
        ProfileCombo(StreamSpec(1280, 720, 10, bgr8), StreamSpec(848, 480, 10, z16)),
        ProfileCombo(StreamSpec(1280, 720, 6, bgr8), StreamSpec(1280, 720, 6, z16)),
        ProfileCombo(StreamSpec(640, 480, 30, bgr8), StreamSpec(640, 480, 30, z16)),
    ]


def try_combo(serial: str, combo: ProfileCombo, warmup: int) -> dict[str, Any]:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(
        rs.stream.color,
        combo.color.width,
        combo.color.height,
        combo.color.fmt,
        combo.color.fps,
    )
    cfg.enable_stream(
        rs.stream.depth,
        combo.depth.width,
        combo.depth.height,
        combo.depth.fmt,
        combo.depth.fps,
    )
    try:
        profile = pipe.start(cfg)
        align = rs.align(rs.stream.color)
        for _ in range(max(int(warmup), 0)):
            frames = pipe.wait_for_frames()
            align.process(frames)
        frames = pipe.wait_for_frames()
        aligned = align.process(frames)
        color = aligned.get_color_frame()
        depth = aligned.get_depth_frame()
        if not color or not depth:
            return {
                "ok": False,
                "reason": "aligned_frames_missing",
                "combo": {
                    "color": stream_spec_to_dict(combo.color),
                    "depth": stream_spec_to_dict(combo.depth),
                },
            }
        color_intr = color.profile.as_video_stream_profile().get_intrinsics()
        depth_intr = depth.profile.as_video_stream_profile().get_intrinsics()
        return {
            "ok": True,
            "combo": {
                "color": stream_spec_to_dict(combo.color),
                "depth": stream_spec_to_dict(combo.depth),
            },
            "aligned_color": {
                "width": color_intr.width,
                "height": color_intr.height,
                "fx": color_intr.fx,
                "fy": color_intr.fy,
            },
            "aligned_depth": {
                "width": depth_intr.width,
                "height": depth_intr.height,
                "fx": depth_intr.fx,
                "fy": depth_intr.fy,
            },
            "device_name": profile.get_device().get_info(rs.camera_info.name),
        }
    except Exception as exc:  # noqa: BLE001
        return {
            "ok": False,
            "reason": repr(exc),
            "combo": {
                "color": stream_spec_to_dict(combo.color),
                "depth": stream_spec_to_dict(combo.depth),
            },
        }
    finally:
        try:
            pipe.stop()
        except Exception:
            pass


def main() -> None:
    args = parse_args()
    dev = find_device(args.serial)
    print(
        json.dumps(
            {
                "device": dev.get_info(rs.camera_info.name),
                "serial": dev.get_info(rs.camera_info.serial_number),
                "profiles": list_sensor_profiles(dev),
            },
            ensure_ascii=True,
            indent=2,
        )
    )
    if args.list_only:
        return
    print("\n=== handeye candidate combos ===", flush=True)
    for combo in candidate_combos():
        result = try_combo(args.serial, combo, args.warmup)
        print(json.dumps(result, ensure_ascii=True), flush=True)


if __name__ == "__main__":
    main()
