from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import pyrealsense2 as rs


@dataclass(frozen=True)
class DeviceInfo:
    name: str
    serial_number: str
    firmware_version: str
    usb_type_descriptor: str
    product_line: str


@dataclass
class CameraRuntime:
    info: DeviceInfo
    pipeline: rs.pipeline
    align: rs.align
    depth_scale: float


def safe_get_info(device: rs.device, info_key: rs.camera_info) -> str:
    """Return device metadata when available without failing the whole scan."""

    try:
        return device.get_info(info_key)
    except RuntimeError:
        return ""


def enumerate_devices() -> list[DeviceInfo]:
    """Collect a small stable snapshot of connected RealSense devices."""

    ctx = rs.context()
    devices = []
    for dev in ctx.query_devices():
        devices.append(
            DeviceInfo(
                name=safe_get_info(dev, rs.camera_info.name),
                serial_number=safe_get_info(dev, rs.camera_info.serial_number),
                firmware_version=safe_get_info(dev, rs.camera_info.firmware_version),
                usb_type_descriptor=safe_get_info(dev, rs.camera_info.usb_type_descriptor),
                product_line=safe_get_info(dev, rs.camera_info.product_line),
            )
        )
    return devices


def select_serials(devices: list[DeviceInfo], serials: list[str] | None, expected_count: int = 2) -> list[str]:
    available = {dev.serial_number for dev in devices}
    if serials:
        missing = [serial for serial in serials if serial not in available]
        if missing:
            raise RuntimeError(f"Requested serials not found: {missing}")
        return serials

    if len(devices) < expected_count:
        raise RuntimeError(
            f"Expected at least {expected_count} RealSense devices, found {len(devices)}. "
            "Use --list-devices to inspect detection state."
        )

    return [device.serial_number for device in devices[:expected_count]]


def intrinsics_to_dict(intrinsics: rs.intrinsics) -> dict[str, Any]:
    return {
        "width": intrinsics.width,
        "height": intrinsics.height,
        "fx": intrinsics.fx,
        "fy": intrinsics.fy,
        "ppx": intrinsics.ppx,
        "ppy": intrinsics.ppy,
        "model": str(intrinsics.model),
        "coeffs": list(intrinsics.coeffs),
    }


def build_runtime(
    device_info: DeviceInfo,
    width: int,
    height: int,
    fps: int,
    enable_depth: bool = True,
    enable_color: bool = True,
    *,
    color_width: int | None = None,
    color_height: int | None = None,
    color_fps: int | None = None,
    depth_width: int | None = None,
    depth_height: int | None = None,
    depth_fps: int | None = None,
) -> CameraRuntime:
    """Start one aligned color/depth pipeline for a selected RealSense device."""

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(device_info.serial_number)
    effective_color_width = color_width or width
    effective_color_height = color_height or height
    effective_color_fps = color_fps or fps
    effective_depth_width = depth_width or width
    effective_depth_height = depth_height or height
    effective_depth_fps = depth_fps or fps
    if enable_depth:
        config.enable_stream(
            rs.stream.depth,
            int(effective_depth_width),
            int(effective_depth_height),
            rs.format.z16,
            int(effective_depth_fps),
        )
    if enable_color:
        config.enable_stream(
            rs.stream.color,
            int(effective_color_width),
            int(effective_color_height),
            rs.format.bgr8,
            int(effective_color_fps),
        )
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.enable_auto_exposure):
        depth_sensor.set_option(rs.option.enable_auto_exposure, 1)

    return CameraRuntime(
        info=device_info,
        pipeline=pipeline,
        # Depth is aligned to color so a segmentation pixel can directly index
        # its corresponding depth sample in the returned frame bundle.
        align=rs.align(rs.stream.color),
        depth_scale=depth_sensor.get_depth_scale(),
    )


def get_aligned_frame_bundle(
    runtime: CameraRuntime,
    depth_min_m: float,
    depth_max_m: float,
    retries: int = 5,
    retry_sleep_sec: float = 0.2,
    timeout_ms: int = 5000,
) -> dict[str, Any]:
    """Fetch one aligned color/depth frame pair with a few startup retries."""

    last_error: RuntimeError | None = None
    aligned_frames = None
    color_frame = None
    depth_frame = None
    for _ in range(max(int(retries), 1)):
        try:
            frames = runtime.pipeline.wait_for_frames(int(timeout_ms))
            aligned_frames = runtime.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if color_frame and depth_frame:
                break
            last_error = RuntimeError(
                f"Missing color or depth frame from RealSense pipeline {runtime.info.serial_number}."
            )
        except RuntimeError as exc:
            last_error = exc
        if retry_sleep_sec > 0:
            import time

            # The first few frames after stream start can be incomplete.
            time.sleep(float(retry_sleep_sec))
    if aligned_frames is None or not color_frame or not depth_frame:
        raise last_error or RuntimeError(
            f"Missing color or depth frame from RealSense pipeline {runtime.info.serial_number}."
        )

    color = np.asanyarray(color_frame.get_data())
    depth = np.asanyarray(depth_frame.get_data())

    color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()

    return {
        "serial_number": runtime.info.serial_number,
        "device_name": runtime.info.name,
        "firmware_version": runtime.info.firmware_version,
        "product_line": runtime.info.product_line,
        "usb_type_descriptor": runtime.info.usb_type_descriptor,
        "color": color,
        "depth": depth,
        "timestamp_ms": aligned_frames.get_timestamp(),
        "frame_number": aligned_frames.get_frame_number(),
        "depth_scale": runtime.depth_scale,
        "color_intrinsics": intrinsics_to_dict(color_intrinsics),
        "depth_intrinsics": intrinsics_to_dict(depth_intrinsics),
    }


def stop_runtimes(runtimes: list[CameraRuntime]) -> None:
    for runtime in runtimes:
        try:
            runtime.pipeline.stop()
        except RuntimeError:
            pass
