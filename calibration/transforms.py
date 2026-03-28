from __future__ import annotations

from typing import Any


class CameraRobotTransform:
    """Placeholder for camera-to-robot coordinate transforms."""

    def __init__(self, calibration_data: dict[str, Any] | None = None) -> None:
        self.calibration_data = calibration_data or {}

    def camera_to_robot_target(self, detection: dict[str, Any]) -> dict[str, Any]:
        """Convert a perception target from camera frame to robot frame.

        This is currently a stub and should later consume real hand-eye calibration.
        """

        return {
            "class_name": detection.get("class_name"),
            "center_robot_xyz_m": detection.get("center_camera_xyz_m"),
            "yaw_deg": detection.get("yaw_deg"),
            "extent_xyz_m": detection.get("extent_xyz_m"),
        }
