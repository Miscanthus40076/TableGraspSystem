from __future__ import annotations

import math
import shlex
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_TOPDOWN_QUATERNION_XYZW = (-0.506, 0.507, -0.496, 0.491)


def yaw_deg_to_quaternion_xyzw(yaw_deg: float) -> tuple[float, float, float, float]:
    """Build a simple Z-axis yaw quaternion.

    This helper is available if later you want to replace the fixed top-down
    orientation with a yaw-derived orientation.
    """

    yaw_rad = math.radians(yaw_deg)
    half = 0.5 * yaw_rad
    return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass
class Ros2BridgeConfig:
    ros_setup_bash: str = "/opt/ros/humble/setup.bash"
    workspace_setup_bash: str = "/home/misca/starai_ws/install/setup.bash"
    position_topic: str = "/position_orientation_topic"
    gripper_topic: str = "/gripper_command_topic"
    use_system_python_path: bool = True


class RobotAdapter:
    """ROS2 topic bridge for the official StarAI arm package.

    Current strategy:
    - publish one PositionOrientation message to `/position_orientation_topic`
    - optionally publish one GripperCommand message to `/gripper_command_topic`

    This adapter intentionally uses `ros2 topic pub --once` through subprocess
    instead of importing `rclpy` directly, which avoids common conflicts between
    ROS2 Humble and conda-managed Python environments.
    """

    def __init__(self, backend: Any | None = None, config: Ros2BridgeConfig | None = None, dry_run: bool = False) -> None:
        self.backend = backend
        self.config = config or Ros2BridgeConfig()
        self.dry_run = dry_run

    def _ros_shell_prefix(self) -> str:
        parts = [
            f"source {shlex.quote(self.config.ros_setup_bash)}",
            f"source {shlex.quote(self.config.workspace_setup_bash)}",
        ]
        if self.config.use_system_python_path:
            parts.append("export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH")
        return " && ".join(parts)

    def _run_ros_command(self, command: str) -> subprocess.CompletedProcess[str] | None:
        full_command = f"{self._ros_shell_prefix()} && {command}"
        if self.dry_run:
            return None
        return subprocess.run(
            ["bash", "-lc", full_command],
            check=False,
            text=True,
            capture_output=True,
        )

    def send_pose(
        self,
        *,
        position_xyz_m: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float] | None = None,
    ) -> dict[str, Any]:
        orientation_xyzw = orientation_xyzw or DEFAULT_TOPDOWN_QUATERNION_XYZW
        x, y, z = position_xyz_m
        qx, qy, qz, qw = orientation_xyzw

        msg = (
            "{"
            f"position_x: {x}, position_y: {y}, position_z: {z}, "
            f"orientation_x: {qx}, orientation_y: {qy}, orientation_z: {qz}, orientation_w: {qw}"
            "}"
        )
        command = (
            "ros2 topic pub --once "
            f"{shlex.quote(self.config.position_topic)} "
            "robo_interfaces/msg/PositionOrientation "
            f"\"{msg}\""
        )
        result = self._run_ros_command(command)
        return {
            "ok": True if self.dry_run else bool(result and result.returncode == 0),
            "command": command,
            "stdout": "" if result is None else result.stdout.strip(),
            "stderr": "" if result is None else result.stderr.strip(),
            "position_xyz_m": list(position_xyz_m),
            "orientation_xyzw": list(orientation_xyzw),
        }

    def send_gripper(self, command_name: str) -> dict[str, Any]:
        if command_name not in {"open", "close"}:
            return {
                "ok": False,
                "message": f"Unsupported gripper command: {command_name}",
                "requested": command_name,
            }

        msg = "{command: " + command_name + "}"
        command = (
            "ros2 topic pub --once "
            f"{shlex.quote(self.config.gripper_topic)} "
            "robo_interfaces/msg/GripperCommand "
            f"\"{msg}\""
        )
        result = self._run_ros_command(command)
        return {
            "ok": True if self.dry_run else bool(result and result.returncode == 0),
            "command": command,
            "stdout": "" if result is None else result.stdout.strip(),
            "stderr": "" if result is None else result.stderr.strip(),
            "gripper_command": command_name,
        }

    def execute_grasp(self, grasp_pose: dict[str, Any]) -> dict[str, Any]:
        position = grasp_pose.get("approach_xyz_m")
        if position is None:
            return {
                "ok": False,
                "message": "Missing approach_xyz_m in grasp pose.",
                "requested_grasp_pose": grasp_pose,
            }

        orientation = grasp_pose.get("orientation_xyzw")
        gripper_command = grasp_pose.get("gripper_command")

        pose_result = self.send_pose(
            position_xyz_m=tuple(float(v) for v in position),
            orientation_xyzw=None if orientation is None else tuple(float(v) for v in orientation),
        )
        gripper_result = None
        if gripper_command:
            gripper_result = self.send_gripper(str(gripper_command))

        ok = bool(pose_result.get("ok")) and (gripper_result is None or bool(gripper_result.get("ok")))
        return {
            "ok": ok,
            "pose_result": pose_result,
            "gripper_result": gripper_result,
            "requested_grasp_pose": grasp_pose,
        }


if __name__ == "__main__":
    adapter = RobotAdapter(dry_run=True)
    demo = adapter.execute_grasp(
        {
            "approach_xyz_m": [0.278, 0.0, 0.438],
            "orientation_xyzw": list(DEFAULT_TOPDOWN_QUATERNION_XYZW),
            "gripper_command": "open",
            "grasp_yaw_deg": 0.0,
        }
    )
    print(demo)
