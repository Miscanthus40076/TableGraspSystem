from __future__ import annotations

from typing import Any


class TabletopGraspPlanner:
    """Minimal placeholder grasp planner."""

    def build_grasp_pose(self, robot_target: dict[str, Any]) -> dict[str, Any]:
        return {
            "approach_xyz_m": robot_target.get("center_robot_xyz_m"),
            "grasp_yaw_deg": robot_target.get("yaw_deg"),
            "class_name": robot_target.get("class_name"),
        }
