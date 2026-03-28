from __future__ import annotations

from typing import Any


class GraspCoordinator:
    """High-level coordinator for the integrated grasp pipeline.

    Intended flow:
    1. Receive perception output from TabletopSeg3D.
    2. Transform target pose from camera frame to robot frame.
    3. Generate a grasp pose.
    4. Send motion commands through the robot adapter.
    """

    def __init__(self, robot_adapter: Any, transform_provider: Any, grasp_planner: Any) -> None:
        self.robot_adapter = robot_adapter
        self.transform_provider = transform_provider
        self.grasp_planner = grasp_planner

    def run_once(self, detection: dict[str, Any]) -> dict[str, Any]:
        robot_target = self.transform_provider.camera_to_robot_target(detection)
        grasp_pose = self.grasp_planner.build_grasp_pose(robot_target)
        result = self.robot_adapter.execute_grasp(grasp_pose)
        return {
            "robot_target": robot_target,
            "grasp_pose": grasp_pose,
            "execution": result,
        }
