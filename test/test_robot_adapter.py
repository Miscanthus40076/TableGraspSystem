from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robot.adapter import DEFAULT_TOPDOWN_QUATERNION_XYZW, RobotAdapter


def main() -> None:
    adapter = RobotAdapter(dry_run=True)
    result = adapter.execute_grasp(
        {
            "approach_xyz_m": [0.278, 0.0, 0.438],
            "orientation_xyzw": list(DEFAULT_TOPDOWN_QUATERNION_XYZW),
            "gripper_command": "open",
            "grasp_yaw_deg": 0.0,
        }
    )
    print(result)


if __name__ == "__main__":
    main()
