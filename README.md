# TabletopGraspSystem

Chinese documentation: [README_cn.md](./README_cn.md)

Integrated tabletop grasping system that combines:

- perception from `TabletopSeg3D`
- robot control from an external robot codebase
- calibration between camera and robot
- grasp planning and execution coordination

## Layout

```text
TabletopGraspSystem/
├── README.md
├── .gitignore
├── app/
│   ├── __init__.py
│   └── coordinator.py
├── calibration/
│   ├── __init__.py
│   └── transforms.py
├── planning/
│   ├── __init__.py
│   └── grasp_pose.py
├── robot/
│   ├── __init__.py
│   └── adapter.py
└── perception/
    └── tabletopseg3d/
        ├── README.md
        ├── README_cn.md
        ├── requirements.txt
        ├── yolo11n-seg.pt
        ├── photo/
        ├── scripts/
        └── src/
```

## Design

The repository is structured so that perception remains reusable and the robot stack remains replaceable.

- `perception/tabletopseg3d/`
  - contains the current realtime vision pipeline
  - outputs object class, center, extent, and tabletop-aligned yaw
- `robot/`
  - wraps the external robot-control repository behind a small adapter interface
- `calibration/`
  - converts camera-frame detections into robot-base coordinates
- `planning/`
  - generates grasp poses from perception outputs
- `app/`
  - orchestrates the full flow: detect -> transform -> plan -> execute

## Recommended Integration Strategy

Do not merge external robot code directly into the perception module.

Instead:

1. Keep `TabletopSeg3D` as the perception module.
2. Keep the robot-control repository independently maintained.
3. Implement a thin adapter in `robot/adapter.py`.
4. Let `app/coordinator.py` coordinate both sides.

## Install Official StarAI Arm ROS2 Package

The official ROS2 package used for the robot side is:

- `fashionstar-starai-arm-ros2`
- repository: `https://github.com/Seeed-Projects/fashionstar-starai-arm-ros2.git`

Recommended installation steps on this machine:

```bash
mkdir -p ~/starai_ws/src
cd ~/starai_ws/src
git clone https://github.com/Seeed-Projects/fashionstar-starai-arm-ros2.git
```

Before building, make sure ROS 2 Humble is sourced:

```bash
source /opt/ros/humble/setup.bash
```

Then build the workspace:

```bash
cd ~/starai_ws
colcon build
```

To make the workspace available in future shells:

```bash
echo "source ~/starai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install the driver dependencies into the system Python as well:

```bash
/usr/bin/python3 -m pip install --user pyserial fashionstar-uart-sdk
```

This is important because `robo_driver` is launched by ROS 2 with the system Python, not the conda environment used by the vision stack.

Important note for conda users:

- if conda changes your default `python3`, ROS package builds may fail
- on this machine the stable build path was:

```bash
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH
cd ~/starai_ws
colcon build
```

Build verification on this machine:

- ROS distro: `Humble`
- workspace: `~/starai_ws`
- result: `14 packages finished`
- `robo_driver` also required `pyserial` and `fashionstar-uart-sdk` to be installed into `/usr/bin/python3`

## Current ROS2 Bridge

The first bridge layer is implemented in:

- [`robot/adapter.py`](./robot/adapter.py)

Current behavior:

- publishes end-effector pose to `/position_orientation_topic`
- publishes gripper commands to `/gripper_command_topic`
- uses `ros2 topic pub --once` through subprocess
- avoids direct `rclpy` import inside the conda-driven application layer

## Current Status

This repository is now initialized as the next-stage system skeleton.

The perception module has already been copied in.
The remaining files define the first integration interfaces for robot grasping.
