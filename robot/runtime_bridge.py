#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import socketserver
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robo_interfaces.msg import GripperCommand, PositionOrientation, SetAngle


ROOT_DIR = Path("/home/misca/TabletopGraspSystem")


def joint_position_to_servo_angle_deg(joint_name: str, joint_position: float) -> float:
    if joint_name == "joint7_left":
        return (joint_position / 0.032) * 100.0 + 100.0
    return math.degrees(joint_position)


@dataclass
class RuntimeBridgeConfig:
    ros_setup_bash: str = "/opt/ros/humble/setup.bash"
    workspace_setup_bash: str = "/home/misca/starai_ws/install/setup.bash"
    host: str = "127.0.0.1"
    port: int = 8765
    position_topic: str = "/position_orientation_topic"
    gripper_topic: str = "/gripper_command_topic"
    set_angle_topic: str = "set_angle_topic"
    joint_state_topic: str = "/joint_states"
    logs_dir: Path = ROOT_DIR / "robot" / "logs"


class RosLaunchManager:
    def __init__(self, config: RuntimeBridgeConfig) -> None:
        self.config = config
        self._processes: dict[str, subprocess.Popen[str]] = {}
        self._log_files: dict[str, Any] = {}
        self._lock = threading.Lock()
        self.config.logs_dir.mkdir(parents=True, exist_ok=True)

    def _launch(self, name: str, ros_command: str) -> dict[str, Any]:
        with self._lock:
            proc = self._processes.get(name)
            if proc is not None and proc.poll() is None:
                return {"ok": True, "message": f"{name} already running", "name": name}

            full_command = (
                f"source {self.config.ros_setup_bash} && "
                f"source {self.config.workspace_setup_bash} && "
                "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH && "
                f"{ros_command}"
            )
            log_path = self.config.logs_dir / f"{name}.log"
            log_fp = open(log_path, "a", encoding="utf-8")
            proc = subprocess.Popen(
                ["bash", "-lc", full_command],
                stdout=log_fp,
                stderr=subprocess.STDOUT,
                text=True,
            )
            self._processes[name] = proc
            self._log_files[name] = log_fp
            return {
                "ok": True,
                "message": f"started {name}",
                "name": name,
                "pid": proc.pid,
                "log_path": str(log_path),
            }

    def start_driver(self) -> dict[str, Any]:
        return self._launch("driver", "ros2 launch cello_moveit_config driver.launch.py")

    def _stop_named(self, *names: str) -> None:
        with self._lock:
            for name in names:
                proc = self._processes.get(name)
                if proc is None:
                    continue
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)
                log_fp = self._log_files.pop(name, None)
                if log_fp is not None:
                    log_fp.close()
                self._processes.pop(name, None)

    def start_driver_free(self) -> dict[str, Any]:
        self._stop_named("driver", "driver_free", "actual_robot_demo", "moveit_write_read")
        return self._launch(
            "driver_free",
            "ros2 run robo_driver driver --ros-args -p robo_type:=cello -p lock:=disable",
        )

    def start_actual_robot_demo(self) -> dict[str, Any]:
        return self._launch(
            "actual_robot_demo",
            "ros2 launch cello_moveit_config actual_robot_demo.launch.py",
        )

    def start_moveit_write_read(self) -> dict[str, Any]:
        return self._launch(
            "moveit_write_read",
            "ros2 launch cello_moveit_config moveit_write_read.launch.py",
        )

    def start_full_stack(self) -> dict[str, Any]:
        """按固定顺序拉起抓取链依赖的三个 ROS 进程。"""
        return {
            "ok": True,
            "results": [
                self.start_driver(),
                self.start_actual_robot_demo(),
                self.start_moveit_write_read(),
            ],
        }

    def stop_all(self) -> dict[str, Any]:
        with self._lock:
            results: list[dict[str, Any]] = []
            for name, proc in list(self._processes.items()):
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=5)
                log_fp = self._log_files.pop(name, None)
                if log_fp is not None:
                    log_fp.close()
                results.append({"name": name, "returncode": proc.returncode})
            self._processes.clear()
            return {"ok": True, "results": results}

    def status(self) -> dict[str, Any]:
        """返回 bridge 当前托管的子进程状态。

        注意这里只能说明“进程是否活着”，不能说明：
        - MoveIt 是否规划成功
        - 机器人是否真的执行了位姿命令
        """
        with self._lock:
            processes = []
            for name, proc in self._processes.items():
                processes.append(
                    {
                        "name": name,
                        "pid": proc.pid,
                        "running": proc.poll() is None,
                        "returncode": proc.returncode,
                        "log_path": str(self.config.logs_dir / f"{name}.log"),
                    }
                )
            return {"ok": True, "processes": processes}


class RosRuntimeBridgeNode(Node):
    def __init__(self, config: RuntimeBridgeConfig) -> None:
        super().__init__("python_runtime_bridge")
        self.config = config
        self._lock = threading.Lock()
        self._latest_joint_state: dict[str, Any] | None = None
        self.position_publisher = self.create_publisher(
            PositionOrientation, self.config.position_topic, 10
        )
        self.gripper_publisher = self.create_publisher(
            GripperCommand, self.config.gripper_topic, 10
        )
        self.set_angle_publisher = self.create_publisher(
            SetAngle, self.config.set_angle_topic, 10
        )
        self.subscription = self.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        """缓存最近一条 joint_state，供 TCP 客户端轮询读取。

        bridge 只做转发和缓存，不做更强的时序保证；
        上层如果要判断“这条状态是否足够新鲜”，需要自己看时间戳。
        """
        with self._lock:
            names = list(msg.name)
            positions = [float(v) for v in msg.position]
            self._latest_joint_state = {
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "joint_names": names,
                "joint_positions": positions,
                "approx_servo_angles_deg": [
                    joint_position_to_servo_angle_deg(name, pos)
                    for name, pos in zip(names, positions)
                ],
            }

    def get_joint_state(self) -> dict[str, Any]:
        """返回 bridge 缓存的最新 joint_state。"""
        with self._lock:
            if self._latest_joint_state is None:
                return {"ok": False, "message": "No joint state received yet"}
            return {"ok": True, "joint_state": self._latest_joint_state}

    def publish_gripper(self, command_name: str) -> dict[str, Any]:
        if command_name not in {"open", "close"}:
            return {
                "ok": False,
                "message": f"Unsupported gripper command: {command_name}",
            }
        msg = GripperCommand()
        msg.command = command_name
        self.gripper_publisher.publish(msg)
        return {"ok": True, "gripper_command": command_name}

    def publish_pose(
        self,
        position_xyz_m: list[float],
        orientation_xyzw: list[float],
    ) -> dict[str, Any]:
        """把位姿目标发布到 position_orientation_topic。

        返回 ok=true 仅表示“消息成功发到 topic”，并不代表：
        - MoveIt 已经规划成功
        - 机械臂已经开始运动
        """
        if len(position_xyz_m) != 3 or len(orientation_xyzw) != 4:
            return {
                "ok": False,
                "message": "position_xyz_m must have 3 values and orientation_xyzw must have 4 values",
            }
        msg = PositionOrientation()
        msg.position_x = float(position_xyz_m[0])
        msg.position_y = float(position_xyz_m[1])
        msg.position_z = float(position_xyz_m[2])
        msg.orientation_x = float(orientation_xyzw[0])
        msg.orientation_y = float(orientation_xyzw[1])
        msg.orientation_z = float(orientation_xyzw[2])
        msg.orientation_w = float(orientation_xyzw[3])
        self.position_publisher.publish(msg)
        return {
            "ok": True,
            "position_xyz_m": position_xyz_m,
            "orientation_xyzw": orientation_xyzw,
        }

    def publish_set_angle(
        self,
        servo_ids: list[int],
        target_angles_deg: list[float],
        time_ms: list[int],
    ) -> dict[str, Any]:
        """直接发布舵机角度命令，绕过 MoveIt。"""
        if not (len(servo_ids) == len(target_angles_deg) == len(time_ms)):
            return {"ok": False, "message": "servo_ids, target_angles_deg, and time_ms must have the same length"}
        msg = SetAngle()
        msg.servo_id = [int(v) for v in servo_ids]
        msg.target_angle = [float(v) for v in target_angles_deg]
        msg.time = [int(v) for v in time_ms]
        msg.speed = []
        self.set_angle_publisher.publish(msg)
        return {
            "ok": True,
            "servo_ids": [int(v) for v in msg.servo_id],
            "target_angles_deg": [float(v) for v in msg.target_angle],
            "time_ms": [int(v) for v in msg.time],
        }


class BridgeRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server: BridgeTcpServer = self.server  # type: ignore[assignment]
        line = self.rfile.readline().decode("utf-8").strip()
        if not line:
            return
        try:
            request = json.loads(line)
            response = server.dispatch(request)
        except Exception as exc:  # noqa: BLE001
            response = {"ok": False, "message": str(exc)}
        self.wfile.write((json.dumps(response, ensure_ascii=True) + "\n").encode("utf-8"))


class BridgeTcpServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        handler_class: type[BridgeRequestHandler],
        ros_node: RosRuntimeBridgeNode,
        launch_manager: RosLaunchManager,
    ) -> None:
        super().__init__(server_address, handler_class)
        self.ros_node = ros_node
        self.launch_manager = launch_manager

    def dispatch(self, request: dict[str, Any]) -> dict[str, Any]:
        """把简单的 TCP JSON 命令映射到 bridge 能力。"""
        command = request.get("cmd")
        if command == "status":
            status = self.launch_manager.status()
            status["joint_state"] = self.ros_node.get_joint_state()
            return status
        if command == "start_driver":
            return self.launch_manager.start_driver()
        if command == "start_driver_free":
            return self.launch_manager.start_driver_free()
        if command == "start_full_stack":
            return self.launch_manager.start_full_stack()
        if command == "stop_all":
            return self.launch_manager.stop_all()
        if command == "get_joint_state":
            return self.ros_node.get_joint_state()
        if command == "send_gripper":
            return self.ros_node.publish_gripper(str(request.get("gripper_command", "")))
        if command == "send_pose":
            return self.ros_node.publish_pose(
                position_xyz_m=[float(v) for v in request.get("position_xyz_m", [])],
                orientation_xyzw=[float(v) for v in request.get("orientation_xyzw", [])],
            )
        if command == "send_set_angle":
            return self.ros_node.publish_set_angle(
                servo_ids=[int(v) for v in request.get("servo_ids", [])],
                target_angles_deg=[float(v) for v in request.get("target_angles_deg", [])],
                time_ms=[int(v) for v in request.get("time_ms", [])],
            )
        if command == "sleep":
            seconds = max(float(request.get("seconds", 0.0)), 0.0)
            time.sleep(seconds)
            return {"ok": True, "slept_seconds": seconds}
        return {"ok": False, "message": f"Unknown command: {command}"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ROS2 runtime bridge for Python clients.")
    parser.add_argument("--host", default="127.0.0.1", help="TCP host to bind.")
    parser.add_argument("--port", type=int, default=8765, help="TCP port to bind.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = RuntimeBridgeConfig(host=args.host, port=args.port)
    launch_manager = RosLaunchManager(config)

    rclpy.init()
    node = RosRuntimeBridgeNode(config)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    server = BridgeTcpServer((config.host, config.port), BridgeRequestHandler, node, launch_manager)
    print(
        json.dumps(
            {
                "ok": True,
                "message": "runtime bridge started",
                "host": config.host,
                "port": config.port,
            },
            ensure_ascii=True,
        ),
        flush=True,
    )

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        launch_manager.stop_all()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
