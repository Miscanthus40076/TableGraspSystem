from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Any


@dataclass
class RuntimeBridgeClient:
    host: str = "127.0.0.1"
    port: int = 8765
    timeout_sec: float = 5.0

    def request(self, payload: dict[str, Any]) -> dict[str, Any]:
        body = (json.dumps(payload, ensure_ascii=True) + "\n").encode("utf-8")
        with socket.create_connection((self.host, self.port), timeout=self.timeout_sec) as sock:
            sock.sendall(body)
            sock_file = sock.makefile("rb")
            line = sock_file.readline()
        if not line:
            return {"ok": False, "message": "No response from runtime bridge"}
        return json.loads(line.decode("utf-8"))

    def status(self) -> dict[str, Any]:
        return self.request({"cmd": "status"})

    def start_driver(self) -> dict[str, Any]:
        return self.request({"cmd": "start_driver"})

    def start_driver_free(self) -> dict[str, Any]:
        return self.request({"cmd": "start_driver_free"})

    def start_full_stack(self) -> dict[str, Any]:
        return self.request({"cmd": "start_full_stack"})

    def stop_all(self) -> dict[str, Any]:
        return self.request({"cmd": "stop_all"})

    def get_joint_state(self) -> dict[str, Any]:
        return self.request({"cmd": "get_joint_state"})

    def send_gripper(self, command_name: str) -> dict[str, Any]:
        return self.request({"cmd": "send_gripper", "gripper_command": command_name})

    def send_pose(
        self,
        *,
        position_xyz_m: list[float],
        orientation_xyzw: list[float],
    ) -> dict[str, Any]:
        return self.request(
            {
                "cmd": "send_pose",
                "position_xyz_m": position_xyz_m,
                "orientation_xyzw": orientation_xyzw,
            }
        )

    def send_set_angle(
        self,
        *,
        servo_ids: list[int],
        target_angles_deg: list[float],
        time_ms: list[int],
    ) -> dict[str, Any]:
        return self.request(
            {
                "cmd": "send_set_angle",
                "servo_ids": servo_ids,
                "target_angles_deg": target_angles_deg,
                "time_ms": time_ms,
            }
        )
