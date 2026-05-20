from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis_xyz: np.ndarray
    mimic_joint: str | None = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0


def _parse_xyz(text: str | None) -> np.ndarray:
    if not text:
        return np.zeros(3, dtype=np.float64)
    return np.asarray([float(v) for v in text.split()], dtype=np.float64)


def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _axis_angle_to_rotation(axis_xyz: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis_xyz, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
        ],
        dtype=np.float64,
    )


def _make_transform(rotation: np.ndarray, translation_xyz: np.ndarray) -> np.ndarray:
    tf = np.eye(4, dtype=np.float64)
    tf[:3, :3] = rotation
    tf[:3, 3] = np.asarray(translation_xyz, dtype=np.float64).reshape(3)
    return tf


def _joint_motion_transform(spec: JointSpec, position: float) -> np.ndarray:
    if spec.joint_type in {"revolute", "continuous"}:
        return _make_transform(_axis_angle_to_rotation(spec.axis_xyz, position), np.zeros(3, dtype=np.float64))
    if spec.joint_type == "prismatic":
        return _make_transform(np.eye(3, dtype=np.float64), spec.axis_xyz * float(position))
    return np.eye(4, dtype=np.float64)


@lru_cache(maxsize=8)
def load_joint_specs(urdf_path: str) -> dict[str, JointSpec]:
    root = ET.fromstring(Path(urdf_path).read_text(encoding="utf-8"))
    specs: dict[str, JointSpec] = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        joint_type = joint.attrib["type"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        axis = joint.find("axis")
        mimic = joint.find("mimic")
        specs[name] = JointSpec(
            name=name,
            joint_type=joint_type,
            parent=parent,
            child=child,
            origin_xyz=_parse_xyz(None if origin is None else origin.attrib.get("xyz")),
            origin_rpy=_parse_xyz(None if origin is None else origin.attrib.get("rpy")),
            axis_xyz=_parse_xyz(None if axis is None else axis.attrib.get("xyz")),
            mimic_joint=None if mimic is None else mimic.attrib.get("joint"),
            mimic_multiplier=1.0 if mimic is None else float(mimic.attrib.get("multiplier", "1.0")),
            mimic_offset=0.0 if mimic is None else float(mimic.attrib.get("offset", "0.0")),
        )
    return specs


@lru_cache(maxsize=32)
def build_link_chain(urdf_path: str, root_link: str, target_link: str) -> list[JointSpec]:
    specs = load_joint_specs(urdf_path)
    by_child = {spec.child: spec for spec in specs.values()}
    chain: list[JointSpec] = []
    current = target_link
    while current != root_link:
        if current not in by_child:
            raise ValueError(f"Could not build chain from {root_link} to {target_link}: missing parent for link {current}")
        spec = by_child[current]
        chain.append(spec)
        current = spec.parent
    chain.reverse()
    return chain


def _resolve_joint_value(spec: JointSpec, joint_positions: dict[str, float]) -> float:
    if spec.mimic_joint:
        base = joint_positions.get(spec.mimic_joint, 0.0)
        return spec.mimic_multiplier * base + spec.mimic_offset
    return joint_positions.get(spec.name, 0.0)


def compute_link_transform(
    *,
    urdf_path: str | Path,
    joint_positions: dict[str, float],
    root_link: str = "base_link",
    target_link: str,
) -> np.ndarray:
    chain = build_link_chain(str(Path(urdf_path)), root_link, target_link)
    tf = np.eye(4, dtype=np.float64)
    for spec in chain:
        origin_tf = _make_transform(_rpy_to_rotation(spec.origin_rpy), spec.origin_xyz)
        joint_tf = _joint_motion_transform(spec, _resolve_joint_value(spec, joint_positions))
        tf = tf @ origin_tf @ joint_tf
    return tf


def compute_tcp_point_in_base(
    *,
    urdf_path: str | Path,
    joint_positions: dict[str, float],
    tcp_link: str,
    tcp_offset_xyz_m: list[float] | tuple[float, float, float] | np.ndarray,
    root_link: str = "base_link",
) -> np.ndarray:
    tf_base_link = compute_link_transform(
        urdf_path=urdf_path,
        joint_positions=joint_positions,
        root_link=root_link,
        target_link=tcp_link,
    )
    tcp_h = np.ones(4, dtype=np.float64)
    tcp_h[:3] = np.asarray(tcp_offset_xyz_m, dtype=np.float64).reshape(3)
    return (tf_base_link @ tcp_h)[:3]

