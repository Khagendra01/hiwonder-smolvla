#!/usr/bin/env python3
"""Minimal URDF forward kinematics for the JetArm (no Isaac/ROS needed).

Parses <joint> origins/axes and evaluates the chain root -> named tip.
Only revolute and fixed joints are used here.

  python3 sim/tools/urdf_fk.py sim/urdf/jetarm_6dof.urdf --tip endpoint --joints 0 0 0 0 0
"""
from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def quat_from_matrix(R: np.ndarray) -> tuple[float, float, float, float]:
    """Return (x, y, z, w), the same order as geometry_msgs/Quaternion."""
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    axis: np.ndarray


class UrdfArm:
    def __init__(self, path: str, root: str = "world"):
        tree = ET.parse(path)
        robot = tree.getroot()
        self.joints: dict[str, Joint] = {}
        self.parent_joint: dict[str, Joint] = {}
        for el in robot.findall("joint"):
            name = el.get("name")
            origin = el.find("origin")
            xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0").split() if origin is not None else "0 0 0".split()))
            rpy = tuple(float(v) for v in (origin.get("rpy", "0 0 0").split() if origin is not None else "0 0 0".split()))
            ax = el.find("axis")
            axis = np.array([float(v) for v in (ax.get("xyz", "1 0 0").split() if ax is not None else "1 0 0".split())])
            j = Joint(name, el.get("type"), el.find("parent").get("link"), el.find("child").get("link"), xyz, rpy, axis)
            self.joints[name] = j
            self.parent_joint[j.child] = j
        self.root = root

    def chain(self, tip: str) -> list[Joint]:
        out = []
        link = tip
        while link in self.parent_joint:
            j = self.parent_joint[link]
            out.append(j)
            link = j.parent
        out.reverse()
        if link != self.root:
            raise ValueError(f"chain ends at {link}, not {self.root}")
        return out

    def fk(self, angles: dict[str, float], tip: str = "endpoint") -> np.ndarray:
        T = np.eye(4)
        for j in self.chain(tip):
            T = T @ transform(j.xyz, j.rpy)
            if j.kind in ("revolute", "continuous"):
                R = np.eye(4)
                R[:3, :3] = axis_angle(j.axis, angles.get(j.name, 0.0))
                T = T @ R
            elif j.kind != "fixed":
                raise ValueError(f"unsupported joint type {j.kind} ({j.name})")
        return T


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--tip", default="endpoint")
    ap.add_argument("--root", default="world")
    ap.add_argument("--joints", nargs=5, type=float, default=[0, 0, 0, 0, 0],
                    help="joint1..joint5 angles in radians")
    args = ap.parse_args()
    arm = UrdfArm(args.urdf, root=args.root)
    names = [j.name for j in arm.chain(args.tip) if j.kind == "revolute"]
    angles = dict(zip(names, args.joints))
    T = arm.fk(angles, tip=args.tip)
    x, y, z, w = quat_from_matrix(T[:3, :3])
    print("revolute chain:", names)
    print("position:", np.round(T[:3, 3], 6).tolist())
    print("quat xyzw:", [round(v, 6) for v in (x, y, z, w)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
