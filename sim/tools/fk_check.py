#!/usr/bin/env python3
"""Compare the URDF FK with Hiwonder's compiled FK over random pulse poses.

Run on the Jetson (ROS 2 workspace available):

  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  python3 sim/tools/fk_check.py --samples 200

It writes sim/calibration/fk_report.json. The expected residual is the small
tool-length difference between the official URDF and the FK (a few mm).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from pulse_map import pulse_to_dh_rad, pulse_to_urdf_rad, safe_pulse_bounds  # noqa: E402
from urdf_fk import UrdfArm, quat_from_matrix  # noqa: E402

ROOT = HERE.parent.parent
URDF = ROOT / "sim/urdf/jetarm_6dof.urdf"


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_angle_deg(R: np.ndarray) -> float:
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    return math.degrees(math.acos(c))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tip", default="tcp", help="URDF frame to compare (tcp = calibrated)")
    ap.add_argument("--out", default=str(ROOT / "sim/calibration/fk_report.json"))
    args = ap.parse_args()

    from kinematics.forward_kinematics import ForwardKinematics

    fk = ForwardKinematics(False)
    arm = UrdfArm(str(URDF))
    rng = np.random.default_rng(args.seed)
    bounds = safe_pulse_bounds()
    print("safe pulse bounds:", bounds)

    pos_err = []
    err_local = []
    rel_rot = []
    worst = None
    for _ in range(args.samples):
        pulses = [float(rng.integers(lo, hi + 1)) for lo, hi in bounds]
        dh = pulse_to_dh_rad(pulses)
        urdf_angles = pulse_to_urdf_rad(pulses)

        p_h, q_h = fk.get_fk(dh)
        p_h = np.asarray(p_h, dtype=float)
        R_h = quat_to_matrix(q_h.x, q_h.y, q_h.z, q_h.w)

        T = arm.fk(dict(zip([f"joint{i}" for i in range(1, 6)], urdf_angles)), tip=args.tip)
        p_u, R_u = T[:3, 3], T[:3, :3]

        e = p_u - p_h
        pos_err.append(float(np.linalg.norm(e)))
        err_local.append(R_u.T @ e)
        rel_rot.append(rot_angle_deg(R_u.T @ R_h))

        if worst is None or np.linalg.norm(e) > worst["err_m"]:
            worst = {
                "pulses": pulses,
                "err_m": float(np.linalg.norm(e)),
                "err_world_m": e.tolist(),
                "err_urdf_frame_m": (R_u.T @ e).tolist(),
                "rel_rot_deg": float(rot_angle_deg(R_u.T @ R_h)),
            }

    pos_err = np.array(pos_err)
    err_local = np.array(err_local)
    report = {
        "samples": args.samples,
        "urdf": str(URDF.relative_to(ROOT)),
        "position_error_m": {
            "mean": float(pos_err.mean()),
            "max": float(pos_err.max()),
            "std": float(pos_err.std()),
        },
        "error_in_urdf_endpoint_frame_m": {
            "mean": err_local.mean(axis=0).tolist(),
            "std": err_local.std(axis=0).tolist(),
        },
        "relative_orientation_deg": {
            "mean": float(np.mean(rel_rot)),
            "std": float(np.std(rel_rot)),
        },
        "worst": worst,
    }
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
