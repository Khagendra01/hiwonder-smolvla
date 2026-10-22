#!/usr/bin/env python3
"""JetArm servo-pulse -> joint-angle mapping (mirrors Hiwonder kinematics/transform.py).

Pulse 0..1000 -> "DH" joint angle in radians, then to the URDF joint angle
convention used by sim/urdf/jetarm_6dof.urdf.

The DH->URDF affine map was verified against Hiwonder's compiled FK on the
Jetson (position error ~3.5 mm, constant along the tool axis; see
sim/calibration/fk_report.json). The residual is the URDF/FK tool-length
difference and is documented in sim/calibration/joint_map.yaml.
"""
from __future__ import annotations

import math

# [pulse_min, pulse_max, pulse_mid, angle_at_min_deg, angle_at_max_deg, angle_at_mid_deg]
JOINT_MAPS = [
    [0, 1000, 500, -120.0, 120.0, 0.0],
    [0, 1000, 500, 30.0, -210.0, -90.0],
    [0, 1000, 500, 120.0, -120.0, 0.0],
    [0, 1000, 500, 30.0, -210.0, -90.0],
    [0, 1000, 500, -120.0, 120.0, 0.0],
]

# Mechanical joint limits used by the real FK (degrees).
FK_LIMITS_DEG = [
    (-120.2, 120.2),
    (-180.2, 0.2),
    (-120.2, 120.2),
    (-200.2, 20.2),
    (-120.2, 120.2),
]


def pulse_to_dh_deg(pulse: list[float]) -> list[float]:
    out = []
    for p, m in zip(pulse, JOINT_MAPS):
        lo, hi, mid, a_lo, a_hi, a_mid = m
        out.append((p - mid) / (hi - lo) * (a_hi - a_lo) + a_mid)
    return out


def pulse_to_dh_rad(pulse: list[float]) -> list[float]:
    return [math.radians(a) for a in pulse_to_dh_deg(pulse)]


def dh_to_urdf_rad(dh: list[float]) -> list[float]:
    """Map Hiwonder DH angles to the URDF joint convention (verified)."""
    return [
        dh[0],
        -dh[1] - math.pi / 2,
        -dh[2],
        -dh[3] - math.pi / 2,
        dh[4],
    ]


def pulse_to_urdf_rad(pulse: list[float]) -> list[float]:
    return dh_to_urdf_rad(pulse_to_dh_rad(pulse))


def dh_deg_to_pulse(dh_deg: list[float]) -> list[float]:
    """Inverse of pulse_to_dh_deg (used when logging sim states as pulses)."""
    out = []
    for a, m in zip(dh_deg, JOINT_MAPS):
        lo, hi, mid, a_lo, a_hi, a_mid = m
        out.append((a - a_mid) / (a_hi - a_lo) * (hi - lo) + mid)
    return out


def dh_rad_to_pulse(dh_rad: list[float]) -> list[float]:
    return dh_deg_to_pulse([math.degrees(a) for a in dh_rad])


def urdf_to_dh_rad(urdf_rad: list[float]) -> list[float]:
    """Inverse of dh_to_urdf_rad. The affine map is an involution."""
    return dh_to_urdf_rad(urdf_rad)


def urdf_to_pulse(urdf_rad: list[float]) -> list[float]:
    return dh_rad_to_pulse(urdf_to_dh_rad(urdf_rad))


def safe_pulse_bounds() -> list[tuple[int, int]]:
    """Pulse intervals that stay inside the real FK mechanical limits."""
    bounds = []
    for m, (lo_deg, hi_deg) in zip(JOINT_MAPS, FK_LIMITS_DEG):
        _, _, mid, a_lo, a_hi, a_mid = m
        # angle(p) = (p-mid)/(hi-lo)*(a_hi-a_lo) + a_mid
        slope = (a_hi - a_lo) / (m[1] - m[0])
        p_lo = (lo_deg - a_mid) / slope + mid
        p_hi = (hi_deg - a_mid) / slope + mid
        lo, hi = sorted((p_lo, p_hi))
        bounds.append((max(0, int(math.ceil(lo))), min(1000, int(math.floor(hi)))))
    return bounds


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("pulses", nargs=5, type=float)
    args = ap.parse_args()
    dh = pulse_to_dh_rad(args.pulses)
    urdf = pulse_to_urdf_rad(args.pulses)
    back = urdf_to_pulse(urdf)
    print("dh_rad  :", [round(v, 5) for v in dh])
    print("urdf_rad:", [round(v, 5) for v in urdf])
    print("roundtrip pulses:", [round(v, 2) for v in back])
    print("safe pulse bounds:", safe_pulse_bounds())
