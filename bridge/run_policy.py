#!/usr/bin/env python3
"""Client-side control loop: run a remote policy on the JetArm.

Reads joint state, sends {"state", "home"} to the policy server, applies the
returned action at a fixed rate, then returns to the home pose. Works with the
dummy policy now and SmolVLA later (same interface).
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy

from jetarm_ros import make_from_env
from policy_client import PolicyClient, parse_hostport


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="127.0.0.1:5555")
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--hold", type=float, default=0.5)
    ap.add_argument("--joint", type=int, default=0)
    a = ap.parse_args()
    dt = 1.0 / a.rate

    rclpy.init()
    arm = make_from_env()
    arm.start(wait_ready_s=10.0)
    if not arm.ready:
        print("no joint state on the servo_states topic (stack up?)")
        arm.stop()
        rclpy.shutdown()
        sys.exit(1)

    home = arm.read_joint_vector().copy()
    print("home :", " ".join(f"{n}={int(x)}" for n, x in zip(arm.jm.names, home)))

    t0 = time.monotonic()
    while time.monotonic() - t0 < a.hold:
        arm.send_joint_vector(home, duration_s=dt)
        time.sleep(dt)

    host, port = parse_hostport(a.server)
    print(f"connecting to policy server {host}:{port}")
    client = PolicyClient(host, port)

    traj = []
    print(f"running policy for {a.seconds}s at {a.rate}Hz (joint={arm.jm.names[a.joint]})")
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        state = arm.read_joint_vector()
        if state is None:
            time.sleep(dt)
            continue
        action = client.infer({"state": state, "home": home})
        arm.send_joint_vector(action, duration_s=dt)
        traj.append(action)
        time.sleep(dt)
    client.close()

    traj = np.asarray(traj)
    if len(traj):
        j = traj[:, a.joint]
        print(f"steps={len(traj)}  {arm.jm.names[a.joint]}: min={j.min():.0f} max={j.max():.0f} "
              f"span={j.max() - j.min():.0f}")

    print("returning to home")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        arm.send_joint_vector(home, duration_s=dt)
        time.sleep(dt)
    final = arm.read_joint_vector()
    print("final:", " ".join(f"{n}={int(x)}" for n, x in zip(arm.jm.names, final)))

    arm.stop()
    rclpy.shutdown()
    print("Phase 1 done.")


if __name__ == "__main__":
    main()
