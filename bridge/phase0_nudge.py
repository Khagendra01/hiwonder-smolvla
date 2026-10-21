#!/usr/bin/env python3
"""Phase 0: safe single-joint nudge test (read -> nudge -> read -> return -> read).

No calibration. Commands absolute pulses relative to the CURRENT pose only.
Defaults to joint1 (base yaw) with a tiny, slow delta.

  python3 bridge/phase0_nudge.py [joint_index] [delta_pulses] [duration_s]

joint_index: 0..5 (0=joint1, 5=gripper). Default 0.
"""
import sys
import time

import rclpy

from jetarm_ros import make_from_env


def show(arm, tag):
    v = arm.read_joint_vector()
    if v is None:
        print(f"  {tag}: <no state>")
        return None
    print(f"  {tag}: " + " ".join(f"{n}={int(x):4d}" for n, x in zip(arm.jm.names, v)))
    return v


def main():
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    delta = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 0.8

    rclpy.init()
    arm = make_from_env()
    arm.start(wait_ready_s=10.0)
    if not arm.ready:
        print("No joint state on /controller_manager/servo_states (stack up?)")
        arm.stop()
        rclpy.shutdown()
        sys.exit(1)

    print(f"NUDGE test: joint={arm.jm.names[idx]} delta={delta} pulses duration={dur}s")
    before = show(arm, "before ")
    if before is None:
        arm.stop()
        rclpy.shutdown()
        sys.exit(1)

    target = before.copy()
    target[idx] = max(0, min(1000, before[idx] + delta))
    print(f"  -> moving {arm.jm.names[idx]} {int(before[idx])} -> {int(target[idx])} over {dur}s")
    arm.send_joint_vector(target, duration_s=dur)
    time.sleep(dur + 0.6)
    show(arm, "nudged ")

    print("  -> returning to original")
    arm.send_joint_vector(before, duration_s=dur)
    time.sleep(dur + 0.6)
    show(arm, "return ")

    arm.stop()
    rclpy.shutdown()
    print("Phase 0 done. Arm back at original pose.")


if __name__ == "__main__":
    main()
