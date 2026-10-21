#!/usr/bin/env python3
"""Keyboard teleop for the JetArm (for collecting demonstrations / testing).

The JetArm has no leader arm, so this provides a minimal joint-space teleop:
  keys 1..6   select the active joint (1..5 = joints, 6 = gripper)
  + / -       nudge the active joint (+/- step pulses)
  [ / ]       decrease / increase step size
  h           go to home pose (all 500, gripper 700)
  q           quit

Publishes absolute pulses to /servo_controller via JetArmROS.

Run inside the ROS2 environment:
  source scripts/env.sh
  python3 bridge/teleop_keyboard.py
"""
import sys
import termios
import tty

import rclpy

from jetarm_ros import JointMap, make_from_env

HOME = [500, 500, 500, 500, 500, 700]


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    arm = make_from_env()
    arm.start(wait_ready_s=10.0)
    if not arm.ready:
        print("No joint state from JetArm. Is the arm stack running (scripts/bringup.sh)?")
        arm.stop()
        rclpy.shutdown()
        sys.exit(1)

    pose = arm.read_joint_vector().astype(int).tolist()
    active = 0
    step = 20

    print("active joint: 1  |  step: 20")
    print("keys: 1-6 select | +/- nudge | [ ] step | h home | q quit")
    try:
        while True:
            k = read_key()
            if k in "123456":
                active = int(k) - 1
            elif k == "+":
                pose[active] = min(1000, pose[active] + step)
                arm.send_joint_vector(pose, duration_s=0.15)
            elif k == "-":
                pose[active] = max(0, pose[active] - step)
                arm.send_joint_vector(pose, duration_s=0.15)
            elif k == "[":
                step = max(1, step - 5)
            elif k == "]":
                step = min(200, step + 5)
            elif k == "h":
                pose = list(HOME)
                arm.send_joint_vector(pose, duration_s=0.6)
            elif k == "q":
                break
            sys.stdout.write(
                f"\ractive joint: {active + 1}  step: {step:3d}  pose: "
                + " ".join(f"{p:4d}" for p in pose)
                + "   "
            )
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        arm.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
