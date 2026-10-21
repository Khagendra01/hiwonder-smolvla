#!/usr/bin/env python3
"""SmolVLA rollout loop for the JetArm (host-native, no container).

  state  /controller_manager/servo_states
  image  /dev/video0 (Orbbec UVC color)
  policy Modal HTTP endpoint (modal/serve_smolvla.py)
  action /servo_controller (absolute pulses)

  python3 bridge/rollout_smolvla.py --url https://...modal.run --run <name> \
      --task "pick up the red cube" --duration 10
Add --dry-run to print actions without moving the arm.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import rclpy

from jetarm_ros import make_from_env
from policy_client import ModalPolicyClient
from record_smolvla import CameraReader


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=os.environ.get("JETARM_ENDPOINT", ""),
                    help="Modal endpoint base URL (or set JETARM_ENDPOINT)")
    ap.add_argument("--run", default="", help="run name under /data/outputs")
    ap.add_argument("--task", default="")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=10.0, help="seconds")
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--max-jump", type=int, default=60, help="max pulses per step")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if not args.url:
        ap.error("--url or JETARM_ENDPOINT is required")
    if not args.run:
        ap.error("--run is required (see GET /health for available runs)")

    rclpy.init()
    arm = make_from_env()
    arm.start(wait_ready_s=30.0)
    if not arm.ready:
        print("No joint state from JetArm. Is the arm stack running?", file=sys.stderr)
        arm.stop()
        return 1

    cam = CameraReader(args.camera, fps=int(args.fps))
    cam.start()
    t0 = time.monotonic()
    while cam.read() is None and time.monotonic() - t0 < 10.0:
        time.sleep(0.05)
    if cam.read() is None:
        print(f"ERROR: no frames from {args.camera}", file=sys.stderr)
        cam.stop()
        arm.stop()
        return 1

    client = ModalPolicyClient(args.url, args.run, args.task)
    last = arm.read_joint_vector()
    if last is None:
        last = np.array([500, 500, 500, 500, 500, 700], dtype=np.float32)

    period = 1.0 / args.fps
    warm_frame = cam.read()
    if warm_frame is not None:
        t_warm = time.monotonic()
        client.infer({"image": warm_frame, "state": last, "task": args.task})
        print(f"endpoint warmup: {time.monotonic() - t_warm:.1f}s")
    start = time.monotonic()
    steps = 0
    print(f"rollout: run={args.run} fps={args.fps} duration={args.duration}s dry_run={args.dry_run}")
    try:
        while time.monotonic() - start < args.duration:
            t = time.monotonic()
            state = arm.read_joint_vector()
            frame = cam.read()
            if state is None or frame is None:
                time.sleep(period)
                continue
            action = client.infer({"image": frame, "state": state, "task": args.task})
            action = np.clip(action, 0, 1000)
            action = np.clip(action, last - args.max_jump, last + args.max_jump)
            if args.dry_run:
                print("state", state.astype(int), "-> action", action.astype(int))
            else:
                arm.send_joint_vector(action, duration_s=period)
            last = action
            steps += 1
            dt = period - (time.monotonic() - t)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        arm.stop()
    print(f"rollout done: {steps} steps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
