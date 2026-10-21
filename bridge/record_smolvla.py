#!/usr/bin/env python3
"""Record JetArm demonstrations on the host (passive; no LeRobot / torch needed).

Teleop is done by the Hiwonder ``joystick_control`` node (your gamepad). This
recorder only *observes* the running stack:

  state   /controller_manager/servo_states   joint feedback (pulses 0..1000)
  action  /servo_controller                  commanded pulses (from the gamepad)
  image   /dev/video0 (Orbbec UVC color)     read directly with OpenCV

Episode control (nothing conflicts with Hiwonder's buttons):
  gamepad L3 (13)   start / stop episode
  gamepad R3 (14)   discard the current episode
  keyboard          r or Enter = record, x = discard, q = quit

Output layout:
  <out>/meta.json
  <out>/episode_0000/states.npy    (N,6) float32
  <out>/episode_0000/actions.npy   (N,6) float32
  <out>/episode_0000/times.npy     (N,)  float64
  <out>/episode_0000/images/000000.jpg ...

    bash scripts/record.sh --task "pick up the red cube" --num-episodes 50
"""
from __future__ import annotations

import argparse
import io
import json
import os
import select
import shutil
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import cv2
import numpy as np
import rclpy
from PIL import Image

from gamepad import Gamepad
from jetarm_ros import make_from_env


class CameraReader(threading.Thread):
    """Continuously grab the newest V4L2 frame in the background."""

    def __init__(self, device: str, width: int = 640, height: int = 480, fps: int = 30):
        super().__init__(daemon=True)
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.ok = False
        self._frame = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()

    def run(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ok = True
        while not self._stopping.is_set():
            got, frame = cap.read()
            if got and frame is not None:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.01)
        cap.release()

    def read(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self):
        self._stopping.set()
        if self.is_alive():
            self.join(timeout=2.0)


class KeyPoller:
    """Non-blocking single-key reader (cbreak keeps Ctrl+C working)."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.enabled = sys.stdin.isatty()
        self.old = None

    def __enter__(self):
        if self.enabled:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def poll(self):
        if not self.enabled:
            return None
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if ready else None

    def __exit__(self, *exc):
        if self.enabled and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def slug(text: str) -> str:
    out = "".join(c.lower() if c.isalnum() else "_" for c in text).strip("_")
    while "__" in out:
        out = out.replace("__", "_")
    return out[:60] or "dataset"


def encode_jpeg(rgb: np.ndarray, size: tuple[int, int], quality: int) -> bytes:
    im = Image.fromarray(np.ascontiguousarray(rgb))
    if im.mode != "RGB":
        im = im.convert("RGB")
    if im.size != size:
        im = im.resize(size, Image.BILINEAR)
    buf = io.BytesIO()
    im.save(buf, format="JPEG", quality=int(quality))
    return buf.getvalue()


def write_episode(out: Path, idx: int, task: str, states, actions, times, frames) -> None:
    epdir = out / f"episode_{idx:04d}"
    if epdir.exists():
        shutil.rmtree(epdir)
    (epdir / "images").mkdir(parents=True, exist_ok=True)
    np.save(epdir / "states.npy", np.asarray(states, dtype=np.float32))
    np.save(epdir / "actions.npy", np.asarray(actions, dtype=np.float32))
    np.save(epdir / "times.npy", np.asarray(times, dtype=np.float64))
    for k, data in enumerate(frames):
        (epdir / "images" / f"{k:06d}.jpg").write_bytes(data)
    (epdir / "episode.json").write_text(
        json.dumps(
            {
                "index": idx,
                "task": task,
                "length": len(states),
                "seconds": float(times[-1]) if times else 0.0,
            },
            indent=2,
        )
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=None, help="output directory (default: datasets/<task>_<timestamp>)")
    ap.add_argument("--task", required=True, help="language instruction for every episode")
    ap.add_argument("--num-episodes", type=int, default=50, help="0 = record until you quit")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--episode-time", type=float, default=0.0, help="auto-stop after N seconds (0 = manual)")
    ap.add_argument("--auto-record", type=float, default=0.0, help="start recording at once for N seconds")
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument("--jpeg-quality", type=int, default=85)
    ap.add_argument("--camera", default="/dev/video0", help="V4L2 color device")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--record-button", type=int, default=13, help="gamepad L3")
    ap.add_argument("--discard-button", type=int, default=14, help="gamepad R3")
    ap.add_argument("--no-gamepad", action="store_true")
    args = ap.parse_args()

    if not args.no_camera and not os.path.exists(args.camera):
        print(f"ERROR: {args.camera} not found. Run: bash scripts/fix_camera.sh", file=sys.stderr)
        return 1

    root = Path(__file__).resolve().parent.parent
    out = Path(args.out) if args.out else root / "datasets" / f"{slug(args.task)}_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    arm = make_from_env()
    arm.start(wait_ready_s=15.0)
    if not arm.ready:
        print("No joint state from JetArm. Is the arm stack running?", file=sys.stderr)
        arm.stop()
        rclpy.shutdown()
        return 1

    cam = None
    if not args.no_camera:
        cam = CameraReader(args.camera)
        cam.start()
        t0 = time.monotonic()
        while cam.read() is None and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        if cam.read() is None:
            print(f"ERROR: no frames from {args.camera}", file=sys.stderr)
            cam.stop()
            arm.stop()
            rclpy.shutdown()
            return 1

    gp = None
    if not args.no_gamepad:
        try:
            gp = Gamepad(0, 0.0)
        except Exception as exc:
            print(f"warning: gamepad unavailable ({exc}); use the keyboard")

    meta = {
        "task": args.task,
        "fps": args.fps,
        "joint_names": list(arm.jm.names),
        "state_units": "pulse",
        "action_units": "pulse",
        "pulse_range": [0, 1000],
        "camera": None if args.no_camera else {"device": args.camera, "source": "orbbec_uvc"},
        "image": None
        if args.no_camera
        else {"size": [args.width, args.height], "format": "jpeg", "quality": args.jpeg_quality, "color": "rgb"},
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "episodes": [],
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))

    period = 1.0 / args.fps
    st = {"recording": False, "ep": 0, "t0": 0.0, "states": [], "actions": [], "times": [], "frames": []}

    def finish(reason: str = ""):
        if not st["recording"]:
            return
        st["recording"] = False
        n = len(st["states"])
        if n < 2:
            print("\n[rec] episode discarded (too short)")
        else:
            idx = st["ep"]
            write_episode(out, idx, args.task, st["states"], st["actions"], st["times"], st["frames"])
            meta["episodes"].append(
                {"index": idx, "length": n, "seconds": float(st["times"][-1]), "task": args.task}
            )
            (out / "meta.json").write_text(json.dumps(meta, indent=2))
            print(f"\n[rec] saved episode {idx:04d}: {n} frames, {st['times'][-1]:.1f}s {reason}")
            st["ep"] += 1
        st["states"], st["actions"], st["times"], st["frames"] = [], [], [], []

    def start():
        st["recording"] = True
        st["t0"] = time.monotonic()
        st["states"], st["actions"], st["times"], st["frames"] = [], [], [], []
        print(f"\n[rec] episode {st['ep']:04d} recording...")

    def discard():
        if st["recording"]:
            st["recording"] = False
            st["states"], st["actions"], st["times"], st["frames"] = [], [], [], []
            print("\n[rec] episode discarded")

    print(f"[rec] output: {out}")
    print("[rec] teleop: Hiwonder joystick_control (gamepad)")
    print("[rec] gamepad: L3=record  R3=discard   keyboard: r/Enter=record  x=discard  q=quit")

    if args.auto_record > 0:
        args.episode_time = args.auto_record
        print(f"[rec] auto-record for {args.auto_record:.1f}s")

    last_status = 0.0
    try:
        with KeyPoller() as keys:
            while True:
                now = time.monotonic()
                actions = []

                if gp is not None:
                    gp.poll()
                    for btn in gp.pressed_edges():
                        if btn == args.record_button:
                            actions.append("record")
                        elif btn == args.discard_button:
                            actions.append("discard")

                key = keys.poll()
                if key is not None:
                    if key == "q":
                        actions.append("quit")
                    elif key in ("r", "\r", "\n", " "):
                        actions.append("record")
                    elif key == "x":
                        actions.append("discard")

                if args.auto_record > 0 and not st["recording"] and st["ep"] == 0:
                    start()

                for act in actions:
                    if act == "quit":
                        finish("(quit)")
                        return 0
                    if act == "record":
                        if st["recording"]:
                            finish()
                        elif args.num_episodes and st["ep"] >= args.num_episodes:
                            print(f"\n[rec] already recorded {args.num_episodes} episodes")
                        else:
                            start()
                    elif act == "discard":
                        discard()

                if st["recording"]:
                    if args.episode_time > 0 and now - st["t0"] >= args.episode_time:
                        finish("(time)")
                    else:
                        state = arm.read_joint_vector()
                        action = arm.read_last_command()
                        img = cam.read() if cam is not None else None
                        if state is not None and action is not None and (cam is None or img is not None):
                            if img is not None:
                                st["frames"].append(
                                    encode_jpeg(img[..., ::-1], (args.width, args.height), args.jpeg_quality)
                                )
                            st["states"].append(state)
                            st["actions"].append(action)
                            st["times"].append(now - st["t0"])

                if now - last_status >= 0.2:
                    last_status = now
                    tag = "REC" if st["recording"] else "---"
                    total = args.num_episodes or "inf"
                    live = "ok" if arm.read_last_command() is not None else "waiting"
                    sys.stdout.write(
                        f"\r[{tag}] ep {st['ep']}/{total}  frames {len(st['states']):4d}  cmd {live}   "
                    )
                    sys.stdout.flush()

                if args.num_episodes and st["ep"] >= args.num_episodes and not st["recording"]:
                    print(f"\n[rec] done: {st['ep']} episodes -> {out}")
                    return 0

                delay = period - (time.monotonic() - now)
                if delay > 0:
                    time.sleep(delay)
    except KeyboardInterrupt:
        finish("(interrupt)")
        return 130
    finally:
        print("\n[rec] shutting down...", flush=True)
        if cam is not None:
            cam.stop()
        if gp is not None:
            gp.close()
        arm.stop()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("[rec] shutdown complete", flush=True)


if __name__ == "__main__":
    _rc = main()
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    finally:
        os._exit(_rc)
