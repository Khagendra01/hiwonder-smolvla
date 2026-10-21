#!/usr/bin/env python3
"""Checkerboard calibration for the JetArm's UVC RGB camera.

  python3 sim/tools/calibrate_camera.py --cols 8 --rows 6 --square 0.011

Hold an 8x6 (inner corners) checkerboard with 11 mm squares in front of the
camera at different positions, distances and tilts. The script auto-captures
`--samples` views and writes sim/calibration/camera.yaml. Extrinsics are left
null; measure/solve the camera mount separately.

Runs on the Jetson (no ROS needed). `--show` opens a preview window.
"""
from __future__ import annotations

import argparse
import datetime as dt
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent.parent


def detect(gray, cols: int, rows: int):
    size = (cols, rows)
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = (cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
                 | cv2.CALIB_CB_ACCURACY)
        ok, corners = cv2.findChessboardCornersSB(gray, size, flags)
        if ok:
            return True, corners.astype(np.float32)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, size, flags)
    if not ok:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--cols", type=int, default=8, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--square", type=float, default=0.011, help="square size, meters")
    ap.add_argument("--samples", type=int, default=20)
    ap.add_argument("--min-interval", type=float, default=0.7)
    ap.add_argument("--out", default=str(ROOT / "sim/calibration/camera.yaml"))
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)
    if out.exists() and not args.force:
        print(f"ERROR: {out} exists; use --force to overwrite", file=sys.stderr)
        return 1

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"ERROR: cannot open {args.device}", file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square

    obj_points, img_points, centers = [], [], []
    last_t = 0.0
    print(f"[calib] target {args.cols}x{args.rows} inner corners, "
          f"{args.square*1000:.1f} mm; need {args.samples} views")
    try:
        while len(obj_points) < args.samples:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = detect(gray, args.cols, args.rows)
            now = time.time()
            if found and now - last_t >= args.min_interval:
                center = corners.reshape(-1, 2).mean(axis=0)
                # skip near-duplicate views (same place on screen)
                if any(np.linalg.norm(center - c) < 40 for c in centers):
                    found = False
            if found:
                obj_points.append(objp)
                img_points.append(corners)
                centers.append(center)
                last_t = now
                print(f"[calib] view {len(obj_points)}/{args.samples} "
                      f"center=({center[0]:.0f},{center[1]:.0f})")
            if args.show:
                vis = cv2.drawChessboardCorners(frame, (args.cols, args.rows),
                                                corners if found else None, found)
                cv2.putText(vis, f"{len(obj_points)}/{args.samples}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("calib", vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[calib] interrupted")
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()

    if len(obj_points) < 6:
        print(f"ERROR: only {len(obj_points)} views; need at least 6", file=sys.stderr)
        return 1

    h, w = gray.shape[:2]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), None, None)
    per_view = []
    for i, (obj, img, rv, tv) in enumerate(zip(obj_points, img_points, rvecs, tvecs)):
        proj, _ = cv2.projectPoints(obj, rv, tv, K, D)
        per_view.append(float(np.sqrt(np.mean((proj - img) ** 2))))
    print(f"[calib] RMS={rms:.3f} px, max={max(per_view):.3f} px, n={len(obj_points)}")
    print(f"[calib] fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print(f"[calib] dist={D.ravel().round(5).tolist()}")

    data = {
        "device": args.device,
        "capture_resolution": [w, h],
        "training_resolution": [w // 2, h // 2],
        "intrinsics": {
            "fx": round(float(K[0, 0]), 4),
            "fy": round(float(K[1, 1]), 4),
            "cx": round(float(K[0, 2]), 4),
            "cy": round(float(K[1, 2]), 4),
            "distortion_model": "plumb_bob",
            "k1": round(float(D.ravel()[0]), 6),
            "k2": round(float(D.ravel()[1]), 6),
            "p1": round(float(D.ravel()[2]), 6),
            "p2": round(float(D.ravel()[3]), 6),
            "k3": round(float(D.ravel()[4]) if D.size > 4 else 0.0, 6),
        },
        "calibration": {
            "date": dt.date.today().isoformat(),
            "target_inner_corners": [args.cols, args.rows],
            "square_m": args.square,
            "views": len(obj_points),
            "rms_px": round(float(rms), 4),
            "max_reprojection_px": round(float(max(per_view)), 4),
        },
        # Fill these by measuring the mount or solving with a marker.
        "extrinsics": {
            "parent": "base_link",
            "child": "camera_optical_frame",
            "xyz": [None, None, None],
            "rpy": [None, None, None],
        },
        "settings": {
            "format": "MJPG",
            "fps": 30,
            "exposure": "auto",
            "white_balance": "auto",
        },
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f"[calib] wrote {out}")
    print("[calib] extrinsics are still null; measure the camera mount next.")
    return 0 if rms < 1.5 else 2


if __name__ == "__main__":
    raise SystemExit(main())
