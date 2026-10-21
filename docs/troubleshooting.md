# Troubleshooting

## Environment

**`source scripts/env.sh` breaks under zsh.**
ROS setup scripts use `${BASH_SOURCE[0]}`; under zsh it is empty, which produces
errors like `can't open file '.../_local_setup_util_sh.py'` and a wrong
`COLCON_CURRENT_PREFIX`. Run the shell scripts (they are bash) or wrap ad-hoc
commands:

```bash
bash -c 'source scripts/env.sh && ros2 topic list'
```

**Host imports.** The recorder/rollout need `rclpy, cv2, pygame, PIL, numpy`:
`python3 -c "import rclpy, cv2, pygame, PIL, numpy"`.

## Arm stack / serial

**Only one process may own `/dev/ttyUSB0`.** `button_scan.service` also opens it.
Symptoms: arm stack starts but no `/controller_manager/servo_states`, or
`ros_robot_controller` exits immediately.

```bash
timeout 5 systemctl is-active button_scan.service
timeout 10 sudo -n systemctl stop button_scan.service     # free the port
timeout 10 sudo -n systemctl start button_scan.service    # restore afterwards
sudo fuser -v /dev/ttyUSB0
pgrep -af "[r]os_robot_controller"
```

`record.sh` and `rollout.sh` already stop/restore it. If there are **two**
`ros_robot_controller` processes, run `bash scripts/restart_robot.sh`.

**Hung `systemctl` / D-Bus.** The shell can stall on `systemctl` even when the
service is fine. Always bound it (`timeout 5 systemctl ...`) and check state with
`pgrep`/`ps` rather than `ros2 topic info`, which can also hang.

**Kill patterns.** `pkill -f foo` matches its own command line in some shells.
Use bracket patterns: `pkill -f "[r]os_robot_controller"`,
`pkill -f "component_container.*camera_container"`.

**Long-running processes.** For a job that must outlive your shell, detach it:

```bash
setsid ros2 launch sdk jetarm_sdk.launch.py > logs/bringup.log 2>&1 < /dev/null &
```

Then poll the log; do not rely on the launching terminal.

## Camera

**`/dev/video0` missing.** Re-enumerate the Orbbec UVC interface:

```bash
bash scripts/fix_camera.sh
```

If it still fails, reseat the camera USB cable. The depth/IR vendor interface
(`2bc5:0614`) currently errors with `can't set config #1, error -71` and is not
needed; RGB (`2bc5:0511`) is what the pipeline uses.

**Camera busy / black frames.** The full Hiwonder bringup (or another recorder)
may own the device. Stop it, or use `--camera /dev/videoN`. A full bringup
(`bringup.launch.py`) also starts app nodes (YOLO, web video, rosbridge) that are
unnecessary and consume CPU; use the minimal `sdk jetarm_sdk.launch.py`.

## Recording

- **No episodes saved:** episodes shorter than a few frames are dropped; hold the
  pose for a moment after L3 stop.
- **States/actions/image counts differ:** the serializer truncates to the shortest
  of the three; a slow camera or dropped RCLPY callback causes this. Check the
  warning in `record_smolvla.py` output.
- **Gamepad not found:** check `/dev/input/js0`, permissions, and that no other
  process grabbed the pad. `SDL_VIDEODRIVER=dummy` is set for headless use.
- **L3/R3 do nothing in teleop:** expected — Hiwonder's `joystick_control` does not
  implement L3/R3. The recorder uses them for start/stop and discard.
- **Arm pose:** Hiwonder's node sends `[500,560,130,115,500,200]` at startup; wait
  for that motion to finish before recording.
- **Sticks move the chassis:** that is Hiwonder's mapping. Use the D-pad/buttons
  for the arm and keep the chassis still.

## Training / Modal

| Symptom | Fix |
|---|---|
| `output_dir ... already exists` | resume (`train.sh ... 1`) or delete the run's `outputs/` dir |
| `NotImplementedError ... 'BFloat16'` | `--policy.use_amp=false` (already set) |
| `no episodes under ...` | upload contains `episode_*` dirs with `images/*.jpg` |
| `custom/<name> not found` | upload landed under a different basename; check `volume ls custom` |
| run not in `/health` | training not finished, or wrong `--run` name |
| cold start ~12 s | expected; the rollout warms up before timing |
| training killed at 24 h | resume with `--resume`; consider 5k-step runs |

Inspect with:

```bash
modal --profile=tobiwarwager volume ls jetarm-data custom
modal --profile=tobiwarwager volume ls jetarm-data outputs
modal --profile=tobiwarwager volume ls jetarm-data outputs/<run>/checkpoints
```

## Rollout

- Always start with `--dry-run`; it exercises the camera, ROS state, and endpoint
  without moving the arm.
- `No joint state from JetArm` → arm stack not running or serial conflict.
- `no frames from /dev/video0` → camera problem above.
- Motion too abrupt → lower `--max-jump` (default 60 pulses/step).
- Motion too slow → endpoint latency dominates (~0.47 s/step). Increasing `--fps`
  above ~2 does not increase the real control rate.
- Start the arm near the pose used in the demonstrations; the policy is not
  robust to arbitrary initial poses.

## Storage

The NVMe reports media errors (`media_errors=290`). Replacement is deferred; no
corruption observed. Avoid long, write-heavy jobs on the root disk until it is
replaced, and keep datasets/checkpoints in the Modal volume.
