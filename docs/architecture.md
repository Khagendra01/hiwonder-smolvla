# Architecture

## Host and robot

| Item | Value |
|---|---|
| Board | NVIDIA Jetson Orin NX 16GB, JetPack 6.0 (L4T R36.3), Ubuntu 22.04, Python 3.10 |
| Robot | Hiwonder JetArm Advanced: 5-DOF arm + gripper + tank chassis |
| ROS | ROS 2 Humble in `~/ros2_ws`, `ROS_DOMAIN_ID=31`, `rmw_fastrtps_cpp` |
| Arm board | STM32 `ros_robot_controller` on `/dev/ttyUSB0` @ 1 Mbps (CH340) |
| Camera | Orbbec Gemini RGB as UVC `/dev/video0` (depth/IR interface currently broken) |
| Input | SHANWAN gamepad `/dev/input/js0` via Hiwonder `joystick_control` |

All host-side code talks to the **existing Hiwonder ROS 2 stack**. No LeRobot or
torch is needed on the Jetson; the policy runs on Modal.

## Control path

```
gamepad ──► joystick_control ──► /servo_controller ──► ros_robot_controller ──► bus servos
                                        ▲
recorder observes state + command       │
rollout publishes policy actions ───────┘
```

| Purpose | Endpoint | Type |
|---|---|---|
| Command arm | `/servo_controller` | `servo_controller_msgs/msg/ServosPosition` (`position_unit=pulse`, `duration` s) |
| Read joints (pulse) | `/controller_manager/servo_states` | `servo_controller_msgs/msg/ServoStateList` |
| Read joints (rad) | `/controller_manager/joint_states` | `sensor_msgs/msg/JointState` |

Joint order is fixed: `[joint1, joint2, joint3, joint4, joint5, gripper]`,
servo ids `[1, 2, 3, 4, 5, 10]`, pulses `0..1000`, home `[500,500,500,500,500,700]`.
See `config/robot.env` and `config/joints.yaml`.

The arm stack used by record/rollout is the minimal
`sdk jetarm_sdk.launch.py` (controller, servo controller, kinematics, connection
check, chassis controller). The full Hiwonder bringup (app, YOLO, web video,
rosbridge) is **not** needed.

### Serial ownership

`button_scan.service` also opens `/dev/ttyUSB0` to read the physical buttons.
Only one owner is allowed. `record.sh` / `rollout.sh` stop it (passwordless sudo),
then restore it on exit. If the arm stack fails to start, check for duplicate
`ros_robot_controller` processes (`scripts/restart_robot.sh`).

## Camera

`CameraReader` (in `bridge/record_smolvla.py`) opens `/dev/video0` with
`cv2.CAP_V4L2`, requests MJPG at 640×480, keeps only the newest frame in a
background thread. Recording downscales to 320×240 JPEG q85. The depth/IR half of
the Gemini enumerates but fails (`error -71`); RGB is unaffected.

## Gamepad

The **primary / shared** operator mapping is `bridge/joy_dofbot.py`
(DOFBOT-style, below). Hiwonder's `joystick_control` is kept as a fallback. Only
one of the two may run, because both command `/servo_controller`.

### Hiwonder mapping (fallback)

Hiwonder's `joystick_control` owns the pad:

- **Sticks**: tank chassis (avoid while recording arm demos).
- **D-pad**: joints 1/2; **LB/RB**: joints 3/4; **B/X**: joint 5;
  **A/Y**: gripper close/open; **Start**: home / mode.
- **L3/R3 are no-ops** in Hiwonder's node, so the recorder reuses them for
  **start/stop** and **discard**.
- Hiwonder sends init pose `[500,560,130,115,500,200]` when it starts.

### Primary: DOFBOT-style (`bridge/joy_dofbot.py`)

This is the canonical mapping for the shared repo; DOFBOT adapters reproduce it
(see [dofbot.md](dofbot.md)). For a DOFBOT-like feel (analog sticks on the arm),
run this instead of Hiwonder's node:

| Input | Action |
|---|---|
| Left stick | joint1 (horizontal) / joint2 (vertical) |
| Right stick | joint4 (horizontal) / joint3 (vertical) |
| LB / RB | joint5 − / + |
| A / Y (cross / triangle) | gripper close / open |
| D-pad | tank chassis (forward/back + rotate) |
| Start | home |
| L3 / R3 | free for the recorder |

Mapping, step size and direction signs live in `config/gamepad.yaml`; verify the
pad's raw axis/button numbers with `python3 bridge/gamepad.py --monitor`, then
run `bash scripts/gamepad.sh` (or `--dry-run` first). Only one teleop node may
command `/servo_controller`; `gamepad.sh` stops Hiwonder's node automatically.

## Recording format

`record_smolvla.py` is **passive**: it reads joint state and the last commanded
pulses; teleop is whatever `joystick_control` publishes.

```
datasets/<task_slug>_<timestamp>/
├── meta.json
├── episode_0000/
│   ├── states.npy     (N,6) float32  measured pulses
│   ├── actions.npy    (N,6) float32  commanded pulses
│   ├── times.npy      (N,)  float64  seconds from episode start
│   ├── episode.json   {index, task, length, seconds}
│   └── images/000000.jpg ...
└── episode_0001/ ...
```

`meta.json` records task, fps (default 20), joint names, units, pulse range,
camera/image settings, and the episode list. Episodes are truncated to the
minimum of states/actions/images.

Controls: **L3** start/stop, **R3** discard; keyboard `r`/Enter, `x`, `q`.
Useful flags: `--num-episodes`, `--episode-time`, `--auto-record`, `--fps`,
`--width/--height/--jpeg-quality`, `--no-camera`, `--no-gamepad`.

## Training data conversion

`modal/train_smolvla.py` uploads the run to `jetarm-data:/custom/<run>` and builds
a `LeRobotDataset` at `/data/lerobot/<run>`:

| LeRobot key | Shape | Source |
|---|---|---|
| `observation.images.top` | (H,W,3) uint8 | `episode_*/images/*.jpg` |
| `observation.state` | (6,) float32 | `states.npy` |
| `action` | (6,) float32 | `actions.npy` |
| `task` | string | `meta.json` |

Each frame is added with the episode's `task` string. Training then renames the
camera to `observation.images.camera1` and fills two unused camera slots:

```
lerobot-train
  --dataset.repo_id=local/jetarm_<run> --dataset.root=/data/lerobot/<run>
  --policy.path=lerobot/smolvla_base --policy.device=cuda
  --policy.use_amp=false --policy.empty_cameras=2
  --rename_map={"observation.images.top":"observation.images.camera1"}
  --batch_size=4 --steps=<N> --save_freq=2000 --save_checkpoint=true
  --output_dir=/data/outputs/<run> ...
```

Notes: `use_amp=false` avoids a bf16 GradScaler `NotImplementedError`;
`output_dir` must not exist unless `--resume=true`; training is expert-only
(~100M of 450M parameters learnable).

## Serving API

`modal/serve_smolvla.py` exposes a FastAPI app on A10G. It loads
`/data/outputs/<run>/checkpoints/last/pretrained_model` lazily and caches it.

`GET /health` →
```json
{"ok": true, "runs": ["<run>", ...], "loaded": ["<run>"]}
```

`POST /act` →
```json
{
  "run": "<run>",
  "task": "pick up the red cube",
  "state": [500, 500, 500, 500, 500, 700],
  "image": "<base64 JPEG>"
}
```
response:
```json
{"action": [<6 absolute pulses>]}
```

The image is decoded to RGB, converted to a CHW float tensor in `[0,1]`, and
passed through the checkpoint's preprocessor; the postprocessor maps the policy
output back to pulse space. One real camera is filled; two are empty.

## Rollout loop

`bridge/rollout_smolvla.py`:

1. Start `JetArmROS`, wait for joint state.
2. Start `CameraReader`, wait for a frame.
3. **Warm-up** request (loads the model, pays cold start, action discarded).
4. At `--fps` (default 10): read state + newest frame → `POST /act` → clip to
   `[0,1000]` and to `last ± --max-jump` → publish to `/servo_controller`
   (unless `--dry-run`).
5. Stop the camera and arm on exit or Ctrl-C.

Measured latency: cold start ~12 s; warm inference **0.46–0.49 s** per step, so
the effective control rate is ~2 Hz regardless of `--fps`. The loop sleeps to
respect the requested period but cannot exceed the endpoint rate.
