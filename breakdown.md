# JetArm → Isaac → SmolVLA: exact build and calibration log

This is the manual for what was actually done to make the Hiwonder JetArm usable
in Isaac, and how the simulator's output feeds the existing SmolVLA training
pipeline. It records the source values, the arithmetic, the files that were
changed, and the remaining unknowns.

Current repo state: commit `890bc9a` (sim scaffold). The Hiwonder meshes are
**not committed**; they are fetched by `sim/scripts/fetch_assets.sh`.

---

## 1. What was read, and from where

### 1.1 Hiwonder kinematics (authoritative numbers)

Read from `~/ros2_ws/src/driver/kinematics/kinematics/transform.py`:

| Constant | Value | Meaning |
|---|---:|---|
| `base_link` (Tank) | `0.10314916202` m | base height, chassis type `Tank` |
| `base_link` (Slide_Rails) | `0.16314916202` m | unused here |
| `link1` | `0.12941763737` m | upper-arm length |
| `link2` | `0.12941763737` m | forearm length |
| `link3` | `0.05445583202` m | wrist-to-flange |
| `tool_link` | `0.112` m | flange-to-real-tool |
| `joint1` limits | `[-120.2, 120.2]°` | mechanical |
| `joint2` limits | `[-180.2, 0.2]°` | mechanical |
| `joint3` limits | `[-120.2, 120.2]°` | mechanical |
| `joint4` limits | `[-200.2, 20.2]°` | mechanical |
| `joint5` limits | `[-120.2, 120.2]°` | mechanical |

The file documents a **Modified DH** table (α₂ = −90°, α₅ = −90°, a₃ = link1,
a₄ = link2). The URDF is geometrically equivalent but uses a different joint
zero/orientation convention, so a pulse→URDF mapping was required.

The pulse maps in the same file are:

```python
joint1_map = [0, 1000, 500, -120,  120,   0]
joint2_map = [0, 1000, 500,   30, -210, -90]
joint3_map = [0, 1000, 500,  120, -120,   0]
joint4_map = [0, 1000, 500,   30, -210, -90]
joint5_map = [0, 1000, 500, -120,  120,   0]
# [pulse_min, pulse_max, pulse_mid, angle_at_min_deg, angle_at_max_deg, angle_at_mid_deg]
```

The forward map implemented by `angle_transform` is:

```
angle_deg = (pulse - pulse_mid) / (pulse_max - pulse_min)
            * (angle_at_max_deg - angle_at_min_deg) + angle_at_mid_deg
```

### 1.2 Compiled FK

`~/ros2_ws/src/driver/kinematics/kinematics/forward_kinematics.so` is a compiled
ARM64 Python extension (no source in the workspace). It was used as the ground
truth:

```python
from kinematics.forward_kinematics import ForwardKinematics
fk = ForwardKinematics(False)          # False = do not print the DH table
position, quaternion = fk.get_fk(dh_radians)
```

`position` is a 3-vector in meters; `quaternion` is a
`geometry_msgs/Quaternion` (x, y, z, w). The FK consumes the **DH angles**, not
pulses, so the pulse map above is applied first.

### 1.3 Official URDF / meshes

Fetched from:

```
https://github.com/Hiwonder/JetArm.git
branch: Jetson_Orin_NX_ros1
path:   jetarm/src/jetarm_6dof_simulate/jetarm_6dof_description
```

The package contains the xacro files, STL meshes, Gazebo/transmission files and
RViz config. It has **no camera link**: the Orbbec Gemini is a separate mount, so
the camera pose must be added in Isaac.

Problems in the official description:

- every arm joint is limited to `±2.1 rad`;
- inertials are placeholders: base `0.1 kg`, arm links `0.001 kg`, finger links
  `0.00001 kg`, inertia `1.0` (fingers `10`);
- the gripper is a separate xacro with a `r_joint` (0..1.57 rad) and mirrored
  `l_joint`/finger joints via `<mimic>`.

---

## 2. Step 1 — fetch the Hiwonder package

`sim/scripts/fetch_assets.sh` does a sparse, shallow clone so only the
description package is downloaded:

```bash
git clone --depth 1 --filter=blob:none --sparse \
  -b Jetson_Orin_NX_ros1 https://github.com/Hiwonder/JetArm.git "$TMP/repo"
git -C "$TMP/repo" sparse-checkout set \
  jetarm/src/jetarm_6dof_simulate/jetarm_6dof_description
cp -a "$TMP/repo/jetarm/src/jetarm_6dof_simulate/jetarm_6dof_description" \
  sim/assets/jetarm_6dof_description
```

The result is ~8.7 MB. It is git-ignored; the repository commits only the
generated plain URDF and the scripts. This keeps the repo small and avoids
redistributing Hiwonder's meshes; users run `fetch_assets.sh` once on the Isaac
machine.

## 3. Step 2 — generate a plain URDF

ROS 2's `xacro` resolves `$(find jetarm_6dof_description)` through the ament
index. To avoid installing the package into a ROS workspace, `build_urdf.sh`
creates a temporary prefix:

```bash
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
ln -s "$ASSETS" "$PREFIX/share/jetarm_6dof_description"
touch "$PREFIX/share/ament_index/resource_index/packages/jetarm_6dof_description"
AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH" \
  xacro "$ASSETS/urdf/jetarm_6dof_robot.urdf.xacro" camera_type:=GEMINI > raw.urdf
```

`camera_type:=GEMINI` is the xacro argument used by the vendor; the generated
URDF still contains no camera link, only the arm and gripper.

`postprocess_urdf.py` then:

1. rewrites `package://jetarm_6dof_description/` to
   `../assets/jetarm_6dof_description/` (paths relative to `sim/urdf/`);
2. replaces the joint limits;
3. appends the calibrated `tcp` frame;
4. writes `sim/urdf/jetarm_6dof.urdf`.

The build is reproducible: running `build_urdf.sh` twice gives byte-identical
output (verified with `diff`).

## 4. Step 3 — the URDF chain that was actually used

Extracted from the generated URDF (lengths in meters, angles in radians):

| Joint | Parent → child | origin xyz | origin rpy | axis | limit after patch |
|---|---|---|---|---|---|
| `base_joint` | `world` → `base_link` | `0 0 0` | `0 0 0` | fixed | — |
| `joint1` | `base_link` → `link1` | `0 0 0.10314916202` | `0 0 0` | `0 0 1` | `±2.0979` |
| `joint2` | `link1` → `link2` | `0 0 0` | `π/2 0 0` | `0 0 1` | `±1.5743` |
| `joint3` | `link2` → `link3` | `0 0.12941763737 0` | `0 0 0` | `0 0 1` | `±2.0979` |
| `joint4` | `link3` → `link4` | `0 0.12941763737 0` | `0 0 0` | `0 0 1` | `±1.9234` |
| `joint5` | `link4` → `link5` | `0 0.05945312631 0` | `−π/2 0 0` | `0 0 1` | `±2.0979` |
| `endpoint` | `link5` → `endpoint` | `0 0 0.11054687369` | `0 0 0` | fixed | — |
| `tcp_joint` | `endpoint` → `tcp` | `0 0 -0.00354416798` | `0 −π/2 0` | fixed | — |

The wrist/tool geometry is the source of the residual:

```
URDF:     0.05945312631 + 0.11054687369 = 0.17000000000 m
Hiwonder: 0.05445583202 + 0.11200000000 = 0.16645583202 m
difference                                 0.00354416798 m
```

The URDF splits the tool length differently from Hiwonder; the physical tool
frame is 3.544 mm closer than the URDF `endpoint`. The `tcp` frame corrects this
without moving any mesh.

## 5. Step 4 — derive the pulse → URDF mapping

For each joint, the URDF angle is an affine function of the Hiwonder DH angle:

```
u1 = +θ1
u2 = −θ2 − π/2
u3 = −θ3
u4 = −θ4 − π/2
u5 = +θ5
```

This was found by comparing the compiled FK with the URDF FK over random pulse
poses and fitting sign + 90° offsets. It is implemented in
`sim/tools/pulse_map.py` (`dh_to_urdf_rad`). The map is an **involution**: applying
it twice returns the original angle, so the same function is used in reverse
when recording simulated states as pulses.

Worked values:

| Pulse | DH angles (deg) | URDF angles (deg) |
|---|---|---|
| `[500,500,500,500,500]` | `[0, −90, 0, −90, 0]` | `[0, 0, 0, 0, 0]` |
| `[750,750,750,750,750]` | `[60, −150, −60, −150, 60]` | `[60, 60, 60, 60, 60]` |
| `[250,250,250,250,250]` | `[−60, −30, 60, −30, −60]` | `[−60, −60, −60, −60, −60]` |
| `[750,250,750,250,750]` | `[60, −30, −60, −30, 60]` | `[60, −60, 60, −60, 60]` |

The clean ±60° rows are a useful sanity check: at the mid-point all joints are
zero, and the mirrored pulse maps make a uniform 750/250 pulse map to a uniform
URDF pose.

### How it was verified

`sim/tools/fk_check.py` samples random poses inside the safe pulse bounds, then:

```python
dh = pulse_to_dh_rad(pulses)
p_h, q_h = fk.get_fk(dh)                    # Hiwonder ground truth
T = urdf_arm.fk(pulse_to_urdf_rad(pulses), tip="tcp")
p_u, R_u = T[:3, 3], T[:3, :3]
```

Before adding the `tcp` frame, the result over 300 poses was:

- position error: **3.54416798 mm**, constant and along the URDF `endpoint`
  local +z axis;
- orientation error: **90.0°**, constant (relative rotation matrix
  `[[0,0,−1],[0,1,0],[1,0,0]]` = `Ry(−90°)`).

After adding the `tcp` frame:

- mean position error `5.78e-17 m`, max `1.71e-16 m` (floating-point noise);
- mean orientation error `1.08e-7°` (quaternion round-off).

So the kinematic model is exact for practical purposes. The report is
`sim/calibration/fk_report.json`.

## 6. Step 5 — real limits and safe pulse ranges

The vendor URDF allows ±2.1 rad for every arm joint. In the URDF convention,
that is the **servo** range, not the mechanical range. The real FK clamps to the
mechanical limits in `transform.py`; converting those through the map gives:

| Joint | DH mechanical limit | URDF limit | Safe pulse range |
|---|---|---|---|
| `joint1` | `[−120.2°, 120.2°]` | `±2.0979 rad` | `[0, 1000]` |
| `joint2` | `[−180.2°, 0.2°]` | `±1.5743 rad` | `[125, 875]` |
| `joint3` | `[−120.2°, 120.2°]` | `±2.0979 rad` | `[0, 1000]` |
| `joint4` | `[−200.2°, 20.2°]` | `±1.9234 rad` | `[41, 959]` |
| `joint5` | `[−120.2°, 120.2°]` | `±2.0979 rad` | `[0, 1000]` |

Example for joint2 (DH slope = −0.24 °/pulse, mid = −90°):

```
pulse = (θ_dh_deg + 90) / (−0.24) + 500
θ_dh = −180.2° → pulse = 875.8 → 875
θ_dh =   +0.2° → pulse = 124.2 → 125
```

`postprocess_urdf.py` writes these limits into the URDF. `safe_pulse_bounds()`
in `pulse_map.py` computes the same intervals; use it when sampling sim actions
so the policy never learns poses the real arm cannot reach.

## 7. Step 6 — the `tcp` frame

Added to the URDF:

```xml
<link name="tcp" />
<joint name="tcp_joint" type="fixed">
  <origin xyz="0 0 -0.00354416798" rpy="0 -1.57079632679 0" />
  <parent link="endpoint" />
  <child link="tcp" />
</joint>
```

Why:

- `xyz.z = −0.00354416798` moves the frame back along the URDF tool axis to the
  Hiwonder tool frame;
- `rpy = (0, −90°, 0)` rotates the frame so the tool axes match (the relative
  rotation is `Ry(−90°)`).

The gripper meshes stay attached to `link5`; `tcp` is a massless marker used for
IK, rewards, and pose logging. It does not change the rendering or the gripper
joints.

## 8. What is stored, and what goes to the simulator

| File | Contents | Committed? |
|---|---|---|
| `sim/urdf/jetarm_6dof.urdf` | generated model, patched limits, `tcp` | yes |
| `sim/assets/jetarm_6dof_description/` | vendor xacro + STL meshes | no (fetched) |
| `sim/tools/pulse_map.py` | pulse/DH/URDF maps, safe bounds, inverse | yes |
| `sim/tools/urdf_fk.py` | dependency-free URDF FK | yes |
| `sim/tools/postprocess_urdf.py` | mesh paths, limits, `tcp` | yes |
| `sim/tools/fk_check.py` | FK comparison + JSON report | yes |
| `sim/calibration/fk_report.json` | 300-pose verification result | yes |
| `sim/calibration/joint_map.yaml` | all maps, limits, safe bounds, status | yes |
| `sim/calibration/camera.yaml` | camera intrinsics/extrinsics template | yes |
| `sim/calibration/dynamics.yaml` | estimated masses/actuator values | yes |
| `sim/tools/calibrate_camera.py` | checkerboard capture + calibration | yes |

### Values the simulator consumes

1. **Geometry**: the plain URDF + fetched STL meshes; lengths are already
   correct (verified FK).
2. **Joint limits**: the patched values in the URDF.
3. **Tool frame**: `tcp` for IK/rewards.
4. **Pulse maps**: `pulse_map.py` converts policy pulse actions to URDF joint
   targets, and simulated joint positions back to pulses for logging.
5. **Camera**: `camera.yaml` intrinsics + mount extrinsics. The vendor URDF has
   no camera link; add a camera prim at the measured pose.
6. **Dynamics/actuator**: `dynamics.yaml` (currently estimates) for masses,
   friction, damping, control latency.

### Action/observation convention

The real policy learns **absolute pulses** (0..1000), 6 values:
`[joint1..joint5, gripper]`. Isaac joints are in radians, so:

```python
q_target = pulse_to_urdf_rad(action_pulses[:5])   # arm
# gripper: pulse -> r_joint angle (still uncalibrated)
```

When logging a simulated step, invert:

```python
state_pulses = urdf_to_pulse(q_actual[:5]) + [gripper_pulse]
```

The inverse functions are in `pulse_map.py` and round-trip exactly
(`750 250 750 250 750` → URDF → `750 250 750 250 750`).

The real controller sends `ServosPosition` with `position_unit="pulse"` and a
`duration`; it interpolates. Sim should use a position drive at the same 20 Hz
and a comparable speed/latency, otherwise the action timing will not match.

## 9. Isaac host and what it produces

Target host: **RTX 5090, 32 GB VRAM, x86_64**. One JetArm + a 640×480 RGB camera
uses a small fraction of that; the headroom is for more parallel environments,
higher-resolution cameras, or rendering quality during domain randomization.
Start with one env, then scale. The Orin cannot run Isaac Sim.



Use the **same on-disk format as the real recorder**, so the existing Modal
converter works unchanged:

```
sim_out/
  meta.json
  episode_0000/
    states.npy      # (N, 6) float32, pulses
    actions.npy     # (N, 6) float32, pulses
    times.npy       # (N,)  float64, seconds
    images/
      000000.jpg    # 320x240 RGB JPEG, quality 85
      000001.jpg
      ...
    episode.json
  episode_0001/
  ...
```

`meta.json` must include at least:

```json
{
  "task": "pick up the red cube and place it in the bowl",
  "fps": 20,
  "joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"],
  "state_units": "pulse",
  "action_units": "pulse",
  "pulse_range": [0, 1000],
  "image": {"size": [320, 240], "format": "jpeg", "quality": 85, "color": "rgb"},
  "episodes": []
}
```

`episode.json` is informational:

```json
{"index": 0, "task": "pick up the red cube and place it in the bowl",
 "length": 200, "seconds": 9.95}
```

Export loop shape:

```python
for t in range(T):
    rgb = camera.get_rgb()                       # 320x240, RGB
    q = robot.get_joint_positions()              # radians
    state = urdf_to_pulse(q[:5]) + [gripper_pulse(q[5])]

    action = expert_or_teleop(t)                 # 6 absolute pulses
    action = clip_to_safe_bounds(action)
    robot.set_joint_targets(pulse_to_urdf_rad(action[:5]), gripper_angle(action[5]))

    images.append(jpeg(rgb, quality=85))
    states.append(state)
    actions.append(action)
    times.append(t / 20.0)
```

Recommended: 50–200 episodes, 5–15 s each, randomized object pose, camera
exposure/lighting, table texture, object size/mass/friction, and small camera
pose perturbations. Validate **one** episode through the Modal converter before
generating hundreds.

## 10. How simulated data fine-tunes SmolVLA

The training app is `modal/train_smolvla.py`. It already accepts the format
above.

### 10.1 Upload

From the RTX machine (after `modal` login), either copy the folder to the Jetson
and use `train.sh`, or upload directly:

```bash
modal volume put jetarm-data ./sim_out custom/sim_pick_cube
```

The app expects `/data/custom/<name>/meta.json` and `episode_*`.

### 10.2 Conversion to LeRobotDataset

`_build_lerobot_dataset`:

- reads `meta.json` for `task` and `fps`;
- sorts `episode_*` directories;
- reads the first JPEG to get `(width, height)`;
- declares features:
  - `observation.images.top` = `(height, width, 3)` image,
  - `observation.state` = `(6,)` float32,
  - `action` = `(6,)` float32;
- creates `LeRobotDataset` with `robot_type="jetarm"`, `use_videos=False`;
- for every aligned frame, calls `add_frame` with the image, state, action, and
  the dataset `task` string;
- calls `save_episode()` per episode and `finalize()`;
- commits the Modal volume.

All episodes in one run share the same task instruction. If you mix tasks, split
them into separate runs or extend the converter to read `episode.json`.

### 10.3 Training

The app runs `lerobot-train` on an A10G with:

```
--dataset.repo_id=local/jetarm_<name>
--dataset.root=/data/lerobot/<name>
--policy.path=lerobot/smolvla_base
--policy.device=cuda
--policy.use_amp=false
--policy.empty_cameras=2
--rename_map={"observation.images.top":"observation.images.camera1"}
--batch_size=4
--steps=<N>
--save_freq=2000
--save_checkpoint=true
--output_dir=/data/outputs/<name>
```

Key points:

- SmolVLA expects `camera1`/`camera2`/`camera3`; the converter writes `top`, and
  the rename map maps it to `camera1`. `empty_cameras=2` supplies the two unused
  camera slots.
- `use_amp=false` avoids the bf16 GradScaler failure seen on the Jetson stack.
- `smolvla_base` is ~450M parameters; the training comments configure the
  expert-only setting (vision frozen). Check the resolved policy config if you
  change the base.
- Checkpoints go to `/data/outputs/<name>/checkpoints/<step>/pretrained_model/`
  and `last/` every 2000 steps.

### 10.4 Pretrain on sim, then fine-tune on real

The intended recipe:

1. Sim pretrain: 20k–50k steps on 50–200 randomized episodes.
2. Real fine-tune: 5k–20k steps on 20–50 real episodes.
3. Evaluate on the real robot with `rollout.sh --dry-run`.

**Current code limitation**: the trainer always starts from
`lerobot/smolvla_base`. To continue from the sim checkpoint you need either:

- add a `--base` argument that sets `--policy.path=/data/outputs/<sim_run>/checkpoints/last/pretrained_model`, or
- merge the sim and real episodes into one custom dataset (same task string) and
  train once.

Neither is implemented yet; the current script does base-only training.

### 10.5 Serving and rollout

`modal/serve_smolvla.py` loads
`/data/outputs/<run>/checkpoints/last/pretrained_model`, and `/act` accepts:

```json
{"run": "<name>", "task": "...", "state": [6 pulses],
 "image": "<base64 jpeg>"}
```

and returns `{"action": [6 pulses]}`.

`bridge/rollout_smolvla.py` (host-native) reads the real state/image, posts to
the endpoint, clips the action to `[0,1000]`, applies `--max-jump` (default 60
pulses/step), and publishes `ServosPosition` at 10 Hz by default. Use
`--dry-run` first.

## 11. What is still uncalibrated

### Camera

The vendor URDF has no camera. Run on the Jetson:

```bash
python3 sim/tools/calibrate_camera.py --cols 8 --rows 6 --square 0.011
```

Use an 8×6-inner-corner checkerboard with 11 mm squares, at 640×480, with varied
positions/tilts. It writes `sim/calibration/camera.yaml` with `fx, fy, cx, cy`
and distortion. The **extrinsics** (camera pose relative to `base_link`) remain
null; measure the mount or solve with a marker, then add a camera prim in Isaac
at that pose.

### Gripper

`r_joint` is 0..1.57 rad, axis `0 -1 0`; `l_joint` and the finger joints mimic
it. The pulse→angle direction/limits are not verified. Test on the real robot:
command pulse 0, 500, 1000 and record the finger gap, then fill
`joint_map.yaml` and `dynamics.yaml`.

### Dynamics / actuator

`dynamics.yaml` contains estimates (base 0.6 kg, links 0.1–0.35 kg, etc.). The
official URDF's real values are worse: base 0.1 kg, arm links 0.001 kg,
fingers 0.00001 kg, inertia 1.0/10. For slow position-controlled pick-place,
estimates + domain randomization are usually enough; for contact-rich tasks,
weigh the links and record a step response.

### Scene and latency

Measure table height, object dimensions/mass, camera height/tilt, and the
command→state delay. Match the control rate (real recorder defaults to 20 Hz)
and the servo interpolation in sim.

## 12. Verification checklist

```bash
# 1. Assets and model
bash sim/scripts/fetch_assets.sh
bash sim/scripts/build_urdf.sh
python3 sim/tools/urdf_fk.py sim/urdf/jetarm_6dof.urdf --tip tcp

# 2. Kinematics vs the real controller
bash -c 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; \
  python3 sim/tools/fk_check.py --samples 300 --tip tcp'

# 3. Pulse round-trip
PYTHONPATH=sim/tools python3 sim/tools/pulse_map.py 750 250 750 250 750

# 4. One sim episode through the converter (on Modal)
modal run modal/train_smolvla.py --name sim_smoke --steps 2 --batch-size 2

# 5. Real dry-run
bash scripts/rollout.sh --run <run> --task "<task>" --duration 10 --dry-run
```

## 13. Repository / push status

Committed locally:

- `ad61c40` — initial record/train/rollout pipeline;
- `890bc9a` — sim scaffold, verified kinematics, calibration templates.

The Hiwonder meshes are intentionally untracked. GitHub push requires a
credential: username + email alone cannot authenticate (GitHub removed password
auth). Use an SSH key added to the account, or a repo-scoped personal access
token. Once credentials are available:

```bash
git push origin main
```
