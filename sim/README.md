# Isaac simulation

Purpose: use Isaac as a **data engine** for SmolVLA — generate diverse pick-place
demonstrations (and optionally RL experts) cheaply, pretrain on them, then
fine-tune on a small real dataset. Sim does not replace the real fine-tune for a
camera-based policy; it reduces how many real demos you need.

## Status

| Piece | State |
|---|---|
| Official URDF/meshes | fetched from Hiwonder (`sim/scripts/fetch_assets.sh`) |
| Plain sim URDF | generated, `sim/urdf/jetarm_6dof.urdf` |
| Pulse → joint mapping | **verified** against Hiwonder's compiled FK |
| Joint limits | applied in URDF (mechanical limits) |
| `tcp` frame | added; matches Hiwonder FK to floating point |
| Inertials / actuator | **not calibrated** (placeholders + estimates) |
| Camera intrinsics/extrinsics | **not calibrated** |
| Gripper pulse → angle | **not calibrated** |
| Isaac Lab environment | not built yet |

## Quick start (Jetson, no Isaac needed)

```bash
bash sim/scripts/fetch_assets.sh            # Hiwonder URDF + meshes (not committed)
bash sim/scripts/build_urdf.sh              # optional: regenerate the plain URDF
python3 sim/tools/urdf_fk.py sim/urdf/jetarm_6dof.urdf --tip tcp

# verify against the real controller's FK (needs ROS 2 + ~/ros2_ws)
bash -c 'source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; \
  python3 sim/tools/fk_check.py --samples 300 --tip tcp'
```

`sim/calibration/fk_report.json` is the current result: **max position error
1.7e-16 m, orientation 0°** over 300 random poses (using the `tcp` frame).

## What was calibrated

Mapping (see `sim/calibration/joint_map.yaml`):

```
pulse --(Hiwonder pulse maps)--> DH angle --(affine)--> URDF joint angle
joint1:  urdf = +dh
joint2:  urdf = -dh - 90°
joint3:  urdf = -dh
joint4:  urdf = -dh - 90°
joint5:  urdf = +dh
```

Safe pulse bounds (mechanical): `j2 125..875`, `j4 41..959`, others `0..1000`.
The URDF is patched to the real mechanical limits; the official URDF had all
joints at ±2.1 rad and placeholder inertials.

TCP correction baked into the URDF as a fixed `tcp` child of `endpoint`:

```
xyz = (0, 0, -0.00354416798) m
rpy = (0, -90°, 0)
```

## What still needs calibration

1. **Dynamics** — link masses/inertias/COM, joint friction/damping, servo
   torque/speed. Start with `sim/calibration/dynamics.yaml` estimates; measure
   with a scale and a step-response log.
2. **Gripper** — pulse → finger angle, open/close direction, max opening, force.
3. **Camera** — intrinsics/distortion and the mount extrinsics relative to
   `base_link`. This is the largest sim-to-real factor for a vision policy.
4. **Actuator latency** — measure command→state delay; match it in sim.
5. **Scene** — table height, object size/mass/friction, lighting/background.

## Isaac setup on the RTX machine

The Orin cannot run Isaac Sim. Use the RTX box (24 GB is comfortable for one arm
+ camera; start at 640×480, one environment).

1. **OS/driver.** Ubuntu 22.04/24.04, recent NVIDIA driver with **RTX 50-series
   (Blackwell)** support (570+), CUDA 12.8+. Verify `nvidia-smi`.
2. **Docker + NVIDIA Container Toolkit.**
   ```bash
   sudo apt install -y docker.io
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   docker run --rm --gpus all ubuntu nvidia-smi
   ```
3. **Isaac Sim + Isaac Lab.** Use an Isaac Sim **5.x** release that lists
   RTX 50-series support, and the matching Isaac Lab 2.x. The official container
   is `nvcr.io/nvidia/isaac-sim:<tag>`; mount the repo and run headless for
   training.
4. **Get this repo + assets** on the RTX machine:
   ```bash
   git clone <repo> && cd hiwonder-smolvla
   bash sim/scripts/fetch_assets.sh
   ```
5. **URDF → USD.** Use Isaac Sim's URDF importer or Isaac Lab's
   `convert_urdf.py` on `sim/urdf/jetarm_6dof.urdf`. Fix the base to the world
   (or to the tank chassis if you simulate the mobile base), then import the
   camera and table.
6. **Environment.** One articulation, 6-D action = absolute joint targets
   (pulses or radians via `sim/tools/pulse_map.py`), 20 Hz control, RGB camera
   at the real pose/resolution, table + cube. Use the safe pulse bounds when
   sampling actions.

### Data flow (important)

Real recordings are **not** needed to set up Isaac. The sim machine needs the
robot model + calibration constants; the demonstrations are for training.

```
Jetson:  record.sh  -> datasets/<real_run>          (stays local)
Modal:   train.sh   -> jetarm-data:/custom/<real_run>
RTX:     Isaac Lab  -> sim episodes (same format)   -> modal volume put custom/sim_<task>
Modal:   train_smolvla.py --name sim_<task>         (pretrain)
         then fine-tune on <real_run>               (or merge sim+real episodes)
Jetson:  rollout.sh --run <final_run>
```

Copy to the RTX machine:
- this repo (`sim/` + tools),
- `sim/calibration/camera.yaml` after calibration,
- optionally 5–10 real frames (not the dataset) to compare camera view/lighting,
- optionally a short real step-response log for actuator latency.

Do **not** copy `datasets/` or model weights to the RTX machine.

## Next steps

1. Calibrate the camera (`sim/tools/calibrate_camera.py` once written, or ROS
   `camera_calibration`) and fill `sim/calibration/camera.yaml`.
2. Build the Isaac Lab env and export one scripted pick-place episode in the
   recorder format; verify `modal/train_smolvla.py` converts it.
3. Add domain randomization (camera pose ±, lighting, textures, object pose,
   mass/friction, latency).
4. Pretrain on sim, fine-tune on real, evaluate with `rollout.sh --dry-run`.
