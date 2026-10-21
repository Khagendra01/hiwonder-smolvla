# Reusing this pipeline for DOFBOT

Short answer: **the training and serving stack is reusable as-is; the robot I/O
layers are not.** Keep one repo, one Modal volume, and one endpoint, with a
separate run name and checkpoint per robot/task.

## What is shared

| Component | Reusable? | Notes |
|---|---|---|
| `modal/train_smolvla.py` | yes | converts the custom format and fine-tunes `lerobot/smolvla_base` |
| `modal/serve_smolvla.py` | yes | HTTP `/act` is robot-agnostic (state + image + task) |
| `bridge/policy_client.py` (`ModalPolicyClient`) | yes | pure HTTP |
| `bridge/record_smolvla.py` (`CameraReader`) | mostly | camera capture is generic OpenCV; episode/button logic can be reused |
| `bridge/rollout_smolvla.py` | no (adapt) | ROS 2 topics and pulse clamps are JetArm-specific |
| `scripts/record.sh` / `rollout.sh` | no (adapt) | Hiwonder launch/serial/`button_scan` logic |
| `config/robot.env` | no (new) | DOFBOT SDK/port/units |

The important interface is the **dataset contract**, not ROS 2.

## Dataset contract

A run is a directory with:

```
meta.json
episode_0000/
  states.npy    (N,D) float32   measured joint positions
  actions.npy   (N,D) float32   commanded joint positions
  times.npy     (N,)  float64   seconds
  images/*.jpg  one per frame
episode_0001/ ...
```

`meta.json` must contain at least `task`, `fps`, and `image.size` (the trainer
reads `task` and `fps`; it infers image size from the first JPEG). Actions must
be **absolute joint targets** (the same convention as the demonstrations), not
deltas or velocities, because the policy is trained to predict absolute targets.

For DOFBOT, record `states`/`actions` in whatever the SDK uses (angles are fine)
and keep the unit consistent across the whole dataset. Normalization is computed
from the data, so degrees and pulses both train; mixing units does not.

## What to implement

1. **Recorder** (`dofbot_record.py`): read joint positions and command targets
   from the DOFBOT SDK, grab a camera frame, write the layout above. Reuse the
   L3/R3 or keyboard episode logic from `record_smolvla.py`.
2. **Rollout** (`dofbot_rollout.py`): loop
   - read state + frame,
   - `ModalPolicyClient.infer({"image":..., "state":..., "task":...})`,
   - clamp to the robot's joint limits and per-step delta,
   - send absolute targets through the DOFBOT SDK.
3. **Teleop**: DOFBOT's own app/keyboard/gamepad; the recorder only observes.

No ROS 2 or `rclpy` is required if the DOFBOT is driven by its Python SDK.

## DOF and feature shapes

`_build_lerobot_dataset()` currently hardcodes `(6,)` for state/action to match
the JetArm. If DOFBOT is also 6-D (5 joints + gripper), no change is needed —
just use a different run name.

If it has a different DOF `D`:

- change `"shape": (6,)` to `(D,)` for `observation.state` and `action` in
  `modal/train_smolvla.py`;
- make the recorder write `(N,D)` arrays;
- make the rollout clamp use the DOFBOT limits.

SmolVLA pads state/action dimensions internally (max 32), so smaller/larger
vectors are supported, but the checkpoint is tied to the `D` it was trained on.

## Same model or separate?

Fine-tune **separate checkpoints from the same base** (`lerobot/smolvla_base`):

- JetArm: `--name jetarm_pick_red`
- DOFBOT: `--name dofbot_pick_red`

Rationale: the action semantics, kinematics, camera view, and workspace differ.
A single jointly trained policy would have to learn both embodiments and usually
performs worse. The base model is general; each fine-tune specializes it.

## Suggested first DOFBOT run

```bash
# 1) record 20-30 demos, fixed camera, consistent start pose
python3 dofbot_record.py --task "pick up the red cube"

# 2) upload + train (works unchanged)
bash scripts/train.sh datasets/dofbot_pick_red_<timestamp> 5000 4

# 3) dry-run, then live
python3 dofbot_rollout.py --run dofbot_pick_red_<timestamp> \
  --task "pick up the red cube" --duration 10 --dry-run
```

The Modal endpoint does not need to be redeployed; `--run` selects the
checkpoint. Keep the camera position and instruction text identical between
recording and rollout.
