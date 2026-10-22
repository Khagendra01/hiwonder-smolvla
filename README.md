# JetArm SmolVLA

Fine-tune a **SmolVLA** vision-language-action policy on demonstrations recorded
on a **Hiwonder JetArm**, then run it closed-loop on the robot.

- **Recording + rollout** run natively on the Jetson (ROS 2 Humble + `/dev/video0`).
- **Training + inference** run on **Modal** (A10G, native bf16).
- The same training/serving stack is reusable for a **DOFBOT** with a small
  recorder/rollout adapter — see [docs/dofbot.md](docs/dofbot.md).

> Status: the full pipeline is wired and smoke-tested end-to-end (recorder →
> upload → fine-tune → checkpoint → HTTP inference → dry-run rollout). Real
> demonstration data, real training, and live rollout are pending. A simulation
> scaffold lives in [`sim/`](sim/README.md): its kinematics are calibrated
> against the real FK; dynamics, camera and the Isaac environment are pending.

---

## Pipeline

```
 JetArm (Jetson)                     Modal cloud
 ───────────────                     ───────────
 gamepad teleop                      jetarm-data volume
   joystick_control                    ├── custom/<run>/     raw episodes
   /servo_controller (actions)         ├── lerobot/<run>/    LeRobotDataset
   /controller_manager/servo_states    └── outputs/<run>/    checkpoints
   /dev/video0 (RGB)
        │                                   ▲
        │  scripts/record.sh                │ scripts/train.sh (upload + fine-tune)
        └──────── datasets/<run>/ ──────────┘
                                            │
        ┌──────── scripts/rollout.sh ────────┘
        │            HTTP /act                serve_smolvla.py (A10G, scale-to-zero)
        ▼
   arm moves (absolute pulses)
```

---

## Quickstart

Prereqs: ROS 2 Humble arm stack works (`~/ros2_ws`), the Modal CLI is logged in,
`/dev/video0` exists, and the host Python can `import rclpy, cv2, pygame, PIL, numpy`.

```bash
cd ~/hiwonder-smolvla

# 1) Record demonstrations (L3 start/stop, R3 discard, q quit).
#    Sticks drive the chassis — use the D-pad/buttons for the arm.
bash scripts/record.sh --task "pick up the red cube" --num-episodes 30

# 2) Fine-tune on Modal. Start small (5k steps), resume if interrupted.
bash scripts/train.sh datasets/<run_name> 5000 4
#    resume: bash scripts/train.sh datasets/<run_name> 5000 4 1

# 3) Dry-run the new checkpoint, then go live (drop --dry-run).
#    The endpoint URL comes from config/local.env (JETARM_ENDPOINT).
bash scripts/rollout.sh --run <run_name> --task "pick up the red cube" --duration 10 --dry-run
```

The endpoint is already deployed; a new checkpoint is picked up by `--run`
without redeploying. Check what the server sees:

```bash
bash -c 'source scripts/env.sh && curl "$JETARM_ENDPOINT/health"'
```

The actual deployment URL is **local-only** in `config/local.env` (git-ignored);
`config/local.env.example` shows the shape. The endpoint is currently
**unauthenticated** — anyone with the URL can spend A10G seconds. Keep the URL out
of public repos, or enable Modal proxy auth before publishing one.

---

## Where the model lives

Fine-tuned weights stay in the **Modal volume** `jetarm-data`, not in this repo:

```
jetarm-data:/outputs/<run>/checkpoints/last/pretrained_model/
    model.safetensors, config.json, pre/post-processor files
```

The inference endpoint loads that path from the volume. To archive or inspect a
checkpoint locally:

```bash
bash scripts/fetch_model.sh <run>        # -> models/<run>/...
```

Model binaries are git-ignored. If you want to publish one, attach it to a
GitHub **Release** (or use Git LFS) rather than committing it.

---

## Repository layout

```
config/       robot.env (ports/topics/joints), joints.yaml
bridge/       host-native ROS 2 + gamepad + camera + dataset recorder + rollout
modal/        train_smolvla.py (data conversion + fine-tune), serve_smolvla.py
scripts/      record.sh, train.sh, rollout.sh, env.sh, fetch_model.sh, utilities
sim/          Isaac URDF/tools/calibration (data engine; see sim/README.md)
docs/         architecture, Modal, DOFBOT, troubleshooting
datasets/     recordings (git-ignored)
models/       downloaded checkpoints (git-ignored)
logs/         run logs (git-ignored)
```

Canonical entrypoints: `scripts/record.sh`, `scripts/train.sh`,
`scripts/rollout.sh`. Some older helpers (`bringup.sh`, `container.sh`,
`pull_monitor.sh`, `bridge/jetarm_robot.py`) are retained for reference; they are
not part of the current path.

---

## Simulation (Isaac)

`sim/` turns the JetArm into an Isaac data engine: generate pick-place
demonstrations, pretrain SmolVLA, then fine-tune on a small real dataset.

- The official Hiwonder URDF is fetched by `sim/scripts/fetch_assets.sh`
  (not committed) and converted to a sim-ready URDF.
- Kinematics are **verified against the real controller FK**: pulse → DH → URDF,
  plus a calibrated `tcp` frame (max error ~1e-16 m over 300 poses).
- The real recordings are **not needed** on the Isaac machine; sim episodes are
  exported in the recorder format and uploaded to the Modal volume.
- Isaac Sim/Lab needs an x86_64 RTX machine: **RTX 5090, 32 GB VRAM** (the Orin
  cannot run it).

Full setup, calibration status and data flow: [`sim/README.md`](sim/README.md).

---

## Docs

- [docs/architecture.md](docs/architecture.md) — control path, topics, dataset
  format, serving API, latency.
- [docs/modal.md](docs/modal.md) — volumes, lifecycles, the 24 h limit, resume,
  cold starts, cost, commands.
- [docs/dofbot.md](docs/dofbot.md) — what is reusable for DOFBOT and what to
  implement.
- [docs/troubleshooting.md](docs/troubleshooting.md) — serial/`button_scan`,
  camera, zsh/ROS env, stray nodes, hung `systemctl`, gamepad.
- [sim/README.md](sim/README.md) — Isaac setup, URDF/FK calibration, sim data
  flow, what is still uncalibrated.
- [breakdown.md](breakdown.md) — exact build/calibration log: vendor values,
  formulas, files, sim episode format, and the SmolVLA training path.

---

## Safety

- The policy outputs **absolute joint pulses**; `--max-jump` caps per-step motion
  (default 60/1000). Always `--dry-run` first.
- Clear the workspace and start near the pose used in the demonstrations.
- Only one process may own the arm serial port: record/rollout stop
  `button_scan.service` and restore it on exit.
- **Primary teleop is `bridge/joy_dofbot.py`** (DOFBOT-style: sticks = arm,
  D-pad = chassis) via `bash scripts/gamepad.sh`. Hiwonder's `joystick_control`
  (sticks = tank) is kept as a fallback; run only one at a time.
- This mapping is the **shared standard**: DOFBOT adapters follow the same
  stick/button semantics (`docs/dofbot.md`), so one operator can switch robots
  without relearning the pad.
- The endpoint/arm have no obstacle awareness. Rollouts are supervised.

---

## Known limitations

- Endpoint latency is ~0.47 s/step (~2 Hz); the arm moves slowly. Action-chunk
  execution is a possible optimization.
- Orbbec depth/IR is currently broken (`error -71`); only RGB (`/dev/video0`) is
  used, which is sufficient for SmolVLA.
- A 20k-step run can approach Modal's 24 h per-invocation limit; use 5k steps and
  `--resume` (`train.sh ... 1`).
- Fine-tuning from ~30 demos learns a narrow task; it will not generalize to new
  objects, poses, or lighting.
