# Modal: training, serving, lifecycles

## Apps and volumes

| Name | Kind | Purpose |
|---|---|---|
| `jetarm-smolvla` | app | training + checkpoint check (`modal/train_smolvla.py`) |
| `jetarm-smolvla-serve` | app | HTTP policy endpoint (`modal/serve_smolvla.py`) |
| `jetarm-data` | volume | datasets, converted LeRobotDataset, checkpoints |
| `jetarm-hf-cache` | volume | Hugging Face cache (base model, tokenizers) |

Volume layout:

```
/data/
├── custom/<run>/                 raw recorder output (meta.json + episode_*)
├── lerobot/<run>/                converted LeRobotDataset
└── outputs/<run>/
    ├── checkpoints/0002000/pretrained_model/
    ├── checkpoints/.../pretrained_model/
    └── checkpoints/last/pretrained_model/   <- endpoint loads this
```

The endpoint reads `outputs/<run>/checkpoints/last/pretrained_model`; you do not
need to download weights to run inference.

## The three lifecycles

**1. Training — one batch job.**
`modal run` starts an A10G container and runs one function invocation. Modal's
maximum function timeout is **24 h**; `train_smolvla.py` sets `timeout=86400`.
Checkpoints are written every **2000 steps** and at the end. If a run is
interrupted (timeout, preemption, Ctrl-C), the volume keeps the latest checkpoint
and `--resume` continues from it.

**2. Idle — nothing runs.**
A serverless web endpoint scales to zero when no requests arrive
(`scaledown_window=600`, i.e. after 10 idle minutes). There is no per-day hosting
limit; idle cost is storage only.

**3. Rollout — on demand.**
The first request cold-starts a container (~12 s measured), loads the checkpoint
from the volume, then serves at **~0.47 s/step** while requests continue. After
the rollout the container scales back down. Rollout script performs a warm-up
request before its timed loop.

The 24 h limit therefore applies to a **single training invocation**, not to
model storage and not to how long the endpoint can exist.

## Checkpointing and resume

```bash
# start a run
bash scripts/train.sh datasets/<run> 5000 4

# resume an interrupted run from its last checkpoint
bash scripts/train.sh datasets/<run> 5000 4 1
```

`train.sh` re-uploads the (unchanged) dataset, then passes `--resume=true` to
LeRobot. Do not delete `outputs/<run>` if you intend to resume.

A practical plan: 5k steps first (a few hours on A10G), evaluate with a dry-run,
then continue with more steps. A 20k-step run may approach 24 h; split/resume.

## Cost (approximate)

- A10G: about **$1.10/hour** while a container is alive.
- Idle endpoint: **$0** (scaled to zero).
- Volume storage: a few GB; negligible.
- A 5k-step run is typically a few dollars.

## Commands

```bash
MODAL=~/.local/bin/modal
PROFILE=tobiwarwager

# train (train.sh wraps upload + run)
$MODAL --profile=$PROFILE run modal/train_smolvla.py --name <run> --steps 5000 --batch-size 4
$MODAL --profile=$PROFILE run modal/train_smolvla.py --name <run> --steps 5000 --batch-size 4 --resume

# deploy the endpoint (only needed after changing serve_smolvla.py)
$MODAL --profile=$PROFILE deploy modal/serve_smolvla.py

# inspect / fetch / delete
$MODAL --profile=$PROFILE volume ls jetarm-data outputs
$MODAL --profile=$PROFILE volume ls jetarm-data outputs/<run>/checkpoints
bash scripts/fetch_model.sh <run>
$MODAL --profile=$PROFILE volume rm -r jetarm-data outputs/smoke
```

`train.sh` uses `modal volume put jetarm-data "$D" custom/ --force`, which lands
at `custom/<basename-of-$D>/`. It removes an existing `custom/<name>` first so
re-uploads cannot mix episode sets.

## HTTP API

```
GET  /health                      -> {"ok":true,"runs":[...],"loaded":[...]}
POST /act
  {"run":"<run>", "task":"...", "state":[6 floats], "image":"<base64 jpeg>"}
  -> {"action":[6 pulses]}
```

The endpoint caches loaded policies per container and calls `DATA.reload()` when
loading a run, so checkpoints written by a finished training job are visible
without redeploying.

## Common errors

| Symptom | Cause / fix |
|---|---|
| `output_dir ... already exists` | pass `--resume` (4th arg `1` to `train.sh`) or delete `outputs/<run>` |
| `NotImplementedError ... 'BFloat16'` | keep `--policy.use_amp=false` (bf16 GradScaler unsupported) |
| `run` missing from `/health` | wrong name, or training not finished; check `volume ls outputs` |
| `no checkpoint for run` | `checkpoints/last/pretrained_model` absent; training failed early |
| first request takes ~12 s | cold start; expected (rollout warms up first) |
| request timeout | client timeout is 180 s; a cold start plus large image is < 20 s |

## Security

The deployed endpoint is currently **unauthenticated**: anyone with the URL can
consume GPU time. Keep `JETARM_ENDPOINT` in `config/local.env` (git-ignored) and
out of public repositories. To publish one safely, enable Modal proxy auth
(`requires_proxy_auth=True` on the ASGI app) and send `Modal-Key`/`Modal-Secret`
headers from `ModalPolicyClient`, or put the endpoint behind your own auth proxy.
