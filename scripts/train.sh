#!/usr/bin/env bash
# Fine-tune SmolVLA on a recorded JetArm dataset via Modal (A10G, native bf16).
#
#   bash scripts/train.sh <dataset> [steps] [batch_size] [resume]
#
# <dataset> is a run directory from record.sh, either a path or a name under
# datasets/. It must contain meta.json and episode_XXXX/.
# Uploads it to the Modal volume "jetarm-data" (custom/<name>), converts it to
# a LeRobotDataset, and fine-tunes lerobot/smolvla_base (expert-only).
# Pass 1 as the 4th argument to resume an interrupted run from its last
# checkpoint (runs are checkpointed every 2000 steps).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODAL="${MODAL:-$HOME/.local/bin/modal}"
PROFILE="${MODAL_PROFILE:-tobiwarwager}"

ARG="${1:?usage: train.sh <dataset> [steps] [batch_size]}"
STEPS="${2:-20000}"
BATCH="${3:-4}"
RESUME="${4:-0}"

if [ -d "$ARG" ]; then
  D="$(cd "$ARG" && pwd)"
else
  D="$HERE/../datasets/$ARG"
fi
NAME="$(basename "$D")"
[ -f "$D/meta.json" ] || { echo "ERROR: $D/meta.json not found" >&2; exit 1; }

echo "[train] dataset=$D"
echo "[train] name=$NAME steps=$STEPS batch=$BATCH gpu=A10G resume=$RESUME"
"$MODAL" --profile="$PROFILE" volume rm -r jetarm-data "custom/$NAME" 2>/dev/null || true
"$MODAL" --profile="$PROFILE" volume put jetarm-data "$D" custom/ --force
ARGS=(--name "$NAME" --steps "$STEPS" --batch-size "$BATCH")
[ "$RESUME" = "1" ] && ARGS+=(--resume)
"$MODAL" --profile="$PROFILE" run "$HERE/../modal/train_smolvla.py" "${ARGS[@]}"
