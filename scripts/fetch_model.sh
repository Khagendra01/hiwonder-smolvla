#!/usr/bin/env bash
# Download a fine-tuned checkpoint from the Modal volume into models/<run>/.
# The endpoint does NOT need this; it loads the checkpoint from the volume.
# Use it for archiving, inspection, or shipping a model as a GitHub Release.
#
#   bash scripts/fetch_model.sh <run> [dest]
#
# Result (default): models/<run>/checkpoints/last/pretrained_model/
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODAL="${MODAL:-$HOME/.local/bin/modal}"
PROFILE="${MODAL_PROFILE:-tobiwarwager}"

RUN="${1:?usage: fetch_model.sh <run> [dest]}"
DEST="${2:-$HERE/../models/$RUN}"

mkdir -p "$DEST"
"$MODAL" --profile="$PROFILE" volume get jetarm-data "outputs/$RUN" "$DEST" --force
echo "[fetch] checkpoint -> $DEST/checkpoints/last/pretrained_model"
echo "[fetch] list runs: $MODAL --profile=$PROFILE volume ls jetarm-data outputs"
