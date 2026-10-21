#!/usr/bin/env bash
# SmolVLA needs Python 3.12 + torch>=2.7, which this JetPack 6.0 host does NOT
# have (it ships Python 3.10 + torch 2.4). So the ML side runs in a container
# built from NVIDIA jetson-containers' `lerobot` recipe.
#
#   bash scripts/container.sh build   # build the lerobot image (slow, once)
#   bash scripts/container.sh shell   # interactive shell in the image
#   bash scripts/container.sh run -- <cmd...>   # run a command in the image
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"

JC="$JETSON_CONTAINERS/jetson-containers"
AT="$JETSON_CONTAINERS/autotag"
[ -x "$JC" ] || { echo "jetson-containers launcher not found at $JC" >&2; exit 1; }

# -q = unattended: auto-select a prebuilt registry image when one exists.
TAG="$("$AT" -q lerobot 2>/dev/null || echo "$LEROBOT_CONTAINER_TAG")"
CMD="${1:-shell}"; shift || true

case "$CMD" in
  build)
    echo "[container] building '$TAG' (this can take a long time on aarch64)"
    exec "$JC" build lerobot
    ;;
  shell)
    echo "[container] shell into '$TAG'  (project mounted at /work)"
    exec "$JC" run --name lerobot --network host -v "$PROJECT_ROOT:/work" "$TAG"
    ;;
  run)
    [ "${1:-}" = "--" ] && shift
    exec "$JC" run --rm --network host -v "$PROJECT_ROOT:/work" "$TAG" "$@"
    ;;
  *)
    echo "usage: bash scripts/container.sh [build|shell|run -- <cmd>]" >&2
    exit 2
    ;;
esac
