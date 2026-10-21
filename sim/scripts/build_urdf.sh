#!/usr/bin/env bash
# Regenerate sim/urdf/jetarm_6dof.urdf from the fetched Hiwonder xacro.
# Requires ROS 2 (xacro). Run fetch_assets.sh first.
#   bash sim/scripts/build_urdf.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
ASSETS="$ROOT/sim/assets/jetarm_6dof_description"

[ -d "$ASSETS/urdf" ] || { echo "ERROR: assets missing; run sim/scripts/fetch_assets.sh" >&2; exit 1; }
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

PREFIX="$(mktemp -d)"
RAW="$(mktemp)"
trap 'rm -rf "$PREFIX" "$RAW"' EXIT
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"
ln -s "$ASSETS" "$PREFIX/share/jetarm_6dof_description"
touch "$PREFIX/share/ament_index/resource_index/packages/jetarm_6dof_description"

AMENT_PREFIX_PATH="$PREFIX:$AMENT_PREFIX_PATH" \
  xacro "$ASSETS/urdf/jetarm_6dof_robot.urdf.xacro" camera_type:=GEMINI > "$RAW"

python3 "$ROOT/sim/tools/postprocess_urdf.py" "$RAW" "$ROOT/sim/urdf/jetarm_6dof.urdf"
echo "[build] wrote sim/urdf/jetarm_6dof.urdf"
