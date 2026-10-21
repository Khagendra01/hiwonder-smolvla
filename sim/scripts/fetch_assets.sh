#!/usr/bin/env bash
# Fetch the official Hiwonder JetArm description (URDF/xacro + STL meshes).
#   bash sim/scripts/fetch_assets.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/../assets/jetarm_6dof_description"
REPO="https://github.com/Hiwonder/JetArm.git"
BRANCH="Jetson_Orin_NX_ros1"
SUBPATH="jetarm/src/jetarm_6dof_simulate/jetarm_6dof_description"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "[fetch] $REPO ($BRANCH)"
git clone --depth 1 --filter=blob:none --sparse -b "$BRANCH" "$REPO" "$TMP/repo"
git -C "$TMP/repo" sparse-checkout set "$SUBPATH"

rm -rf "$DEST"
mkdir -p "$(dirname "$DEST")"
cp -a "$TMP/repo/$SUBPATH" "$DEST"
echo "[fetch] -> $DEST"
du -sh "$DEST"
