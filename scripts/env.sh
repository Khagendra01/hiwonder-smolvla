#!/usr/bin/env bash
# Source this to get a working ROS2 + project environment.
#   source scripts/env.sh
#
# It sources the Hiwonder ROS2 workspace (mirroring ros2_ws/.hiwonderrc) plus our
# project config. ROS setup files are not nounset-safe, so we relax `set -u`.

# remember caller's nounset state
case "$-" in *u*) _had_u=1 ;; *) _had_u=0 ;; esac
set +u

# --- ROS2 base ---
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

# --- Project config ---
# Portable: works when sourced from bash or zsh (avoids BASH_SOURCE).
PROJECT_ROOT="${PROJECT_ROOT:-$HOME/hiwonder-smolvla}"
# shellcheck disable=SC1091
source "$PROJECT_ROOT/config/robot.env"

# --- Hiwonder workspace overlays ---
for f in \
    "$ROS_WS/install/local_setup.bash" \
    "$THIRD_PARTY_ROS2/third_party_ws/install/local_setup.bash" \
    "$THIRD_PARTY_ROS2/orbbec_ws/install/local_setup.bash"; do
  [ -f "$f" ] && source "$f"
done

export PROJECT_ROOT

# Optional local-only settings (endpoint URL, overrides). Git-ignored.
[ -f "$PROJECT_ROOT/config/local.env" ] && source "$PROJECT_ROOT/config/local.env"

# restore nounset if the caller had it
[ "$_had_u" = "1" ] && set -u
unset _had_u
