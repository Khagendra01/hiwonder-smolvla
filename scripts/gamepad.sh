#!/usr/bin/env bash
# Run the DOFBOT-style gamepad teleop (bridge/joy_dofbot.py).
#
# Sticks drive the arm, D-pad drives the tank, L3/R3 stay free for the recorder.
# This REPLACES Hiwonder's joystick_control (only one node may command
# /servo_controller), so it stops that node if it is running.
#
#   bash scripts/gamepad.sh             # live
#   bash scripts/gamepad.sh --dry-run   # inspect, do not publish
#
# The arm stack must already be up (ros2 launch sdk jetarm_sdk.launch.py).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"
cd "$PROJECT_ROOT"

n_ctl=$(pgrep -fc "[r]os_robot_controller --ros-args" 2>/dev/null)
n_ctl=${n_ctl:-0}
if [ "$n_ctl" -ne 1 ]; then
  echo "ERROR: expected exactly 1 ros_robot_controller, found $n_ctl." >&2
  echo "       Start the stack first: ros2 launch sdk jetarm_sdk.launch.py" >&2
  exit 1
fi

dry_run=0
for arg in "$@"; do
  [ "$arg" = "--dry-run" ] && dry_run=1
done

if pgrep -f "[j]oy_dofbot" >/dev/null 2>&1; then
  echo "ERROR: joy_dofbot is already running." >&2
  exit 1
fi

if [ "$dry_run" = 0 ] && pgrep -f "[j]oystick_control" >/dev/null 2>&1; then
  echo "== stopping Hiwonder joystick_control (one teleop at a time) =="
  pkill -TERM -f "[j]oystick_control" 2>/dev/null || true
  sleep 1
fi

exec python3 "$PROJECT_ROOT/bridge/joy_dofbot.py" --config "$PROJECT_ROOT/config/gamepad.yaml" "$@"
