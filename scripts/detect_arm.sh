#!/usr/bin/env bash
# Read-only JetArm detection.
#
# Confirms the robot model, the controller serial device, and (if the stack is up)
# reads the 6 arm servo positions via the low-level ROS2 service. Sends NO motion.
#
# Usage: bash scripts/detect_arm.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"

echo "=============================================="
echo " JetArm detection"
echo "=============================================="
echo "ROBOT_MODEL : $ROBOT_MODEL"
echo "ARM_DOF     : $ARM_DOF"
echo "CAMERA_TYPE : $CAMERA_TYPE"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

echo
echo "--- controller serial device ---"
for d in "$RRC_PORT" /dev/ttyCH341USB0 /dev/ttyCH341USB1; do
  if [ -e "$d" ]; then
    printf '  %-22s -> %s\n' "$d" "$(readlink -f "$d")"
  fi
done

echo
echo "--- port holders (a running daemon may own the board) ---"
if command -v fuser >/dev/null 2>&1 && [ -e "$RRC_PORT" ]; then
  holders="$(fuser "$RRC_PORT" 2>/dev/null | tr -s ' ')"
  echo "  $RRC_PORT held by PID(s):${holders:-none}"
fi

echo
echo "--- ROS2 graph ---"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "  ros2 CLI not found after sourcing env" >&2
  exit 1
fi
nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
if [ -z "$nodes" ]; then
  echo "  No ROS2 nodes running."
  echo "  => Start the arm stack with: bash scripts/bringup.sh"
  echo "  => Then re-run this script to read live joint positions."
  exit 0
fi
echo "$nodes" | sed 's/^/  /'

echo
echo "--- arm joint feedback (read-only) ---"
if echo "$nodes" | grep -q ros_robot_controller; then
  python3 "$HERE/bus_query.py" || echo "  (could not read servo state)"
else
  echo "  ros_robot_controller node not running."
fi
