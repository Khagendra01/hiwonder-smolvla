#!/usr/bin/env bash
# Bring up the JetArm software stack.
#
#   bash scripts/bringup.sh arm    # arm + kinematics + chassis (no app/camera)
#   bash scripts/bringup.sh full   # the full Hiwonder bringup (app, camera, webserver)
#
# NOTE: the low-level STM32 board is /dev/ttyUSB0. The wifi/button daemon
# (wifi_manager/button_scan.py) also opens it. Set TAKE_SERIAL=1 to stop that
# daemon first so the robot_controller owns the port cleanly.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"

MODE="${1:-arm}"

if [ "${TAKE_SERIAL:-0}" = "1" ]; then
  echo "[bringup] stopping button_scan.service to free $RRC_PORT (wifi stays up)"
  sudo systemctl stop button_scan.service 2>/dev/null || pkill -f "wifi_manager/button_scan.py" 2>/dev/null || true
  sleep 1
fi

case "$MODE" in
  arm)
    echo "[bringup] launching arm SDK stack (ros_robot_controller + servo_controller + kinematics + chassis)"
    exec ros2 launch sdk jetarm_sdk.launch.py
    ;;
  full)
    echo "[bringup] launching full Hiwonder bringup"
    exec ros2 launch bringup bringup.launch.py
    ;;
  camera)
    echo "[bringup] launching Orbbec Gemini depth camera"
    exec ros2 launch peripherals depth_camera.launch.py
    ;;
  *)
    echo "usage: bash scripts/bringup.sh [arm|full|camera]" >&2
    exit 2
    ;;
esac
