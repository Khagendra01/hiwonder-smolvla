#!/usr/bin/env bash
# Cleanly restart the full JetArm bringup with a SINGLE owner of the STM32
# serial port (/dev/ttyUSB0). Stops button_scan, kills stray controllers,
# then starts bringup.launch.py and verifies.
#
#   bash scripts/restart_robot.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"
cd "$PROJECT_ROOT"
mkdir -p logs

echo "== stop button_scan (shares $RRC_PORT) =="
sudo systemctl stop button_scan.service 2>/dev/null || true
sleep 1

echo "== stop old stack / stray controllers =="
pkill -TERM -f "[b]ringup.launch" 2>/dev/null || true
pkill -TERM -f "[j]etarm_sdk.launch" 2>/dev/null || true
pkill -TERM -f "[r]os_robot_controller" 2>/dev/null || true
pkill -TERM -f "[s]ervo_controller" 2>/dev/null || true
pkill -TERM -f "[c]ontroller_manager" 2>/dev/null || true
pkill -TERM -f "[s]earch_kinematics_solutions" 2>/dev/null || true
pkill -TERM -f "[c]heck_servo_connection" 2>/dev/null || true
pkill -TERM -f "[j]oystick_control" 2>/dev/null || true
sleep 3
pkill -KILL -f "[r]os_robot_controller" 2>/dev/null || true
pkill -KILL -f "[b]ringup.launch" 2>/dev/null || true
sleep 1

echo "== serial after cleanup =="
sudo fuser -v "$RRC_PORT" 2>&1 || echo "(free)"

echo "== start bringup =="
setsid ros2 launch bringup bringup.launch.py > logs/bringup_full.log 2>&1 < /dev/null &

for _ in $(seq 1 60); do
  timeout 5 ros2 topic list 2>/dev/null | grep -qx "$TOPIC_SERVO_STATES" && break
  sleep 1
done

if ! timeout 5 ros2 topic list 2>/dev/null | grep -qx "$TOPIC_SERVO_STATES"; then
  echo "bringup failed to come up; see logs/bringup_full.log" >&2
  tail -20 logs/bringup_full.log
  exit 1
fi

sleep 2
n_ctl=$(pgrep -fc "[r]os_robot_controller --ros-args" 2>/dev/null || echo 0)
echo "ros_robot_controller processes: $n_ctl (expect 1)"
sudo fuser -v "$RRC_PORT" 2>&1 | tail -3
echo "OK. Gamepad: joystick_control (mode 0 = joint, mode 1 = Cartesian)."
