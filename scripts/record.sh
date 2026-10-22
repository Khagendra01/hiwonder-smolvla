#!/usr/bin/env bash
# Record demonstrations in PASSIVE mode. Teleop = your gamepad, either Hiwonder
# joystick_control or bridge/joy_dofbot.py (DOFBOT-style); video = /dev/video0
# (Orbbec UVC color). The broken Orbbec depth/IR node is NOT started. Nothing
# already running is touched.
#
#   bash scripts/record.sh --task "pick up the red cube" --num-episodes 50
#   bash scripts/record.sh --task "..." --out datasets/pick_red --fps 20
#
# If /dev/video0 is missing:  bash scripts/fix_camera.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"
cd "$PROJECT_ROOT"
mkdir -p logs datasets

stack_up() { pgrep -f "[r]os_robot_controller --ros-args" >/dev/null 2>&1; }

STARTED_ARM=0
STARTED_JOY=0
WAS_BUTTON_SCAN=0
if timeout 5 systemctl is-active --quiet button_scan.service; then
  WAS_BUTTON_SCAN=1
fi
cleanup() {
  if [ "$STARTED_JOY" = 1 ]; then
    pkill -TERM -f "[j]oystick_control" 2>/dev/null || true
  fi
  if [ "$STARTED_ARM" = 1 ]; then
    pkill -TERM -f "[j]etarm_sdk.launch" 2>/dev/null || true
    pkill -TERM -f "[r]os_robot_controller" 2>/dev/null || true
    pkill -TERM -f "[s]ervo_controller" 2>/dev/null || true
    pkill -TERM -f "[s]earch_kinematics_solutions" 2>/dev/null || true
    pkill -TERM -f "[c]heck_servo_connection" 2>/dev/null || true
    pkill -TERM -f "[c]ontroller_manager" 2>/dev/null || true
    pkill -TERM -f "[c]hassis_contrller" 2>/dev/null || true
  fi
  if [ "$STARTED_JOY" = 1 ] || [ "$STARTED_ARM" = 1 ]; then
    sleep 2
    pkill -KILL -f "[j]oystick_control" 2>/dev/null || true
    pkill -KILL -f "[j]etarm_sdk.launch" 2>/dev/null || true
    pkill -KILL -f "[r]os_robot_controller" 2>/dev/null || true
    pkill -KILL -f "[s]ervo_controller" 2>/dev/null || true
    pkill -KILL -f "[c]ontroller_manager" 2>/dev/null || true
  fi
  if [ "$WAS_BUTTON_SCAN" = 1 ] && ! pgrep -f "[r]os_robot_controller" >/dev/null 2>&1; then
    timeout 10 sudo -n systemctl start button_scan.service 2>/dev/null || true
  fi
}
trap cleanup EXIT

if [ ! -e /dev/video0 ]; then
  echo "ERROR: /dev/video0 missing. Run: bash scripts/fix_camera.sh" >&2
  exit 1
fi

# button_scan shares /dev/ttyUSB0 with ros_robot_controller: never run both.
if [ "$WAS_BUTTON_SCAN" = 1 ]; then
  echo "== stopping button_scan (shares $RRC_PORT) =="
  timeout 10 sudo -n systemctl stop button_scan.service 2>/dev/null || true
  sleep 1
fi

n_ctl=$(pgrep -fc "[r]os_robot_controller --ros-args" 2>/dev/null)
n_ctl=${n_ctl:-0}
if [ "$n_ctl" -gt 1 ]; then
  echo "ERROR: $n_ctl ros_robot_controller processes are fighting over $RRC_PORT." >&2
  echo "       Run: bash scripts/restart_robot.sh" >&2
  exit 1
fi

if ! stack_up; then
  echo "== starting arm stack =="
  setsid ros2 launch sdk jetarm_sdk.launch.py > logs/bringup.log 2>&1 < /dev/null &
  STARTED_ARM=1
  deadline=$((SECONDS + 60))
  while [ "$SECONDS" -lt "$deadline" ]; do
    stack_up && break
    sleep 1
  done
  if ! stack_up; then
    echo "arm stack did not come up; see logs/bringup.log" >&2
    tail -20 logs/bringup.log
    exit 1
  fi
  sleep 2
fi

if pgrep -f "[j]oy_dofbot" >/dev/null 2>&1; then
  echo "== gamepad teleop: joy_dofbot already running =="
else
  echo "== starting gamepad teleop (joystick_control) =="
  pkill -TERM -f "[j]oystick_control" 2>/dev/null || true
  sleep 1
  setsid ros2 launch peripherals joystick_control.launch.py > logs/joystick.log 2>&1 < /dev/null &
  STARTED_JOY=1
  sleep 3
fi

echo "== recording: L3=start/stop, R3=discard (keyboard r/x/q) =="
python3 "$PROJECT_ROOT/bridge/record_smolvla.py" "$@"
