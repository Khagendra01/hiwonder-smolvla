#!/usr/bin/env bash
# Run a trained SmolVLA policy on the JetArm via the Modal HTTP endpoint.
#
#   bash scripts/rollout.sh --url https://...modal.run --run <name> \
#       --task "pick up the red cube" --duration 10 [--dry-run]
#
# Starts only the arm stack (not the Hiwonder app/camera), frees the serial
# from button_scan, and restores it on exit.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/env.sh"
cd "$PROJECT_ROOT"
mkdir -p logs

stack_up() { pgrep -f "[r]os_robot_controller --ros-args" >/dev/null 2>&1; }

STARTED_ARM=0
WAS_BUTTON_SCAN=0
if timeout 5 systemctl is-active --quiet button_scan.service; then
  WAS_BUTTON_SCAN=1
fi
cleanup() {
  if [ "$STARTED_ARM" = 1 ]; then
    pkill -TERM -f "[j]etarm_sdk.launch" 2>/dev/null || true
    pkill -TERM -f "[r]os_robot_controller" 2>/dev/null || true
    pkill -TERM -f "[s]ervo_controller" 2>/dev/null || true
    pkill -TERM -f "[s]earch_kinematics_solutions" 2>/dev/null || true
    pkill -TERM -f "[c]heck_servo_connection" 2>/dev/null || true
    pkill -TERM -f "[c]ontroller_manager" 2>/dev/null || true
    pkill -TERM -f "[c]hassis_contrller" 2>/dev/null || true
    sleep 2
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

if [ "$WAS_BUTTON_SCAN" = 1 ]; then
  echo "== stopping button_scan (shares $RRC_PORT) =="
  timeout 10 sudo -n systemctl stop button_scan.service 2>/dev/null || true
  sleep 1
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

python3 "$PROJECT_ROOT/bridge/rollout_smolvla.py" "$@"
