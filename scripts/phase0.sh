#!/usr/bin/env bash
# One-shot Phase 0 runner: start the arm stack, read -> nudge -> return, stop stack.
# Everything happens in a single process so the stack does not need to persist.
#
#   bash scripts/phase0.sh [joint_index] [delta_pulses] [duration_s]
set -u
source "$HOME/hiwonder-smolvla/scripts/env.sh"
cd "$PROJECT_ROOT"
mkdir -p logs

cleanup() {
  pkill -f "jetarm_sdk.launch" 2>/dev/null || true
  pkill -f "ros_robot_controller" 2>/dev/null || true
  pkill -f "servo_controller" 2>/dev/null || true
  pkill -f "search_kinematics_solutions" 2>/dev/null || true
  pkill -f "check_servo_connection" 2>/dev/null || true
  pkill -f "controller_manager" 2>/dev/null || true
  sleep 1
  sudo systemctl start button_scan.service 2>/dev/null || true
}
trap cleanup EXIT

# Free the STM32 serial port for ros_robot_controller (button_scan holds it).
echo "== freeing serial port (stop button_scan) =="
sudo systemctl stop button_scan.service 2>/dev/null || true
sleep 1

echo "== starting arm stack (detached) =="
setsid ros2 launch sdk jetarm_sdk.launch.py > logs/bringup.log 2>&1 < /dev/null &
sleep 2

echo "== waiting for /controller_manager/servo_states =="
up=0
for i in $(seq 1 40); do
  if timeout 5 ros2 topic list 2>/dev/null | grep -q /controller_manager/servo_states; then
    echo "   stack up after ~${i}s"; up=1; break
  fi
  sleep 1
done
if [ "$up" != "1" ]; then
  echo "   stack did NOT come up. bringup log:"; tail -20 logs/bringup.log
  exit 1
fi

echo "== nodes =="
timeout 6 ros2 node list 2>/dev/null | sed 's/^/   /'

echo "== Phase 0 nudge =="
PYTHONPATH="$PROJECT_ROOT/bridge:${PYTHONPATH:-}" python3 bridge/phase0_nudge.py "${1:-0}" "${2:-20}" "${3:-0.8}"
