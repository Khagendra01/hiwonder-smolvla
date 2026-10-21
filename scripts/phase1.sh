#!/usr/bin/env bash
# Phase 1: validate the server/client topology with a dummy policy.
#   arm stack -> dummy policy server (separate process) -> client control loop
#
#   bash scripts/phase1.sh [joint] [amp] [period_s] [seconds]
set -u
source "$HOME/hiwonder-smolvla/scripts/env.sh"
cd "$PROJECT_ROOT"
mkdir -p logs

PORT="${PORT:-5555}"

cleanup() {
  pkill -f "[r]un_policy.py" 2>/dev/null || true
  pkill -f "[d]ummy_policy_server" 2>/dev/null || true
  pkill -f "[j]etarm_sdk.launch" 2>/dev/null || true
  pkill -f "[r]os_robot_controller" 2>/dev/null || true
  pkill -f "[s]ervo_controller" 2>/dev/null || true
  pkill -f "[s]ervo_manager" 2>/dev/null || true
  pkill -f "[k]inematics" 2>/dev/null || true
  pkill -f "[c]heck_servo_connection" 2>/dev/null || true
  pkill -f "[c]ontroller_manager" 2>/dev/null || true
  sleep 1
  sudo systemctl start button_scan.service 2>/dev/null || true
}
trap cleanup EXIT

echo "== freeing serial port (stop button_scan) =="
sudo systemctl stop button_scan.service 2>/dev/null || true
sleep 1

echo "== starting arm stack =="
setsid ros2 launch sdk jetarm_sdk.launch.py > logs/bringup.log 2>&1 < /dev/null &
up=0
for i in $(seq 1 30); do
  if timeout 3 ros2 topic list 2>/dev/null | grep -q /controller_manager/servo_states; then
    echo "   stack up after ~${i}s"; up=1; break
  fi
  sleep 1
done
if [ "$up" != "1" ]; then echo "   stack did NOT come up:"; tail -20 logs/bringup.log; exit 1; fi

echo "== starting dummy policy server (separate process) =="
setsid python3 bridge/dummy_policy_server.py --port "$PORT" \
  --joint "${1:-0}" --amp "${2:-15}" --period "${3:-2.0}" \
  > logs/policy_server.log 2>&1 < /dev/null &
sleep 1
sed 's/^/   /' logs/policy_server.log

echo "== client control loop =="
PYTHONPATH="$PROJECT_ROOT/bridge:${PYTHONPATH:-}" python3 bridge/run_policy.py \
  --server "127.0.0.1:$PORT" --joint "${1:-0}" --seconds "${4:-6.0}"
