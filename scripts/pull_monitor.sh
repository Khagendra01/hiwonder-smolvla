#!/usr/bin/env bash
LOG="/home/ubuntu/hiwonder-smolvla/logs/pull_progress.log"
mkdir -p "$(dirname "$LOG")"
while true; do
  ts=$(date '+%H:%M:%S')
  unit=$(systemctl is-active lerobotpull 2>/dev/null)
  data=$(du -sh /var/lib/docker 2>/dev/null | cut -f1)
  free=$(df -h / | tail -1 | awk '{print $4}')
  img=$(docker images -q dustynv/lerobot:r36.3.0 2>/dev/null | head -1)
  last=$(journalctl -u lerobotpull --no-pager -n 1 2>/dev/null | sed 's/.*docker\[[0-9]*\]: //')
  echo "[$ts] unit=$unit disk=$data free=$free img=${img:-none} | $last" >> "$LOG"
  if [ -n "$img" ]; then echo "[$ts] IMAGE READY" >> "$LOG"; break; fi
  [ "$unit" = "active" ] || { echo "[$ts] monitor ended (unit=$unit)" >> "$LOG"; break; }
  sleep 30
done
