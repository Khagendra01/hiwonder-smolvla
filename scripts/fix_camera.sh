#!/usr/bin/env bash
# Restore /dev/video0 (Orbbec UVC color interface) by re-enumerating just that
# USB device. The separate depth/IR vendor interface is not needed for RGB.
#
#   bash scripts/fix_camera.sh
set -u

if [ -e /dev/video0 ]; then
  echo "OK: /dev/video0 already present"
  exit 0
fi

dev=""
for d in /sys/bus/usb/devices/*; do
  [ -e "$d/idVendor" ] || continue
  if [ "$(cat "$d/idVendor" 2>/dev/null)" = "2bc5" ] && [ "$(cat "$d/idProduct" 2>/dev/null)" = "0511" ]; then
    dev="$(basename "$d")"
    break
  fi
done

if [ -z "$dev" ]; then
  echo "ERROR: Orbbec UVC device (2bc5:0511) not found - reseat the camera USB cable" >&2
  exit 1
fi

echo "re-enumerating $dev"
echo 0 | sudo tee "/sys/bus/usb/devices/$dev/authorized" >/dev/null 2>&1 || true
sleep 2
echo 1 | sudo tee "/sys/bus/usb/devices/$dev/authorized" >/dev/null 2>&1 || true
sleep 3

if [ -e /dev/video0 ]; then
  echo "OK: /dev/video0"
else
  echo "FAILED: reseat the camera USB cable (depth/IR device is erroring -71)" >&2
  exit 1
fi
