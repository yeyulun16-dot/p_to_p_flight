#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/onboard_ws/install/setup.bash"

echo "== Serial devices =="
ls -l /dev/ttyS6 /dev/ttyS4 /dev/ttyS3 2>/dev/null || true

echo "== Required topics =="
for topic in /scan /laser_array/ground_height /target_position /target_velocity /p_to_p/state; do
  if ros2 topic list | grep -Fxq "${topic}"; then
    echo "OK  ${topic}"
  else
    echo "MISS ${topic}"
  fi
done

echo "== One height sample =="
timeout 3 ros2 topic echo --once /laser_array/ground_height || true

echo "== One map->laser_link transform =="
timeout 3 ros2 run tf2_ros tf2_echo map laser_link || true

echo "Review all MISS/timeouts before arming. This script does not authorize flight."
