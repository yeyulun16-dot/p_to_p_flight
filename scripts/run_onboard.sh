#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/onboard_ws/install/setup.bash"

exec ros2 launch p_to_p_mission p_to_p.launch.py "$@"
