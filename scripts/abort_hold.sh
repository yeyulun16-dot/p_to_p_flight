#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/onboard_ws/install/setup.bash"
ros2 service call /p_to_p/abort_hold std_srvs/srv/Trigger '{}'
