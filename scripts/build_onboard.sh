#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/onboard_ws"

python3 "${ROOT_DIR}/scripts/verify_offline.py"
source /opt/ros/humble/setup.bash
cd "${WS_DIR}"

rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

echo "Build complete. Run: source ${WS_DIR}/install/setup.bash"
