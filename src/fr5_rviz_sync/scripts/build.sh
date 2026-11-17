#!/usr/bin/env bash
set -eo pipefail
task_live_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
test -f "$task_live_ws/src/fr5_rviz_sync/package.xml" || { echo 'Place this package under workspace/src'; exit 1; }
source /opt/ros/humble/setup.bash
cd "$task_live_ws"
colcon build --symlink-install --packages-select fairino_msgs fairino_description fr5_rviz_sync
echo 'Built read-only live display; no robot process was launched.'
