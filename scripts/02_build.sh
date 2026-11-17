#!/usr/bin/env bash
set -eo pipefail
task_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test -f /opt/ros/humble/setup.bash || { echo 'Run scripts/01_setup_ubuntu.sh first.'; exit 1; }
source /opt/ros/humble/setup.bash
cd "$task_ws_root"
colcon build --symlink-install --packages-select fairino_msgs fairino_description fairino_hardware fr5_python --cmake-args -DBUILD_TESTING=OFF
echo "Build completed. In each new terminal: source $task_ws_root/scripts/03_source.bash"
