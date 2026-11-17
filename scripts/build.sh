#!/usr/bin/env bash
set -e
set -o pipefail
task_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_ws"
source /opt/ros/humble/setup.bash
exec colcon build --base-paths src
