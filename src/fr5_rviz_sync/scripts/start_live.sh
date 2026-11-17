#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/source_live.bash"
task_live_rviz=true
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  task_live_rviz=false
  echo 'No desktop display in this terminal: starting feedback/TF only.'
  echo 'Open an Ubuntu desktop terminal and run scripts/view_rviz.sh to view the model.'
fi
ros2 launch fr5_rviz_sync live.launch.py rviz:="$task_live_rviz"
