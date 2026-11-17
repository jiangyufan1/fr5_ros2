#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/source_live.bash"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo 'Run this in an Ubuntu desktop/remote-desktop terminal, not a plain SSH terminal.'
  exit 1
fi
task_live_config="$(ros2 pkg prefix fr5_rviz_sync)/share/fr5_rviz_sync/config/live.rviz"
ros2 run rviz2 rviz2 -d "$task_live_config" --ros-args \
  -r __ns:=/fr5_live -r /tf:=/fr5_live/tf -r /tf_static:=/fr5_live/tf_static
