#!/usr/bin/env bash
set -e
set -o pipefail
task_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_ws"
if pgrep -u "$(id -u)" -f '(^|/)(ros2_control_node|ros2_cmd_server)( |$)' > /dev/null; then
  echo 'Another robot driver is running. Stop it before starting this program.' >&2
  exit 1
fi
source "$task_ws/scripts/03_source.bash"
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo 'Run this command in an Ubuntu desktop terminal for RViz.' >&2
  exit 1
fi
exec ros2 launch "$task_ws/scripts/fr5_moveit_live.launch.py" robot_ip:=192.168.57.2 enable_motion:=true
