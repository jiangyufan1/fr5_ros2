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
exec ros2 run fairino_hardware ros2_cmd_server
