#!/usr/bin/env bash
set -eo pipefail
task_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$task_ws_root/scripts/03_source.bash"
ros2 pkg prefix fairino_msgs
ros2 pkg prefix fairino_hardware
ros2 pkg prefix fairino_description
ros2 pkg executables fr5_python
ros2 interface show fairino_msgs/srv/RemoteCmdInterface
python3 -c 'from fairino import Robot; print("SDK source:", Robot.__file__)'
echo 'Package/import checks finished; no connection to the robot was made.'
