# Source this file from bash.
task_fr5_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$task_fr5_ws/install/setup.bash"
export PYTHONPATH="$task_fr5_ws/vendor/fairino_python_sdk${PYTHONPATH:+:$PYTHONPATH}"
if [ -f "$task_fr5_ws/configs/legacy_fr5/ros_env.bash" ]; then
  source "$task_fr5_ws/configs/legacy_fr5/ros_env.bash"
fi
unset task_fr5_ws
