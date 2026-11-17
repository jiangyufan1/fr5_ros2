# Source this file from bash.
_task_fr5_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

_task_fr5_had_nounset=0
case $- in
  *u*)
    _task_fr5_had_nounset=1
    set +u
    ;;
esac

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble is not installed." >&2
  if [ "$_task_fr5_had_nounset" = 1 ]; then set -u; fi
  return 1
fi

if [ ! -f "$_task_fr5_ws/install/setup.bash" ]; then
  echo "ERROR: Workspace is not built. Run build scripts first." >&2
  if [ "$_task_fr5_had_nounset" = 1 ]; then set -u; fi
  return 1
fi

source /opt/ros/humble/setup.bash
source "$_task_fr5_ws/install/setup.bash"

export PYTHONPATH="$_task_fr5_ws/vendor/fairino_python_sdk${PYTHONPATH:+:$PYTHONPATH}"

if [ -f "$_task_fr5_ws/configs/legacy_fr5/ros_env.bash" ]; then
  source "$_task_fr5_ws/configs/legacy_fr5/ros_env.bash"
fi

if [ "$_task_fr5_had_nounset" = 1 ]; then
  set -u
fi

unset _task_fr5_ws _task_fr5_had_nounset
