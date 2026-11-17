# Source from the source tree: source src/fr5_rviz_sync/scripts/source_live.bash
task_live_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -f "$task_live_ws/install/setup.bash" ]]; then
  echo 'Build the workspace first'
  unset task_live_ws
  return 1
fi
source /opt/ros/humble/setup.bash
source "$task_live_ws/install/setup.bash"
unset task_live_ws

# >>> FR5 ROS isolation >>>
if [ -f "/data/Projects/fr5_ros2/configs/legacy_fr5/ros_env.bash" ]; then
  . "/data/Projects/fr5_ros2/configs/legacy_fr5/ros_env.bash"
fi
echo "Live display: ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
# <<< FR5 ROS isolation <<<
