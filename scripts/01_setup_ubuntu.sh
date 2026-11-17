#!/usr/bin/env bash
set -eo pipefail
if [ "$(id -u)" = 0 ]; then echo 'Run as the normal Ubuntu user, not sudo bash.'; exit 1; fi
. /etc/os-release
if [ "$ID" != ubuntu ] || [ "$VERSION_ID" != 22.04 ]; then echo 'Ubuntu 22.04 required.'; exit 1; fi
if [ "$(uname -m)" != x86_64 ]; then echo 'The supplied vendor library requires Linux x86_64.'; exit 1; fi
task_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sudo apt-get update
if [ ! -f /opt/ros/humble/setup.bash ]; then
  sudo apt-get --no-remove install -y curl ca-certificates software-properties-common locales python3
  sudo add-apt-repository -y universe
  sudo locale-gen en_US.UTF-8
  export LANG=en_US.UTF-8
  mkdir -p "$task_ws_root/.setup-downloads"
  task_ros_source_version="$(curl --fail --location --retry 2 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  curl --fail --location --retry 2 -o "$task_ws_root/.setup-downloads/ros2-apt-source.deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${task_ros_source_version}/ros2-apt-source_${task_ros_source_version}.jammy_all.deb"
  sudo dpkg -i "$task_ws_root/.setup-downloads/ros2-apt-source.deb"
  sudo apt-get update
  sudo apt-get --no-remove install -y ros-humble-desktop
fi
sudo apt-get --no-remove install -y build-essential cmake python3-colcon-common-extensions python3-rosdep python3-setuptools ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-rclpy ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-rviz2
echo 'Humble and core build dependencies installed. No robot driver has been started.'
echo "Next: bash $task_ws_root/scripts/02_build.sh"
