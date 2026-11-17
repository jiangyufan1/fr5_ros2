#!/usr/bin/env bash
set -Eeuo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WS"

echo "Building all FR5 ROS 2 packages..."

colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF

echo "Full workspace build completed."
