#!/usr/bin/env bash
set -Eeuo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WS"

echo "Building FR5 MoveIt and IKFast packages..."

colcon build --symlink-install \
  --packages-up-to fairino5_ikfast_plugin fairino5_v6_moveit2_config \
  --cmake-args -DBUILD_TESTING=OFF

echo "MoveIt build completed."
