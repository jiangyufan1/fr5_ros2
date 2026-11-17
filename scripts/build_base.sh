#!/usr/bin/env bash
set -Eeuo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WS"

echo "Building FR5 base packages..."

colcon build --symlink-install \
  --packages-up-to fairino_hardware fr5_python fr5_rviz_sync \
  --cmake-args -DBUILD_TESTING=OFF

echo "Base build completed."
echo "Run: source $WS/scripts/03_source.bash"
