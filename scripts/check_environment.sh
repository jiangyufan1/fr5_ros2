#!/usr/bin/env bash
set -Eeuo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "========== OS =========="
source /etc/os-release
echo "$PRETTY_NAME"

if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo "ERROR: Ubuntu 22.04 required."
  exit 1
fi

echo "========== ROS 2 =========="
test -f /opt/ros/humble/setup.bash || {
  echo "ERROR: ROS 2 Humble not found."
  exit 1
}
set +u
source /opt/ros/humble/setup.bash
set -u
echo "$ROS_DISTRO"

echo "========== TOOLS =========="
command -v colcon
command -v cmake
command -v python3

echo "========== SDK LINKS =========="
for link in \
  "$WS/src/fairino_hardware/libfairino" \
  "$WS/vendor/fairino_python_sdk"
do
  if [[ -L "$link" && -e "$link" ]]; then
    echo "OK: $link -> $(readlink -f "$link")"
  else
    echo "ERROR: missing or broken SDK link: $link"
    exit 1
  fi
done

echo "========== ROS PACKAGES =========="
cd "$WS"
colcon list

echo "Environment check completed."
