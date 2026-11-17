#!/usr/bin/env bash
set -e
set -o pipefail
if [[ $# -ne 1 ]]; then
  echo 'Usage: bash install_fr5_hybrid.sh /data/Projects/fr5_ros2/outputs/.../host_validation_...' >&2
  exit 2
fi
source /data/Projects/fr5_ros2/scripts/03_source.bash
source /data/Projects/fr5_ros2/install/setup.bash
cd /data/Projects/fr5_ros2
exec /usr/bin/python3 /data/Projects/fr5_ros2/tools/ikfast/install_fr5_hybrid.py "$1"
