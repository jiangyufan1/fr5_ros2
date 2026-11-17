#!/usr/bin/env bash
set -e
set -o pipefail
if [[ $(id -u) -eq 0 ]]; then
  echo 'Run as dj306, not sudo bash. Only Docker commands use sudo.' >&2
  exit 1
fi
task_tools="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
task_work="/data/Projects/fr5_ros2/outputs/ikfast_generation_$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$task_work"
exec > >(tee -a "$task_work/generation.log") 2>&1
echo "Generation artifacts: $task_work"
/usr/bin/python3 "$task_tools/prepare_model.py" "$task_work"
cp "$task_tools/generate_openrave.py" "$task_work/generate_openrave.py"
cp "$task_tools/ikfast_compat.py" "$task_work/ikfast_compat.py"
if ! sudo docker image inspect personalrobotics/ros-openrave:latest >/dev/null 2>&1; then
  sudo docker pull personalrobotics/ros-openrave:latest
else
  echo 'Reusing local OpenRAVE image; no registry connection needed.'
fi
task_image=$(sudo docker image inspect --format '{{.Id}}' personalrobotics/ros-openrave:latest)
printf '%s\n' "$task_image" > "$task_work/container_image_id.txt"
task_container="fr5-ikfast-$(id -u)-$$"
trap 'sudo docker stop --time 10 "$task_container" >/dev/null 2>&1 || true' EXIT
timeout --foreground 7200 sudo docker run --rm --name "$task_container" \
  --network none --user "$(id -u):$(id -g)" \
  -v "$task_work:/work" -w /work -e HOME=/work \
  -e PYTHONPATH=/work/deps/sympy-0.7.1 \
  --entrypoint /bin/bash "$task_image" -lc 'python -u /work/generate_openrave.py'
test -s "$task_work/fr5_ikfast_solver.cpp"
printf '%s\n' "$task_work" > /data/Projects/fr5_ros2/outputs/ikfast_generation_latest.txt
echo 'Generation complete. No MoveIt configuration was changed and no robot was connected.'
cat "$task_work/generation_result.json"
