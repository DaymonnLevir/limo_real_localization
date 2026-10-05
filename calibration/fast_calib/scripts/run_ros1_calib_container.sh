#!/bin/bash
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
# Refuse conflicting UDP receivers. Never stop other workloads.
if ss -H -uan | grep -Eq ':(56101|56201|56301|56401|11321)[[:space:]]'; then
  echo 'Portas Livox ocupadas. Pare manualmente o driver existente antes da aquisição.' >&2; exit 1
fi
if ss -H -ltn | grep -Eq ':11321[[:space:]]'; then
  echo 'Porta do master ROS1 11321 ocupada.' >&2; exit 1
fi
[[ -c /dev/video0 ]] || { echo '/dev/video0 indisponível' >&2; exit 1; }
docker run -d --name limo-fast-calib-ros1 --network host \
  --device=/dev/video0:/dev/video0 --user "$(id -u):$(id -g)" \
  --group-add "$(stat -c %g /dev/video0)" \
  --mount "type=bind,src=$root,dst=/calibration" limo-fast-calib-ros1:noetic
