#!/bin/bash
set -euo pipefail
root=$(cd "$(dirname "$0")/../../.." && pwd)
docker build -t limo-fast-calib-ros1:noetic "$root/docker/fast_calib_ros1"
