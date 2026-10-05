#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! ip -4 addr show eth0 | grep -q 'inet 192.168.1.50/24'; then
  echo 'IP 192.168.1.50/24 ausente; para a Fase 2: sudo ip addr add 192.168.1.50/24 dev eth0' >&2
  exit 1
fi
if docker ps --format '{{.Names}}' | grep -qx limo-ct-lio; then
  echo 'Container limo-ct-lio já ativo.'
  exit 0
fi
mkdir -p validation/ct_lio/raw
docker run -d --name limo-ct-lio --network host --ipc host \
  -e ROS_DOMAIN_ID=46 -e DISPLAY="${DISPLAY:-:0}" \
  -e XAUTHORITY=/tmp/limo.xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${XAUTHORITY:-/run/user/1000/gdm/Xauthority}":/tmp/limo.xauthority:ro \
  -v "$PWD/validation/ct_lio":/validation \
  limo-real-ct-lio:humble sleep infinity
