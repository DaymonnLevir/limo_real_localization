#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if ! ip -4 addr show eth0 | grep -q 'inet 192.168.1.50/24'; then
  echo 'Falta o IP temporário. Execute: sudo ip addr add 192.168.1.50/24 dev eth0' >&2
  exit 1
fi
if docker ps --format '{{.Names}}' | grep -qx limo-point-lio; then
  echo 'Container limo-point-lio já está ativo.'
  exit 0
fi
mkdir -p validation/point_lio/raw
docker run -d --name limo-point-lio --network host --ipc host \
  --device /dev/ttyTHS0:/dev/ttyTHS0 \
  -e ROS_DOMAIN_ID=45 -e DISPLAY="${DISPLAY:-:0}" \
  -e XAUTHORITY=/tmp/limo.xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${XAUTHORITY:-/run/user/1000/gdm/Xauthority}":/tmp/limo.xauthority:ro \
  -v "$PWD/validation/point_lio":/validation \
  limo-real-point-lio:humble sleep infinity
echo 'Container pronto. Inicie uma única instância do driver Livox; veja docs/POINT_LIO_LIMO.md.'
