#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
expected=sha256:bba1ef40cc6efaa3ed6e540cff9eda97d9140c77dafd6a04a0532b2fc7981ce1
actual=$(docker image inspect limo-real-fast-livo2:stage2 --format '{{.Id}}')
if [[ "$actual" != "$expected" ]]; then
  echo "Imagem base inesperada: $actual (esperada: $expected)" >&2
  exit 1
fi
docker build -f docker/Dockerfile.ct_lio -t limo-real-ct-lio:humble .
