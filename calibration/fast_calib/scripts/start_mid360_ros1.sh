#!/bin/bash
set -euo pipefail
ip -4 addr show | grep -q '192.168.1.50/' || { echo 'Configure primeiro o IP Ethernet 192.168.1.50.' >&2; exit 1; }
if ss -H -uan | grep -Eq ':(56101|56201|56301|56401)[[:space:]]'; then
 echo 'Portas Livox ocupadas; pare manualmente o driver concorrente.' >&2; exit 1
fi
exec roslaunch /calibration/config/mid360.launch
