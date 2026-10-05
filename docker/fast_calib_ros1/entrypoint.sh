#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/fast_calib_ws/devel/setup.bash
exec "$@"
