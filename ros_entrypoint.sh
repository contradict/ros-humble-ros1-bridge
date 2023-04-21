#!/bin/bash
set -e

# setup ros2 environment
source "/colcon_ws/install/setup.bash" --

exec "$@"
