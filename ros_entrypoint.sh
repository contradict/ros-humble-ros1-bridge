#!/bin/bash
set -e

# setup ros2 environment
source "/ros1_bridge/install/setup.bash" --

exec "$@"
