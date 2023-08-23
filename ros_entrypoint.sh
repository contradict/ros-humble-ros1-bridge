#!/bin/bash
set -e

# setup ros2 environment
source "/ros2_iron/install/local_setup.bash" --

exec "$@"
