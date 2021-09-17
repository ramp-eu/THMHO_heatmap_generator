#!/bin/bash
set -e

# setup ros environment
export TURTLEBOT3_MODEL=burger
cd /
source "$CATKIN_WS/install/setup.bash"
exec "$@"
