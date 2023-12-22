#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "install/setup.bash"
source "install/local_setup.bash"
exec "$@"