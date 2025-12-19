#!/usr/bin/env bash
set -e

# Source ROS
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace install if available
if [ -f "/home/dev/ros_ws/install/setup.bash" ]; then
    source /home/dev/ros_ws/install/setup.bash
fi

# Export X11 display by default
export DISPLAY=${DISPLAY:-host.docker.internal:0.0}
export QT_X11_NO_MITSHM=1

# Set ROS domain ID for DDS discovery
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Helpful aliases
alias cb='colcon build --symlink-install'
alias cbt='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias src='source install/setup.bash'

exec "$@"

