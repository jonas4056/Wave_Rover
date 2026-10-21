#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /opt/thirdparty_ws/install/setup.bash
if [ -f "$HOME/wave_rover/install/setup.bash" ]; then
    source "$HOME/wave_rover/install/setup.bash"
fi

exec "$@"
