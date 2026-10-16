#!/bin/bash
set -e
source /opt/ros/melodic/setup.bash
if [ -f /ws/devel/setup.bash ]; then source /ws/devel/setup.bash; fi
if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi
exec "$@"
