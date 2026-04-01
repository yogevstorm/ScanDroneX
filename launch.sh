#!/bin/bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source "$WORKSPACE_DIR/install/setup.bash"

# Kill any leftover Gazebo processes from previous runs
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 1

exec ros2 launch scandronex_sim scandronex_sim.launch.py "$@"
