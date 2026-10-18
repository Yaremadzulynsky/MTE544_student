#!/usr/bin/env bash
# Local TurtleBot3 simulation environment; no TurtleBot4 lab DDS profile.
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE ROS_DISCOVERY_SERVER

if ss -H -ltn 'sport = :8765' | read -r _; then
    echo 'Port 8765 is already listening. Stop the existing bridge before restarting.' >&2
    exit 1
fi
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765 use_sim_time:=true
