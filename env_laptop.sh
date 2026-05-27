#!/usr/bin/env bash
# Source on the laptop or inside Docker when monitoring ROS2 topics.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
unset FASTRTPS_DEFAULT_PROFILES_FILE
echo "ROS2 env: Domain=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION} Localhost=${ROS_LOCALHOST_ONLY}"
