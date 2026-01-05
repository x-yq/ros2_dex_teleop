#!/usr/bin/env bash

set -euo pipefail

############################
# Configuration
############################

# ROS Domain (can be overridden externally)
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Docker container NAMES
SIMULATOR_CONTAINER="config_files-simulator-1"
SYSTEM_CONTAINER="teleop-system"
DRIVERS_CONTAINER="teleop-drivers"
COMPONENTS_CONTAINER="teleop-components"

############################
# Helper function
############################

launch_in_container() {
  local container_name=$1
  local launch_cmd=$2

  echo "▶ Launching in container: ${container_name}"

  docker exec -d "${container_name}" bash -c "
    set -e
    source /opt/ros/humble/setup.bash
    source /root/dev_ws/install/setup.bash
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    ${launch_cmd}
  "
}

############################
# Launch processes
############################

# Teleop system manager
launch_in_container \
  "${SYSTEM_CONTAINER}" \
  "ros2 launch teleop_launcher system_manager.launch.py ros_domain_id:=\${ROS_DOMAIN_ID}"

# Teleop components
launch_in_container \
  "${COMPONENTS_CONTAINER}" \
  "ros2 launch teleop_launcher component_nodes.launch.py ros_domain_id:=\${ROS_DOMAIN_ID}"

# Teleop drivers
launch_in_container \
  "${DRIVERS_CONTAINER}" \
  "ros2 launch teleop_launcher driver_nodes.launch.py ros_domain_id:=\${ROS_DOMAIN_ID}"

# Simulator / monitor
launch_in_container \
  "${SIMULATOR_CONTAINER}" \
  "export CONTAINER_NAME=simulator && ros2 launch teleop_launcher simulation_monitor.launch.py ros_domain_id:=\${ROS_DOMAIN_ID}"

echo " All ROS 2 launch files started successfully."
