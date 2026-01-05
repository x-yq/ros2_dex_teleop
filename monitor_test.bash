#!/usr/bin/env bash

set -euo pipefail

############################
# Configuration
############################

# Container IDs (FULL IDs as required by your services)
SIMULATOR_ID="dd11e2af62b1971335f7e774bdfccee5102459eb5b9ff13e1e3bc02b250942a5"
SYSTEM_ID="a1c3dfc3e9ea80c92e1da3c3df153c4b32ba758c205a601bde170e874f1f5596"
DRIVERS_ID="c4473d0928ff3f9dd1772537a5da80e2c6fc08f7beb7bb037b366bea542e3147"
COMPONENTS_ID="57ad6baaa0e8e1c999c54bb1f8b2f93486fb736f17f28da6b540ff5fe11adc14"

############################
# Helper functions
############################

start_monitor() {
  local service_name=$1
  local container_id=$2

  echo "▶ Starting monitor: ${service_name}"
  ros2 service call \
    "${service_name}/start_monitor" \
    system_interfaces/srv/StartMonitor \
    "{container_ids: ['${container_id}']}"
}

stop_monitor() {
  local service_name=$1
  local container_id=$2

  echo "■ Stopping monitor: ${service_name}"
  ros2 service call \
    "${service_name}/stop_monitor" \
    system_interfaces/srv/StopMonitor \
    "{container_ids: ['${container_id}']}"
}

############################
# Start monitors
############################

start_monitor "/monitor_ubuntu_simulator"        "${SIMULATOR_ID}"
sleep 1
start_monitor "/monitor_ubuntu_teleop_system"    "${SYSTEM_ID}"
sleep 1
start_monitor "/monitor_ubuntu_teleop_drivers"   "${DRIVERS_ID}"
sleep 1
start_monitor "/monitor_ubuntu_teleop_components" "${COMPONENTS_ID}"
sleep 1

############################
# Start system
############################

echo "▶ Starting system"
ros2 service call \
  /system_control \
  system_interfaces/srv/SystemControl \
  "{description: 'start'}"

############################
# Wait
############################

echo "System running for 60 seconds..."
sleep 60

############################
# Pause system
############################

echo "⏸ Pausing system"
ros2 service call \
  /system_control \
  system_interfaces/srv/SystemControl \
  "{description: 'pause'}"

############################
# Stop monitors
############################

stop_monitor "/monitor_ubuntu_simulator"         "${SIMULATOR_ID}"
sleep 1
stop_monitor "/monitor_ubuntu_teleop_system"     "${SYSTEM_ID}"
sleep 1
stop_monitor "/monitor_ubuntu_teleop_drivers"    "${DRIVERS_ID}"
sleep 1
stop_monitor "/monitor_ubuntu_teleop_components" "${COMPONENTS_ID}"

echo "✅ All monitors stopped. Done."
