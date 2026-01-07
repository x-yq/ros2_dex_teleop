#!/usr/bin/env bash

set -euo pipefail

############################
# Configuration
############################

# Container IDs (FULL IDs as required by your services)
SIMULATOR_ID="f962a026f2e1049d7fc4770703f3f742ecde8c7c18e49e721d8755f8ceee2429"
SYSTEM_ID="6334e2379e3cbc04dae10a496473eec6ae7cd86795d5228c0b538d5f0503ce93"
DRIVERS_ID="db017a78854947b7d2bc8b6969dcf7769d4a59a43ca89aa115a0cb4f1669a83c"
COMPONENTS_ID="4ed5ebb0fed76dcf77a686809e36156573f5c0c057c9b5e397198e7261f706d7"

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

