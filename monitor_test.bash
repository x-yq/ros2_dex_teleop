#!/usr/bin/env bash

set -euo pipefail

############################
# Configuration
############################

# Container IDs (FULL IDs as required by your services)
SIMULATOR_ID="f962a026f2e1049d7fc4770703f3f742ecde8c7c18e49e721d8755f8ceee2429"
SYSTEM_ID="47baa4bd5388b59f4baddad9d2f17feff758c72135b270f24a75e229e7d4b737"
DRIVERS_ID="752b16673b2d5f5119a36d7606b1879ed80eb0252a517bb6d1705eb0650083af"
COMPONENTS_ID="9ddf97dba7eeac3827c9d4696453f9470aa7e3ed19e194f6631875a99e7fec96"

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
