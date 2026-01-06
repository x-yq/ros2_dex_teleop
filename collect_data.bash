#!/usr/bin/env bash

set -euo pipefail

############################
# Configuration
############################

DEMO="HZ_90"

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
# Start data collector
############################

ros2 service call /start_data_collection system_interfaces/srv/StartDataCollection "{demo_id: '${DEMO}'}"
sleep 3

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
# Stop data collector
############################

ros2 service call /stop_data_collection system_interfaces/srv/StopDataCollection "{demo_id: '${DEMO}'}"

sleep 5

echo "✅ All monitors stopped. Done."
