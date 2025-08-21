#!/bin/bash

# ROS2 System Launch Script
# This script runs all system nodes individually using ros2 run

# Set ROS domain ID
export ROS_DOMAIN_ID=0

# Set config paths
export CONFIG_DIR="/root/dev_ws/src/config_files"
export COMPONENT_FILE_PATH="/root/dev_ws/src/config_files/component_node"

echo "Starting ROS2 system nodes..."
echo "Config directory: $CONFIG_DIR"
echo "Component file path: $COMPONENT_FILE_PATH"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# Function to run node in background and store PID
run_node() {
    local package=$1
    local executable=$2
    local node_name=$3
    local extra_params=$4
    
    echo "Starting $node_name..."
    if [ -n "$extra_params" ]; then
        ros2 run $package $executable $extra_params &
    else
        ros2 run $package $executable &
    fi
    
    local pid=$!
    echo "  -> PID: $pid"
    echo $pid >> /tmp/ros2_system_pids.txt
    sleep 1
}

# Clean up any existing PID file
rm -f /tmp/ros2_system_pids.txt

echo "=== Starting System Nodes ==="
echo ""

# 1. System Manager (doesn't use Hydra)
run_node "system_manager" "system_manager_node" "System Manager" "--ros-args -p config_folder_path:=$CONFIG_DIR"

# 2. Input Sensor Driver (Rococo)
run_node "input_sensor_driver" "input_sensor_driver_node" "Input Sensor Driver (Rococo)"

# 3. Input Data Processor
run_node "input_data_processor" "input_data_processor_node" "Input Data Processor"

# 4. Motion Retarget
run_node "motion_retarget" "motion_retarget_node" "Motion Retarget"

# 5. Robot Driver
run_node "robot_driver" "robot_driver_node" "Robot Driver"

# 6. Feedback Sensor Driver (Realsense)
run_node "feedback_sensor_driver" "feedback_sensor_driver_node" "Feedback Sensor Driver (Realsense)"

# 7. Feedback Data Processor
run_node "feedback_data_processor" "feedback_data_processor_node" "Feedback Data Processor"

# 8. Feedback Projector Driver
run_node "feedback_projector_driver" "feedback_projector_driver_node" "Feedback Projector Driver"

# 9. KPI Monitor
run_node "kpi_monitor" "monitor_0" "KPI Monitor"

# Optional: Data Collector (uncomment if needed)
# run_node "data_collector" "data_collector_node" "Data Collector"

echo ""
echo "=== All nodes started ==="
echo ""
echo "Node PIDs stored in: /tmp/ros2_system_pids.txt"
echo ""
echo "To stop all nodes, run:"
echo "  kill \$(cat /tmp/ros2_system_pids.txt)"
echo ""
echo "Press Ctrl+C to stop this script and all nodes..."

# Function to cleanup on script exit
cleanup() {
    echo ""
    echo "Stopping all nodes..."
    if [ -f /tmp/ros2_system_pids.txt ]; then
        while read pid; do
            if kill -0 $pid 2>/dev/null; then
                echo "Killing process $pid"
                kill $pid
            fi
        done < /tmp/ros2_system_pids.txt
        rm -f /tmp/ros2_system_pids.txt
    fi
    echo "All nodes stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Wait for all background processes
wait