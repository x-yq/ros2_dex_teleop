#!/bin/bash

echo "Force stopping all ROS2 system nodes..."

# Method 1: Kill by exact process names from ps output
echo "Killing processes by name..."
pkill -f "input_sensor_dr" -SIGKILL
pkill -f "input_data_proc" -SIGKILL
pkill -f "feedback_sensor" -SIGKILL
pkill -f "feedback_data_p" -SIGKILL
pkill -f "feedback_projec" -SIGKILL
pkill -f "monitor_0" -SIGKILL
pkill -f "robot_driver_no" -SIGKILL
pkill -f "motion_retarget" -SIGKILL
pkill -f "system_manager" -SIGKILL

sleep 1

# Method 2: Kill by PID if we have the file
if [ -f /tmp/ros2_system_pids.txt ]; then
    echo "Killing processes from PID file..."
    while read pid; do
        if [ -n "$pid" ] && kill -0 $pid 2>/dev/null; then
            echo "Force killing PID: $pid"
            kill -9 $pid 2>/dev/null
        fi
    done < /tmp/ros2_system_pids.txt
    rm -f /tmp/ros2_system_pids.txt
fi

sleep 1

# Method 3: Find and kill by specific PIDs from your ps output
echo "Finding and killing specific ROS2 processes..."

# Get all python processes that might be ROS nodes
pids=$(ps aux | grep python3 | grep -E "(input_|feedback_|robot_|monitor|motion_|system_)" | awk '{print $2}')

if [ -n "$pids" ]; then
    echo "Found PIDs: $pids"
    for pid in $pids; do
        if kill -0 $pid 2>/dev/null; then
            echo "Killing PID: $pid"
            kill -9 $pid 2>/dev/null
        fi
    done
fi


sleep 2

# Method 5: Nuclear option - kill all python3 processes (be careful!)
echo ""
echo "If nodes are still running, you may need to kill all python3 processes:"
echo "WARNING: This will kill ALL python3 processes!"
echo "Run this command manually if needed:"
echo "  killall -9 python3"
echo ""

# Check what's still running
echo "Checking remaining processes..."
remaining=$(ps aux | grep -E "(input_|feedback_|robot_|monitor|motion_|system_)" | grep -v grep)
if [ -n "$remaining" ]; then
    echo "Still running:"
    echo "$remaining"
    echo ""
    echo "Manual cleanup needed. Run:"
    echo "  killall -9 python3"
else
    echo "All processes stopped successfully!"
fi

# Final check
echo ""
echo "Final ROS2 node check:"
ros2 node list 2>/dev/null || echo "No nodes running (good!)"