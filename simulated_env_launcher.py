#!/usr/bin/env python3
import os
import subprocess
import sys

def main():
    # Get parameters (from environment variables or command-line args)
    robot = os.environ.get("ROBOT", None)
    eef = os.environ.get("EEF", None)
    gripper = os.environ.get("GRIPPER", None)

    # Determine whether to add a gripper
    add_gripper = "true" if (eef or gripper) else "false"

    # Validate the robot type
    valid_robots = ["xarm5", "xarm6", "xarm7"]
    if not robot:
        print("❌ Error: ROBOT environment variable not set.")
        print(f"Valid options: {valid_robots}")
        sys.exit(1)
    elif robot not in valid_robots:
        print(f"❌ Error: Unsupported robot type '{robot}'. Valid options: {valid_robots}")
        sys.exit(1)

    # Construct the launch command
    launch_cmd = [
        "ros2", "launch", "xarm_moveit_config", f"{robot}_moveit_gazebo.launch.py"
    ]

    # Add the gripper argument if enabled
    if add_gripper == "true":
        launch_cmd.append(f"add_gripper:={add_gripper}")

    # Print what’s being launched
    print("=========================================")
    print("🚀 Launching Simulation Environment")
    print(f"🤖 Robot: {robot}")
    print(f"🦾 Add gripper: {add_gripper}")
    print("=========================================")

    # Execute the command
    try:
        subprocess.run(launch_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Launch failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
