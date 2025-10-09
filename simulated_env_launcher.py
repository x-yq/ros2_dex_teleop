# #!/usr/bin/env python3
# import os
# import subprocess
# import sys

# def main():
#     # Get parameters (from environment variables or command-line args)
#     robot = os.environ.get("ROBOT", None)
#     eef = os.environ.get("EEF", None)
#     gripper = os.environ.get("GRIPPER", None)

#     # Determine whether to add a gripper
#     add_gripper = "true" if (eef or gripper) else "false"

#     # Validate the robot type
#     valid_robots = ["xarm5", "xarm6", "xarm7"]
#     if not robot:
#         print("❌ Error: ROBOT environment variable not set.")
#         print(f"Valid options: {valid_robots}")
#         sys.exit(1)
#     elif robot not in valid_robots:
#         print(f"❌ Error: Unsupported robot type '{robot}'. Valid options: {valid_robots}")
#         sys.exit(1)

#     # Construct the launch command
#     launch_cmd = [
#         "ros2", "launch", "xarm_moveit_config", f"{robot}_moveit_gazebo.launch.py"
#     ]

#     # Add the gripper argument if enabled
#     if add_gripper == "true":
#         launch_cmd.append(f"add_gripper:={add_gripper}")

#     # Print what’s being launched
#     print("=========================================")
#     print("🚀 Launching Simulation Environment")
#     print(f"🤖 Robot: {robot}")
#     print(f"🦾 Add gripper: {add_gripper}")
#     print("=========================================")

#     # Execute the command
#     try:
#         subprocess.run(launch_cmd, check=True)
#     except subprocess.CalledProcessError as e:
#         print(f"❌ Launch failed: {e}")
#         sys.exit(1)

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import os
import subprocess
import sys
import time
import signal
import psutil  # ensure psutil is installed (add to Dockerfile if not)

def kill_gzserver():
    """Find and kill all running gzserver processes."""
    killed = []
    for proc in psutil.process_iter(['pid', 'name']):
        if "gzserver" in proc.info['name']:
            try:
                proc.kill()
                killed.append(proc.info['pid'])
            except psutil.NoSuchProcess:
                continue
    if killed:
        print(f"🧹 Killed gzserver processes: {killed}")
    else:
        print("ℹ️ No gzserver process found to kill.")

def launch_simulation(robot, add_gripper):
    """Run ros2 launch for the selected robot."""
    launch_cmd = [
        "ros2", "launch", "xarm_moveit_config", f"{robot}_moveit_gazebo.launch.py"
    ]
    if add_gripper == "true":
        launch_cmd.append(f"add_gripper:={add_gripper}")

    print("=========================================")
    print("🚀 Launching Simulation Environment")
    print(f"🤖 Robot: {robot}")
    print(f"🦾 Add gripper: {add_gripper}")
    print("=========================================")

    process = subprocess.Popen(
        launch_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )

    return process

def main():
    # Get parameters
    robot = os.environ.get("ROBOT")
    eef = os.environ.get("EEF")
    gripper = os.environ.get("GRIPPER")

    add_gripper = "true" if (eef or gripper) else "false"

    valid_robots = ["xarm5", "xarm6", "xarm7"]
    if not robot or robot not in valid_robots:
        print(f"❌ Error: Invalid or missing ROBOT (choose from {valid_robots})")
        sys.exit(1)

    retry_once = False
    max_retries = 1

    for attempt in range(max_retries + 1):
        process = launch_simulation(robot, add_gripper)

        hang_detected = False
        try:
            # Monitor for hang message
            for line in process.stdout:
                print(line, end="")  # stream output
                if "Waiting for spawn_entity" in line:
                    hang_detected = True
                    print("⚠️  Detected Gazebo spawn hang — gzserver might be unresponsive.")
                    break
        except KeyboardInterrupt:
            process.terminate()
            sys.exit(0)

        if hang_detected:
            # Wait briefly for Gazebo window
            print("🕐 Waiting 10s for Gazebo window to open before restart...")
            time.sleep(10)

            # Kill gzserver and relaunch
            kill_gzserver()
            print("🔁 Restarting the simulation launch...")
            time.sleep(5)
            retry_once = True
            continue
        else:
            # If process exited normally or user stopped it
            ret = process.wait()
            if ret == 0:
                print("✅ Simulation exited normally.")
            else:
                print(f"❌ Simulation exited with code {ret}.")
            break

    if retry_once:
        print("✅ Relaunch complete. Check Gazebo for proper model spawn.")

if __name__ == "__main__":
    main()
