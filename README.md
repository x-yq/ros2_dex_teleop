# xArm5 Teleoperation System — Usage Guide

## Overview

This repository contains a complete xArm5 teleoperation system built on ROS2 Humble. The system consists of four Docker containers plus two peripheral devices:

- **`teleop-system`, `teleop-components`, `teleop-drivers`** — the three core teleop containers. These can run on **any linux machine** (Linux, x86\_64 or arm64) that meets the prerequisites. They do not have to be on the same machine — you can spread them across multiple PCs by adjusting `hardware.yaml` and `deployment.yaml`.
- **`simulator` (Gazebo)** — must run on an **x86\_64 Ubuntu machine** due to Gazebo's architecture dependency. This is the only container with a hardware constraint.
- **VR Headset** and **xArm5 Robot** — peripheral devices connected over the network; they do not run containers.

All configuration files, Dockerfiles, launch files, and ROS2 packages are pre-configured. You only need to clone the repository, fill in your machine IPs and SSH credentials, and run the deployment script.

### System Architecture

The diagram below shows one common layout — all three teleop containers on one PC and the simulator on a separate x86 Ubuntu machine. You can redistribute the teleop containers across different machines by editing `deployment.yaml`.

```
Your PC (Linux, any arch)              Ubuntu x86 PC
┌────────────────────────┐              ┌─────────────────────────┐
│  teleop-system         │              │  simulator (Gazebo)     │
│  ├─ system_manager     │◄────────────►│  ├─ simulation_monitor  │
│  └─ data_collector     │   ROS2 DDS   │  └─ xarm_gazebo         │
├────────────────────────┤   (host net) └─────────────────────────┘
│  teleop-components     │
│  ├─ input_processor    │
│  ├─ motion_retarget    │
│  └─ feedback_processor │
├────────────────────────┤              xArm5 Robot
│  teleop-drivers        │◄────────────  192.168.x.x
│  ├─ vr_headset_driver  │              VR Headset
│  └─ xarm5_driver       │◄────────────  192.168.x.x
└────────────────────────┘

Note: teleop-system, teleop-components, and teleop-drivers can each
      run on different machines — the simulator is the only container
      with a strict x86_64 Ubuntu requirement.
```

---

## Prerequisites

### Hardware Requirements

| Container | Can run on | OS | Architecture |
|---|---|---|---|
| `teleop-system` | Any machine of your choice | Linux | x86\_64 or arm64 |
| `teleop-components` | Any machine of your choice | Linux | x86\_64 or arm64 |
| `teleop-drivers` | Any machine of your choice | Linux | x86\_64 or arm64 |
| `simulator` | **Dedicated x86 Ubuntu machine** | **Ubuntu 22.04** | **x86\_64 only** |

> **Flexibility:** The three teleop containers have no hardware constraints — you can run all of them on a single laptop, or split them across multiple machines. To change which machine a container runs on, update the `hardware_id` field for that container in `deployment.yaml` and add the machine to `hardware.yaml`.

> **Why x86 for Gazebo?** The Gazebo simulation container and its ROS2 Humble packages are built for x86\_64. Running on arm64 is not supported for the simulator container.

### Software Requirements (both machines)

- Docker Engine ≥ 24.0 and Docker Compose ≥ 2.20
- Python 3.8+
- `sshpass` (for password-based SSH from the deployment machine): `sudo apt install sshpass` or `brew install hudochenkov/sshpass/sshpass`
- `rsync` (usually pre-installed): `sudo apt install rsync`

### Network Requirements

- Both machines must be on the **same local network** (or VPN) so ROS2 DDS traffic can flow freely between them
- The VR headset and xArm5 robot must also be reachable from the primary PC on the same network
- All machines must use the **same `ROS_DOMAIN_ID`** (default: `0` — change this if other ROS2 systems are running on your network)

### Ubuntu Version Compatibility

ROS2 Humble officially targets **Ubuntu 22.04 (Jammy)**. The Docker images in this repository are built on `ros:humble-ros-base` which is Ubuntu 22.04-based. You do not need Ubuntu 22.04 on the host machine itself — Docker handles the OS layer

---

## VR Headset Setup

> 📌 **For full VR headset setup instructions, see the [Open-Teach VR Setup Guide](https://github.com/aadhithya14/Open-Teach/blob/main/docs/vr.md).**

### Download & Install

Download and sideload [**`SingleArmBot.apk`**](https://github.com/aadhithya14/Open-Teach/tree/main/VR/APK) onto your Meta Quest headset. Follow the sideloading steps in the Open-Teach guide linked above.

### Network Configuration

When the app prompts for an IP address, enter the IP of the **execution platform** — the machine running the `teleop-drivers` container (i.e. the PC connected to the robot), not the headset's own IP or the development machine.

### Motion Retargeting Controls

Once the system is running, the right hand is tracked and mapped to the robot arm. Keep the following gestures and posture in mind during operation:

| Gesture / Posture | Action |
|-------------------|--------|
| Thumb tip touches **pinky** tip | Toggle gripper **open / close** |
| Thumb tip touches **middle finger** tip | **Pause** motion and reset calibration; touch again to **resume** with new origin |
| Right hand movement (palm facing **down**) | Robot end-effector mimics hand pose |

> ⚠️ Keep your **palm facing down** throughout teleoperation. The retargeting coordinate mapping assumes this orientation — other palm orientations will produce unexpected robot motion.

---

## Step 1 — Clone the Repository

Clone the repository on **both machines** (the primary PC and the simulation PC). The deployment script will sync the repo automatically to remote machines, but it is still recommended to clone manually on both so you can inspect and edit config files locally.

```bash
git clone <repository-url>
cd <repository-name>
```

On the simulation PC (Ubuntu x86), you only need the repository present — the deployment script handles starting the container remotely from your primary machine.

---

## Step 2 — Configure `hardware.yaml`

Open `config_files/hardware.yaml`. This file contains the network addresses and SSH credentials for every machine in the system.

**Find your IP addresses:**
```bash
ip addr show | grep "inet " | grep -v 127.0.0.1
```

Fill in the values for your environment. Replace every `<...>` placeholder:

```yaml
execution_plattforms:

  - hardware_id: mac_pc
    execution_plattform: true
    ip: <your-primary-PC-IP>        # e.g. 192.168.1.45
    ssh_user: <your-username>        # Linux: whoami
    ssh_password: <your-password>

  - hardware_id: ubuntu
    execution_plattform: true
    ip: <your-ubuntu-x86-PC-IP>     # e.g. 192.168.1.72
    ssh_user: <ubuntu-username>
    ssh_password: <ubuntu-password>

sensors:
  - hardware_id: vr_headset
    execution_plattform: false
    driver_layer_cn_id: sensor_driver_rococo
    ip: <your-VR-headset-IP>         # e.g. 192.168.1.103

robots:
  - hardware_id: xarm5_robot
    execution_plattform: false
    driver_layer_cn_id: xarm5_robot
    ip: <your-xarm5-robot-IP>        # e.g. 192.168.1.108
```

> ⚠️ **Never commit this file with credentials filled in.** Add `config_files/hardware.yaml` to your `.gitignore`, or clear the credential fields before pushing.

---

## Step 3 — Verify and Adjust `deployment.yaml`

`config_files/deployment.yaml` maps each container to a machine via `hardware_id`. The default configuration places all three teleop containers on a single machine (`mac_pc`) and the simulator on the `ubuntu` machine:

```yaml
platforms:
  - hardware_id: mac_pc
    containers:
      - name: teleop-system
        node_count: 3

  - hardware_id: mac_pc
    containers:
      - name: teleop-components
        node_count: 3

  - hardware_id: mac_pc
    containers:
      - name: teleop-drivers
        node_count: 4

  - hardware_id: ubuntu
    containers:
      - name: simulator
        simulator_name: gazebo
        robot: xarm5
        eef: xarm_gripper
```

**If you want to run the teleop containers on different machines**, change the `hardware_id` for each container to match an entry you have defined in `hardware.yaml`. For example, to move `teleop-drivers` to a second machine registered as `ubuntu_pc2`:

```yaml
  - hardware_id: ubuntu_pc2      # ← changed from mac_pc
    containers:
      - name: teleop-drivers
        node_count: 4
```

Then add `ubuntu_pc2` to `hardware.yaml` with its IP and SSH credentials. The `simulator` container's `hardware_id` in `deployment.yaml` is ignored at deploy time — the simulator always deploys locally on whichever machine runs `deploy_system.py --launch-env` — so you do not need to change it.

---

## Step 4 — Verify Network Connectivity

Before deploying, confirm that your primary PC can reach all other machines:

```bash
# Ping the simulation PC
ping <your-ubuntu-x86-PC-IP>

# Ping the robot
ping <your-xarm5-robot-IP>

# Test SSH access to the simulation PC (enter password when prompted)
ssh <ubuntu-username>@<your-ubuntu-x86-PC-IP>
```

Also verify that Docker is running on both machines:

```bash
# Local
docker info

# Remote (via SSH)
ssh <ubuntu-username>@<your-ubuntu-x86-PC-IP> "docker info"
```

---

## Step 5 — Deploy the Teleop Containers

Run the deployment script from the repository root on your **primary PC**. This will SSH into the machines defined in `hardware.yaml`, sync the repository, build the Docker images, and start the containers.

```bash
# Deploy all non-simulator containers (teleop-system, teleop-components, teleop-drivers)
python3 deploy_system.py --ros-domain-id 0
```

The script will:
1. SSH into the primary PC (or run locally if `--local-mode` is set)
2. Sync the repository to the remote machine via `rsync`
3. Run `docker compose up -d` for each container in sequence
4. Print container logs (last 10 lines) after each successful start

**If you are running everything on a single machine** (no remote deployment), add `--local-mode`:

```bash
python3 deploy_system.py --ros-domain-id 0 --local-mode
```

**If the remote machine already has the repository** from a previous run and you want to force a fresh sync:

```bash
python3 deploy_system.py --ros-domain-id 0 --force-update
```

Expected output (per container):
```
--- Deploying container: teleop-system ---
✓ Successfully deployed container: teleop-system
--- Deploying container: teleop-components ---
✓ Successfully deployed container: teleop-components
--- Deploying container: teleop-drivers ---
✓ Successfully deployed container: teleop-drivers
```

---

## Step 6 — Deploy the Simulator

On your **x86 Ubuntu host**, clone the repository and run:
```bash
# Allow Docker containers to access the host display
xhost +local:
export DISPLAY=:0

# Deploy the simulator container locally
python3 deploy_system.py --launch-env --ros-domain-id 0
```

Then enter the container and launch:
```bash
docker exec -it <simualtor_container_name> bash
source /root/dev_ws/install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch xarm_gazebo xarm5_beside_table_gazebo.launch.py
```

> ⚠️ **First-launch issue:** If you see `Waiting for spawn_entity Service` stuck in the terminal, kill the hung `gzserver` process (`ps -a | grep gzserver` → `kill -9 <PID>`) and rerun the launch file.

---

## Step 7 — Verify All Containers Are Running

Check that all four containers are up on their respective machines:

```bash
# On the primary PC
docker ps --format "table {{.Names}}\t{{.Status}}"

# Expected output:
# NAMES                STATUS
# teleop-system        Up X minutes
# teleop-components    Up X minutes
# teleop-drivers       Up X minutes

# On the Ubuntu x86 machine
docker ps --format "table {{.Names}}\t{{.Status}}"
# NAMES                STATUS
# simulator            Up X minutes
```

If a container shows `Restarting` or `Exited`, inspect its logs:

```bash
docker logs --tail 50 <container-name>
```

---

## Step 8 — Launch ROS2 Nodes

Containers start with `tail -f /dev/null` and do not run any ROS2 nodes automatically. You must exec into each container and run the appropriate launch command.

Open a separate terminal for each container:

**Terminal 1 — System Manager** (on primary PC):
```bash
docker exec -it teleop-system bash
ros2 launch teleop_launcher system_manager.launch.py ros_domain_id:=0
```

**Terminal 2 — Component Nodes** (on primary PC):
```bash
docker exec -it teleop-components bash
ros2 launch teleop_launcher component_nodes.launch.py ros_domain_id:=0
```

**Terminal 3 — Driver Nodes** (on primary PC):
```bash
docker exec -it teleop-drivers bash
ros2 launch teleop_launcher driver_nodes.launch.py ros_domain_id:=0
```

**Terminal 4 — Simulator** (on Ubuntu x86 machine):
> 📌 **Coming soon** — see Step 6.

After launching all nodes, verify they are visible on the ROS2 network from any container:

```bash
# Inside any running container
ros2 node list
```

You should see all nodes including the monitor nodes and all component and driver nodes.

---

## Step 9.1 — Start / Pause the System

The system is controlled via the `/system_control` service, which broadcasts lifecycle commands to all registered component nodes simultaneously.

**Start the system:**
```bash
ros2 service call \
  /system_control \
  system_interfaces/srv/SystemControl \
  "{description: 'start'}"
```

**Pause the system:**
```bash
ros2 service call \
  /system_control \
  system_interfaces/srv/SystemControl \
  "{description: 'pause'}"
```

**Emergency stop** (immediately halts all components and sets them to `disconnected`):
```bash
ros2 service call \
  /system_control \
  system_interfaces/srv/SystemControl \
  "{description: 'urgent_stop'}"
```

A successful response looks like:
```
success: true
system_state: running        # or 'ready' after pause
message: "..."
```

---

## Step 9.2 — Start / Stop Monitoring

Each container has a dedicated monitor node. You must call `start_monitor` on each one individually **before** starting the system, and `stop_monitor` **after** pausing it. Both services require the **full** Docker container ID (not the short hash).

**Get full container IDs first:**
```bash
docker ps --no-trunc --format "table {{.Names}}\t{{.ID}}"
```

**Start all monitors:**
```bash
ros2 service call /monitor_ubuntu_simulator/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_system/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_drivers/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_components/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['']}"
```

**Stop all monitors** (this triggers metric computation and flushes logs):
```bash
ros2 service call /monitor_ubuntu_simulator/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_system/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_drivers/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['']}"

ros2 service call /monitor_ubuntu_teleop_components/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['']}"
```

> ⚠️ **Order matters:** Always start monitors *before* starting the system, and stop monitors *after* pausing it. Stopping a monitor immediately flushes and closes the stats stream — any metrics collected after that point will be lost.

> 📄 KPI logs are written to `/log/monitor_<container_id>.log` inside the monitor's container. For the full recommended sequence, see `monitor_test.bash`.

---


## Step 9.3 — Start / Stop Data Collection


Unlike the per-container monitor nodes, data collection is controlled through a single pair of global services: `/start_data_collection` and `/stop_data_collection`. Both take a `demo_id` string that identifies the session, and this same identifier is used to name the output files.

**Start a recording session:**

```bash
ros2 service call \
  /start_data_collection \
  system_interfaces/srv/StartDataCollection \
  "{demo_id: 'example'}"
```

**Stop a recording session:**

```bash
ros2 service call \
  /stop_data_collection \
  system_interfaces/srv/StopDataCollection \
  "{demo_id: 'example'}"
```

> ⚠️ **Order matters:** call `start_data_collection` *before* `start` on `/system_control`, and call `stop_data_collection` *after* `pause`. This way the recorded window covers the full teleoperation run rather than missing the first or last messages.

> 📄 If you need to override the default topic list or storage path for a specific session instead of using the values in `config_files/demo.yaml`, check the `StartDataCollection` srv definition for optional fields — the basic call above only needs `demo_id`.

A full run looks like this:

```bash
DEMO="Example"

ros2 service call /start_data_collection system_interfaces/srv/StartDataCollection "{demo_id: '${DEMO}'}"
sleep 3

ros2 service call /system_control system_interfaces/srv/SystemControl "{description: 'start'}"
# ... perform teleoperation ...
ros2 service call /system_control system_interfaces/srv/SystemControl "{description: 'pause'}"

ros2 service call /stop_data_collection system_interfaces/srv/StopDataCollection "{demo_id: '${DEMO}'}"
sleep 5
```

For details on how topics map to recorded features, and how to change the default topic list and storage path, see the [Data Collection doc](config_files/docs/data_collector.md).

---

## Step 10 — Stop the System

### Stop ROS2 Nodes

In each terminal running a launch file, press `Ctrl+C` to gracefully shut down the nodes. Then exit the container:

```bash
exit
```

### Stop Containers

To stop all teleop containers on the primary PC:

```bash
python3 deploy_system.py --stop --ros-domain-id 0
```

To stop the simulator container (run on the Ubuntu x86 machine):

```bash
python3 deploy_system.py --launch-env --stop
```

Or stop containers directly with Docker:

```bash
docker stop teleop-system teleop-components teleop-drivers
docker stop simulator
```

---

<!-- ## Troubleshooting

### Container fails to start

```bash
docker logs --tail 100 <container-name>
```

Common causes: missing volume paths (e.g. `/tmp/argus_socket` does not exist yet — create it with `mkdir -p /tmp/argus_socket`), or Docker image build failure during first run.

### ROS2 nodes cannot see each other across machines

- Confirm all machines use the same `ROS_DOMAIN_ID`
- Confirm `network_mode: host` is set in `docker-compose.yaml` for all services
- Check that no firewall is blocking UDP multicast traffic between the machines:
  ```bash
  sudo ufw status
  # If active, allow ROS2 DDS ports:
  sudo ufw allow 7400:7500/udp
  ```
- Try pinging between containers using host IPs to confirm basic connectivity

### SSH deployment fails

- Verify `sshpass` is installed on the machine running `deploy_system.py`
- Double-check the `ip`, `ssh_user`, and `ssh_password` values in `hardware.yaml`
- Test SSH manually: `ssh <user>@<ip>`
- If using key-based auth instead of a password, leave `ssh_password` blank and ensure your key is loaded in `ssh-agent`

### Simulator container fails to open Gazebo window

- Ensure `DISPLAY` is set correctly on the Ubuntu x86 host: `echo $DISPLAY`
- Run `xhost +local:docker` on the host before starting the container
- Verify the host is x86\_64: `uname -m` (must output `x86_64`)

### `docker compose` not found (vs `docker-compose`)

The deployment script uses the modern `docker compose` (with a space, Docker Compose V2). If your system has the older `docker-compose` binary instead, upgrade Docker Engine or install the Compose plugin:

```bash
sudo apt install docker-compose-plugin
``` -->

---

## Quick Reference

### Deployment

```bash
# ── Deploy ────────────────────────────────────────────────────────────
# Deploy teleop containers (remote)
python3 deploy_system.py --ros-domain-id 0

# Deploy teleop containers (local machine only)
python3 deploy_system.py --ros-domain-id 0 --local-mode

# Deploy simulator — run on Ubuntu x86 machine (see Step 6 for full setup)
python3 deploy_system.py --launch-env --ros-domain-id 0

# Force re-sync repo to remote before deploying
python3 deploy_system.py --ros-domain-id 0 --force-update

# ── Launch nodes (inside each container) ────────────────────────────
docker exec -it teleop-system     bash -c "source /opt/ros/humble/setup.bash && source /root/dev_ws/install/setup.bash && ros2 launch teleop_launcher system_manager.launch.py ros_domain_id:=0"
docker exec -it teleop-components bash -c "source /opt/ros/humble/setup.bash && source /root/dev_ws/install/setup.bash && ros2 launch teleop_launcher component_nodes.launch.py ros_domain_id:=0"
docker exec -it teleop-drivers    bash -c "source /opt/ros/humble/setup.bash && source /root/dev_ws/install/setup.bash && ros2 launch teleop_launcher driver_nodes.launch.py ros_domain_id:=0"
# simulator launch — coming soon, see Step 6

# ── Stop ─────────────────────────────────────────────────────────────
python3 deploy_system.py --stop
python3 deploy_system.py --launch-env --stop
```

### Monitoring

```bash
# Show name alongside full ID
docker ps --no-trunc --format "table {{.Names}}\t{{.ID}}"

# Start monitor for one container
ros2 service call \
  /<monitor_node_name>/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['<full_container_id>']}"

## Example
ros2 service call \
  /monitor_ubuntu_simulator/start_monitor \
  system_interfaces/srv/StartMonitor \
  "{container_ids: ['f962a026f2e1049d7fc4770703f3f742ecde8c7c18e49e721d8755f8ceee2429']}"

# Stop monitor for one container
ros2 service call \
  /<monitor_node_name>/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['<full_container_id>']}"

## Example
ros2 service call \
  /monitor_ubuntu_simulator/stop_monitor \
  system_interfaces/srv/StopMonitor \
  "{container_ids: ['f962a026f2e1049d7fc4770703f3f742ecde8c7c18e49e721d8755f8ceee2429']}"
```

---

## What's More

For more details please see the following docs.

- **Config and deploy a new teleoperation system using this framework** - [Doc](config_files/docs/config_deploy.md)
- **System Manager and Component Node** - [Doc](config_files/docs/systemManager_component_node.md.md)
- **ROS2 Interface** — [Doc](config_files/docs/ros2_interface.md)
- **Resource Monitoring** — [Doc](config_files/docs/monitor_node.md)
- **Latency Measurement** —  `5. Latency Measurement: End-to-End Setup Guide` in the [Doc](config_files/docs/systemManager_component_node.md)

- **Data Collection** — how to configure and trigger the `data_collector` node, see the guide in the [Doc](config_files/docs/data_collector.md)
- **Components Descriptions** — what each ROS2 package, i.e. the how pipeline does, see the [Doc]((config_files/docs/components.md))

- **Simulator Setup** — full Gazebo environment setup, display configuration, and xArm5 model installation on Ubuntu x86, see the [Doc](config_files/docs/xarm5_gazebo_simulation.md)
