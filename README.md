# xArm5 Teleoperation System — Usage Guide

> **Note:** This document is a work in progress. Future sections will cover individual package descriptions, resource usage monitoring, latency measurement, data collection, and system start/pause control via ROS2 service calls. For now this guide focuses on getting the system deployed and running.

---

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

> 📌 **Coming soon:** The Gazebo simulation environment requires additional setup (x86\_64 Ubuntu host, display configuration, xArm5 Gazebo model installation). A dedicated setup and deployment guide for the simulator will be added here.
>
> In the meantime, the short version: SSH into your Ubuntu x86 machine, clone the repository there, and run:
> ```bash
> python3 deploy_system.py --launch-env --ros-domain-id 0
> ```
> The `--launch-env` flag always deploys the simulator locally on the machine where the command is run. The `hardware_id` of the simulator entry in `deployment.yaml` is not used for routing — it is only metadata.

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

## Step 9 — Start / Pause the System

> 📌 **Coming soon:** Detailed instructions for starting and pausing the teleoperation session via ROS2 service calls will be added here once the service interface is finalized.

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

## VR Headset Setup

> 📌 **Coming soon:** This section will cover pairing and configuring the VR headset for use with the teleoperation system, including network setup, driver installation, and verifying the headset stream is reachable by the `teleop-drivers` container.

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
docker exec -it teleop-system     bash -c "ros2 launch teleop_launcher system_manager.launch.py ros_domain_id:=0"
docker exec -it teleop-components bash -c "ros2 launch teleop_launcher component_nodes.launch.py ros_domain_id:=0"
docker exec -it teleop-drivers    bash -c "ros2 launch teleop_launcher driver_nodes.launch.py ros_domain_id:=0"
# simulator launch — coming soon, see Step 6

# ── Monitor ──────────────────────────────────────────────────────────
docker ps                               # check container status
docker logs --tail 50 <container-name>  # container logs
ros2 node list                          # list all active ROS2 nodes (inside any container)
ros2 topic list                         # list all active ROS2 topics

# ── Stop ─────────────────────────────────────────────────────────────
python3 deploy_system.py --stop
python3 deploy_system.py --launch-env --stop
```
---

## What's Next

The following sections will be added to this guide:

- **Simulator Setup** — full Gazebo environment setup, display configuration, and xArm5 model installation on Ubuntu x86
- **VR Headset Setup** — pairing, driver installation, and network configuration for the VR headset
- **System Start / Pause** — ROS2 service calls to start and pause the teleoperation session without restarting containers or nodes
- **Package Descriptions** — what each ROS2 package does and how the nodes communicate
- **Resource Monitoring** — how to use the `kpi_monitor` to track CPU, memory, and network usage per node
- **Latency Measurement** — tools and procedures for measuring end-to-end teleoperation latency
- **Data Collection** — how to configure and trigger the `data_collector` node
