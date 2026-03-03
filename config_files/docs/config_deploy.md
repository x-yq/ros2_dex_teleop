# Configuration & Deployment Guide

## Overview

This system deploys a distributed ROS2 teleoperation stack across multiple machines using Docker containers. Three YAML configuration files work together to describe **what** to deploy, **where** to deploy it, and **how** the containers are built. A single Python script (`deploy_system.py`) reads all three and orchestrates the full deployment — locally or over SSH.

```
config_files/
├── hardware.yaml       # Physical machines, sensors, robots (IPs, SSH credentials)
├── deployment.yaml     # Which containers go on which hardware
└── docker-compose.yaml # How each container is built and configured
deploy_system.py        # Deployment orchestrator
docker/
├── Dockerfile.base       # Dockerfile template
├── Dockerfile.sysManager  # Dockerfile for system manager container
└── ...                    # Other Dockerfiles defined by user
teleop_launcher/        # Launcher package for storing all launch files
└── launcher/               
    ├── base.launch.py       # Launch file template
    ├── Dockerfile.sysManager  # Launch file for system manager container
    └── ...                    # Other launch files defined by user
```

---

## Configuration Files

### 1. `hardware.yaml`

**Purpose:** Registry of all physical hardware in the system. Provides network addresses and SSH credentials that `deploy_system.py` uses to reach each machine.

**Used by:** `deploy_system.py` → `get_hardware_info()` to resolve IP and SSH credentials for each `hardware_id` referenced in `deployment.yaml`.

**Structure:**

```yaml
execution_plattforms:       # Machines that can run Docker containers
  - hardware_id: <id>       # Unique identifier — must match deployment.yaml
    execution_plattform: true
    ip: <IP address>
    ssh_user: <username>    # ⚠ Do NOT commit to git
    ssh_password: <password> # ⚠ Do NOT commit to git

sensors:                    # Sensor hardware (not execution platforms)
  - hardware_id: <id>
    execution_plattform: false
    driver_layer_cn_id: <driver node id>
    ip: <IP address>

robots:                     # Robot hardware (not execution platforms)
  - hardware_id: <id>
    execution_plattform: false
    driver_layer_cn_id: <driver node id>
    ip: <IP address>
```

**Required fields:**

| Section | Required Fields | Notes |
|---|---|---|
| `execution_plattforms` | `hardware_id`, `ip`, `ssh_user`, `ssh_password` | Only these entries are used for remote deployment |
| `sensors` | `hardware_id`, `ip`, `driver_layer_cn_id` | Informational; referenced by driver layer nodes |
| `robots` | `hardware_id`, `ip`, `driver_layer_cn_id` | Informational; referenced by driver layer nodes |

**Example:**
```yaml
execution_plattforms:
  - hardware_id: mac_pc
    execution_plattform: true
    ip: 172.16.248.5
    ssh_user: myuser
    ssh_password: mypassword

  - hardware_id: ubuntu
    execution_plattform: true
    ip: 192.168.178.50
    ssh_user: myuser
    ssh_password: mypassword

sensors:
  - hardware_id: vr_headset
    execution_plattform: false
    driver_layer_cn_id: sensor_driver_rococo
    ip: 192.168.1.103

robots:
  - hardware_id: xarm5_robot
    execution_plattform: false
    driver_layer_cn_id: xarm5_robot
    ip: 192.168.1.108
```

> **Security note:** Never commit `ssh_user` and `ssh_password` values to the git repository. Leave the values blank in the committed file and fill them in locally.

---

### 2. `deployment.yaml`

**Purpose:** Declares which Docker containers should run on which hardware platform, and provides metadata (e.g. node counts, simulator configuration) consumed by both the deployment script and the in-container launch files.

**Used by:**
- `deploy_system.py` → determines which containers to start on which machine
- `system_manager.launch.py` → reads `node_count` via `count_nodes_from_config()` to tell the system manager how many nodes to expect
- `xxx.launch.py` → reads `hardware_id` and container name via `get_exec_platform_id()` to generate unique monitor node names

**Structure:**

```yaml
platforms:
  - hardware_id: <id>          # Must match an entry in hardware.yaml
    containers:
      - name: <container-name> # Must match a service in docker-compose.yaml
        node_count: <int>      # Number of ROS2 nodes launched in this container
                               # (used by system_manager to track liveness)

  # Simulator container — special fields
  - hardware_id: <id>
    containers:
      - name: simulator
        simulator_name: <gazebo|isaac|...>
        robot: <robot model>
        eef: <end-effector model>
```

**Required fields per container entry:**

| Container Type | Required Fields | Optional Fields |
|---|---|---|
| Regular container | `name`, `node_count` | — |
| Simulator container | `name` (must be `simulator`) | `simulator_name`, `robot`, `eef` |

**Example:**
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

> **Note:** Multiple platform entries can share the same `hardware_id`. Each entry maps to exactly one container group. The `simulator` container is treated specially by `deploy_system.py` — it is only deployed when `--launch-env` is passed.

---

### 3. `docker-compose.yaml`

**Purpose:** Defines how each container is built, what environment variables it receives, which volumes it mounts, and what its startup command is. This is the standard Docker Compose service definition file.

**Used by:** `deploy_system.py` → `deploy_containers()` calls `docker compose -f <path> up -d <container>` using this file. It is also validated upfront via `verify_docker_compose_services()` to catch missing service names before attempting deployment.

**Key fields and their role:**

| Field | Purpose |
|---|---|
| `build.dockerfile` | Points to the Dockerfile for this container's image |
| `environment` | `ROS_DOMAIN_ID` is injected at deploy time; `CONTAINER_NAME` is used by in-container logic for self-identification (see below) |
| `network_mode: host` | Required for ROS2 DDS discovery across containers and hosts |
| `ipc: host` | Enables shared memory between containers on the same host |
| `volumes` | `/dev` and udev mounts allow hardware access (e.g. USB devices); Docker socket mount allows containers to manage sibling containers |
| `command` | Containers idle with `tail -f /dev/null` after sourcing ROS2; actual ROS2 nodes are launched separately inside the container via `ros2 launch` |
| `restart: on-failure` | Automatically restarts if the container crashes |

> **Important:** Every `name` defined in `deployment.yaml` must have a matching service entry in `docker-compose.yaml`. The deployment script validates this before attempting to start any container.

#### The `CONTAINER_NAME` Environment Variable

`CONTAINER_NAME` is a **critical** environment variable that must be set in every service definition. It is the primary mechanism by which a running container identifies itself within the deployment.

When a `xxx.launch.py` file starts inside a container, it calls `get_exec_platform_id()` from `utils.py`, which reads `CONTAINER_NAME` from the environment and matches it against `deployment.yaml` to look up the corresponding `hardware_id`. This resolved pair is then used to construct a unique, deterministic ROS2 monitor node name:

```
monitor_<hardware_id>_<container_name>
# e.g.  monitor_mac_pc_teleop_components
```

**Resolution order inside `get_exec_platform_id()` (utils.py):**

1. `CONTAINER_NAME` environment variable — set explicitly in `docker-compose.yaml` ← **primary, always use this**
2. `/etc/hostname` — Docker sets this to an unpredictable container hash by default ← fallback only, unreliable

Because the `/etc/hostname` fallback produces a hash string that will never match a name in `deployment.yaml`, the lookup fails and the launch file falls back to a generic monitor name (e.g. `monitor_0_compo`). To avoid this, **always set `CONTAINER_NAME` explicitly** and ensure its value matches the `name` field in `deployment.yaml` exactly:

```yaml
# docker-compose.yaml
services:
  teleop-components:
    container_name: teleop-components
    environment:
      - CONTAINER_NAME=teleop-components   # ← must exactly match deployment.yaml

# deployment.yaml
platforms:
  - hardware_id: mac_pc
    containers:
      - name: teleop-components            # ← same string as CONTAINER_NAME above
```

#### Base Service Template

The following is the minimal service definition required for any new container in this system. Copy and adapt it when adding a new service:

```yaml
services:
  <your-container-name>:
    build:
      context: ..                               # repo root, so Dockerfile can COPY packages
      dockerfile: docker/Dockerfile.<n>         # path to this container's Dockerfile
    container_name: <your-container-name>       # must match CONTAINER_NAME and deployment.yaml
    environment:
      - ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}       # injected by deploy_system.py at runtime
      - CONTAINER_NAME=<your-container-name>    # used by utils.get_exec_platform_id()
    network_mode: host                          # required for ROS2 DDS discovery
    ipc: host                                   # required for shared memory between containers
    privileged: true                            # required for hardware/device access
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock   # allows container to manage Docker
      - /run/udev:/run/udev                         # udev events for device hotplug
      - /dev:/dev                                   # full device access
    device_cgroup_rules:
      - 'c *:* rmw'                            # allow read/write/mknod on all char devices
    command: >
      bash -c "source /opt/ros/humble/setup.bash && tail -f /dev/null"
    restart: on-failure
```

**Optional additions** — include only when needed:

```yaml
    # For containers that open GUI windows (rviz, gazebo, etc.)
    environment:
      - DISPLAY=${DISPLAY}
      - XAUTHORITY=${HOME}/.Xauthority
    volumes:
      - ${HOME}/.Xauthority:${HOME}/.Xauthority
      - /tmp/.X11-unix/:/tmp/.X11-unix/

    # For containers that use the argus IPC socket
    volumes:
      - /tmp/argus_socket:/tmp/argus_socket

    # For the simulator container only
    stdin_open: true
    tty: true
    environment:
      - ROBOT=${ROBOT}
      - EEF=${EEF}
```

---

## Dockerfiles

Each service in `docker-compose.yaml` references its own Dockerfile. All Dockerfiles extend a common `Dockerfile.base` that establishes the shared ROS2 environment, installs system and Python dependencies, copies the core packages, and builds the workspace.

### `Dockerfile.base` — Shared Foundation

`Dockerfile.base` is the foundation that all container-specific Dockerfiles build on. It must cover the following:

| Responsibility | What to include |
|---|---|
| Base image | `FROM ros:humble-ros-base` |
| System dependencies | `apt-get` packages: build tools, Python3, OpenCV, Docker client, networking tools |
| Python dependencies | `pip3`: numpy, pyyaml, opencv-python, docker, psutil |
| Core ROS2 packages | `kpi_monitor`, `core_bases`, `component_interfaces`, `system_interfaces`, `teleop_launcher`, `config_files` |
| Workspace build | `colcon build` sourced against `/opt/ros/humble/setup.bash` |
| Entrypoint | Shell script that sources both ROS2 and the workspace `install/setup.bash` before executing the container command |
| Default env vars | `ROS2_DOMAIN_ID=0`, `CONFIG_FOLDER_PATH=/root/dev_ws/src/config_files` |

**Minimal `Dockerfile.base` structure:**

```dockerfile
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/dev_ws

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    git wget curl vim build-essential cmake \
    libopencv-dev python3-opencv \
    docker.io net-tools iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN pip3 install --no-cache-dir \
    numpy pyyaml opencv-python docker psutil cv-bridge

# Create workspace and copy core packages
RUN mkdir -p ${WORKSPACE}/src
COPY kpi_monitor            ${WORKSPACE}/src/kpi_monitor
COPY core_bases             ${WORKSPACE}/src/core_bases
COPY component_interfaces   ${WORKSPACE}/src/component_interfaces
COPY system_interfaces      ${WORKSPACE}/src/system_interfaces
COPY teleop_launcher        ${WORKSPACE}/src/teleop_launcher
COPY config_files           ${WORKSPACE}/src/config_files
# TODO: add your own packages here

# Install rosdep dependencies and build
WORKDIR ${WORKSPACE}
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Entrypoint: sources ROS2 + workspace before running the container command
RUN echo '#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nsource ${WORKSPACE}/install/setup.bash\nexec "$@"' \
    > /entrypoint.sh && chmod +x /entrypoint.sh

ENV ROS2_DOMAIN_ID=0
ENV CONFIG_FOLDER_PATH=/root/dev_ws/src/config_files
WORKDIR ${WORKSPACE}
ENTRYPOINT ["/entrypoint.sh"]
```

### Container-Specific Dockerfiles

Each container extends `Dockerfile.base` and adds what is unique to that container: its own ROS2 packages and any extra dependencies. The system manager Dockerfile is already defined and serves as the reference pattern.

**`Dockerfile.sysManager` (already defined):**

```dockerfile
FROM <base-image>

# Copy system-manager-specific packages
COPY system_manager  ${WORKSPACE}/src/system_manager
COPY data_collector  ${WORKSPACE}/src/data_collector

# Rebuild workspace to include new packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
```

**Pattern for any new container Dockerfile:**

```dockerfile
FROM <base-image>

# Copy this container's specific ROS2 packages
COPY <your_package>  ${WORKSPACE}/src/<your_package>

# Install any extra system or Python dependencies specific to this container (if needed)
RUN apt-get update && apt-get install -y <extra-deps> && rm -rf /var/lib/apt/lists/*

# Rebuild to include the new packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"
```

> **Build context note:** `docker-compose.yaml` sets `context: ..` (the repository root), so every `COPY` instruction in any Dockerfile is relative to the repo root. All packages must live at the top level of the repository for `COPY` to find them.

---

## Launch Files (`xxx.launch.py`)

Each container has a corresponding ROS2 launch file inside the `teleop_launcher` package. The launch file is the entry point for starting all ROS2 nodes within that container after the container is running. A `base.launch.py` template is provided to use as the starting point for any new launch file.

### What a Launch File Does

Every launch file in this system follows the same pattern:

1. Reads `CONTAINER_NAME` from the environment (set by `docker-compose.yaml`)
2. Calls `get_exec_platform_id()` to resolve the `hardware_id` and `container_name` from `deployment.yaml`
3. Constructs the monitor node name: `monitor_<hardware_id>_<container_name>`
4. Calls `check_monitor_node_exists()` to avoid launching a duplicate monitor if one is already running on the ROS2 domain
5. Launches the `kpi_monitor_node` (if not already present) and all component nodes specific to that container

### Launch Arguments

All launch files share the same two base arguments:

| Argument | Default | Description |
|---|---|---|
| `ros_domain_id` | `"0"` | ROS 2 Domain ID — must match the value passed to `deploy_system.py` |
| `config_folder_path` | `/root/dev_ws/src/config_files` | Path to the folder containing `deployment.yaml` and other config files inside the container |

Additional arguments can be declared per launch file to pass `layer_cn_id` values to individual nodes (see the `component_nodes.launch.py` example).

### `base.launch.py` — Template

`base.launch.py` provides the minimal structure every launch file must have. Copy it and extend the `nodes` list with the nodes specific to your container:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch_ros.actions import Node
import os

from utils.utils import get_exec_platform_id, check_monitor_node_exists

def launch_setup(context, *args, **kwargs):
    ros_domain_id    = context.launch_configurations['ros_domain_id']
    config_folder_path = context.launch_configurations['config_folder_path']

    deployment_config_path = os.path.join(config_folder_path, "deployment.yaml")

    # Resolve hardware_id + container_name from CONTAINER_NAME env var
    hardware_id, container_name = get_exec_platform_id(deployment_config_path)
    hardware_id = hardware_id.replace('-', '_')

    nodes = []

    # Build and conditionally launch the monitor node
    if hardware_id:
        monitor_node_name = f"monitor_{hardware_id}_{container_name}"
    else:
        monitor_node_name = "monitor_0_"   # fallback — indicates CONTAINER_NAME was not set
        print(f"Warning: Could not determine hardware_id, using default monitor name: {monitor_node_name}")

    if not check_monitor_node_exists(monitor_node_name, ros_domain_id):
        nodes.append(
            Node(
                package="kpi_monitor",
                executable="kpi_monitor_node",
                name=monitor_node_name,
                output="screen",
                arguments=["--layer_cn_id", monitor_node_name, "--ros2_domain_id", ros_domain_id]
            )
        )

    # TODO: add your component nodes here
    # nodes.extend([
    #     Node(
    #         package="my_package",
    #         executable="my_node",
    #         name="my_node",
    #         output="screen",
    #         arguments=["--layer_cn_id", context.launch_configurations['my_node'],
    #                    "--ros2_domain_id", ros_domain_id]
    #     ),
    # ])

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ros_domain_id",       default_value="0"),
        DeclareLaunchArgument("config_folder_path",  default_value="/root/dev_ws/src/config_files"),

        # Declare one argument per node's layer_cn_id
        DeclareLaunchArgument("example_component", default_value="example_component"),

        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        OpaqueFunction(function=launch_setup)
    ])
```

### How to Create a Launch File for a New Container

**1. Copy `base.launch.py`** into `teleop_launcher/launch/` and rename it (e.g. `my_container.launch.py`).

**2. Declare a `LaunchArgument` for each node's `layer_cn_id`** in `generate_launch_description()`. The default value should be the node's logical identifier:

```python
DeclareLaunchArgument("my_component", default_value="my_component"),
```

**3. Extend the `nodes` list** inside `launch_setup()` with a `Node` entry for every ROS2 node this container should run. Pass `--layer_cn_id` and `--ros2_domain_id` as arguments so each node can identify itself on the network:

```python
nodes.extend([
    Node(
        package="my_package",
        executable="my_node_executable",
        name="my_node",
        output="screen",
        arguments=[
            "--layer_cn_id", context.launch_configurations['my_component'],
            "--ros2_domain_id", ros_domain_id
        ]
    ),
])
```

**4. Update `node_count` in `deployment.yaml`** to reflect the total number of nodes now launched in this container (component nodes + 1 for the monitor node):

```yaml
- name: my-container
  node_count: 3   # 1 monitor + 2 component nodes
```

**5. Register the launch file** as an entry in the `teleop_launcher` package's `setup.py` or `CMakeLists.txt` so it is installed and discoverable by `ros2 launch`.

### Key Rules

- **Always keep `OpaqueFunction`** wrapping `launch_setup` — it defers resolution of `LaunchConfiguration` values to runtime, which is required for dynamic argument passing.
- **Never hardcode `hardware_id` or `container_name`** — always let `get_exec_platform_id()` resolve them from `deployment.yaml` via `CONTAINER_NAME`. This ensures the monitor node name is consistent with what `deploy_system.py` and the rest of the system expect.
- **`check_monitor_node_exists()` uses a 5-second `ros2 node list` timeout** — if the ROS2 daemon is slow to start, the check may return `False` even if a monitor exists. In practice this is not a problem because a duplicate monitor will simply fail to register a conflicting node name.

---

## `deploy_system.py` — Deployment Script

### What It Does

`deploy_system.py` is the central orchestrator. It:

1. Loads all three config files
2. Iterates over each platform entry in `deployment.yaml`
3. Resolves the target machine's IP and SSH credentials from `hardware.yaml`
4. Validates that the requested containers exist in `docker-compose.yaml`
5. In remote mode: syncs the local repository to the remote machine via `rsync` (or `tar+ssh` as fallback)
6. Runs `docker compose up -d <container>` on the target machine (locally or over SSH)

### Command-Line Arguments

| Argument | Type | Default | Description |
|---|---|---|---|
| `--ros-domain-id` | `str` | `"0"` | ROS 2 Domain ID passed to all containers as `ROS_DOMAIN_ID` |
| `--local-mode` | flag | `False` | Run `docker compose` commands on the local machine instead of SSH-ing to the remote host defined in `hardware.yaml` |
| `--stop` | flag | `False` | Stop containers instead of starting them |
| `--launch-env` | flag | `False` | Deploy only the `simulator` container, always on the local machine (ignores `hardware_id` for that entry) |
| `--force-update` | flag | `False` | When syncing to a remote that already has the repository folder, force overwrite. Without this flag, an existing folder is reused as-is |

### SSH Authentication

The script supports three authentication methods, tried in priority order:

1. **SSH key** — set `ssh_key_path` in the `RemoteExecutor` constructor (e.g. `~/.ssh/id_rsa`)
2. **Password** — set `ssh_password` in `hardware.yaml`; requires `sshpass` installed on the host
3. **SSH agent / default key** — falls back to whatever key is loaded in the current SSH agent

### Repository Sync (Remote Mode Only)

When deploying to a remote machine (`--local-mode` not set), the script automatically copies the entire local repository to the remote host before running Docker Compose. It uses `rsync` when available, and falls back to `tar` piped over SSH. The `data/` directory is excluded from the transfer.

---

## Full Deployment Walkthrough

This section walks through deploying the complete system from scratch, matching the example configuration in the provided files.

### Step 1 — Fill in `hardware.yaml`

Add the SSH credentials for each execution platform. **Do not commit these values.**

```yaml
execution_plattforms:
  - hardware_id: mac_pc
    execution_plattform: true
    ip: 172.16.248.5
    ssh_user: myuser
    ssh_password: mypassword

  - hardware_id: ubuntu
    execution_plattform: true
    ip: 192.168.178.50
    ssh_user: ubuntuuser
    ssh_password: ubuntupassword
```

### Step 2 — Define `deployment.yaml`

Map each container to the hardware platform it should run on. Make sure every `name` matches a service in `docker-compose.yaml`, and that `node_count` reflects the total number of ROS2 nodes (including the monitor node) that will be launched inside that container.

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

### Step 3 — Prepare Dockerfiles

Each container needs a Dockerfile under `docker/`. All Dockerfiles must `COPY` their packages relative to the repository root (the build context is `..` in `docker-compose.yaml`).

**Minimum checklist per Dockerfile:**

- Extends `Dockerfile.base` (or builds from `ros:humble-ros-base` directly following the base structure above)
- Copies any packages specific to this container into `${WORKSPACE}/src/`
- Runs `colcon build` to include the new packages

The `Dockerfile.sysManager` is already defined and ready to use as a reference. 

### Step 4 — Prepare `docker-compose.yaml`

Add a service entry for every container name used in `deployment.yaml`. Use the base service template from the `docker-compose.yaml` section above. The two most important things to get right:

- `CONTAINER_NAME` in `environment` must **exactly** match the `name` field in `deployment.yaml`
- `dockerfile` must point to the correct `Dockerfile` under `docker/`

### Step 5 — Prepare Launch Files

For each container, create a launch file inside the `teleop_launcher` package by copying `base.launch.py` and extending it with the nodes that container should run (see the Launch Files section for the full pattern).

**Checklist per launch file:**

- Named clearly and placed in `teleop_launcher/launch/` (e.g. `my_container.launch.py`)
- One `DeclareLaunchArgument` per node's `layer_cn_id` in `generate_launch_description()`
- All component nodes added to the `nodes` list inside `launch_setup()`
- `node_count` in `deployment.yaml` updated to match the total nodes (component nodes + 1 monitor)
- Launch file registered in the package's `setup.py` or `CMakeLists.txt`

The launch files for `teleop-system` (`system_manager.launch.py`) and `teleop-components` (`component_nodes.launch.py`) are already defined and can be used as working references alongside `base.launch.py`.

### Step 6 — Deploy the Teleop Containers (Remote)

Run the deploy script from the repository root. This will SSH into each machine defined in `hardware.yaml`, sync the repository, and start the non-simulator containers.

```bash
python3 deploy_system.py --ros-domain-id 0
```

The script will:
- SSH into `mac_pc` and start `teleop-system`, `teleop-components`, `teleop-drivers`
- Skip the `simulator` container (requires `--launch-env`)

### Step 7 — Deploy the Simulator (Local)

The simulator container is always launched locally, regardless of its `hardware_id` in `deployment.yaml`. Run:

```bash
python3 deploy_system.py --launch-env --ros-domain-id 0
```

This starts the `simulator` service on the local machine and injects `ROBOT=xarm5` and `GRIPPER=xarm_gripper` as environment variables, taken from `deployment.yaml`.

### Step 8 — Launch ROS2 Nodes Inside Containers

Once containers are running, connect to each container and launch the appropriate ROS2 nodes using the launch files prepared in Step 5.

```bash
# Inside teleop-system container
ros2 launch teleop_launcher system_manager.launch.py ros_domain_id:=0

# Inside teleop-components container
ros2 launch teleop_launcher component_nodes.launch.py ros_domain_id:=0

# Inside teleop-drivers container
ros2 launch teleop_launcher driver_nodes.launch.py ros_domain_id:=0

# Inside simulator container running a monitor node
ros2 launch teleop_launcher simulation_monitor.launch.py ros_domain_id:=0
```

Each launch file reads `deployment.yaml` via `CONTAINER_NAME` to:
- Resolve the `hardware_id` and derive the unique monitor node name (`monitor_<hardware_id>_<container_name>`)
- Count total expected nodes (`node_count` sum) for the system manager's liveness tracking

### Step 9 — Stopping the System

To stop all non-simulator containers on their remote hosts:

```bash
python3 deploy_system.py --stop
```

To stop the simulator:

```bash
python3 deploy_system.py --launch-env --stop
```

---

## Configuration File Cross-Reference

```
hardware.yaml ──────────────────────────────────────────────────────────────────┐
  hardware_id, ip, ssh_user, ssh_password                                       │
        │                                                                        │
        │ resolved by get_hardware_info()                                        │
        ▼                                                                        │
deployment.yaml ────────────────────────────────────────────────────────────────┤
  hardware_id     (must match hardware.yaml)                                     │
  container name  (must match docker-compose.yaml services)                      │
  node_count      (read by system_manager.launch.py → count_nodes_from_config)  │
        │                                                                        │
        │ iterated by deploy_platform()                    also read at runtime  │
        ▼                                                  by xxx.launch.py ─────┼──┐
docker-compose.yaml ◄───────────────────────────────────────────────────────────┘  │
  CONTAINER_NAME env var  (must match deployment.yaml container name)               │
  service definitions validated by verify_docker_compose_services()                 │
  started via: docker compose -f <path> up -d <container>                           │
        │                                                                            │
        │ running container (CONTAINER_NAME available in env)                        │
        ▼                                                                            │
  xxx.launch.py  (inside container, in teleop_launcher package) ◄───────────────────┘
    get_exec_platform_id() reads CONTAINER_NAME → matches deployment.yaml
    → resolves hardware_id + container_name
    → builds monitor node name: monitor_<hardware_id>_<container_name>
    check_monitor_node_exists() → avoids duplicate monitor on the ROS domain
    → launches kpi_monitor_node + all container-specific component nodes

  Dockerfiles  (docker/Dockerfile.<n>)
    all extend Dockerfile.base
    each COPYs its own ROS2 packages and runs colcon build
    base.launch.py lives in teleop_launcher/launch/ as the template
```