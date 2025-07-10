  - [Package Description](#package-description)
  - [System/Hardware Requirements](#systemhardware-requirements)
    - [1. Architecture and Operating System Support](#1-architecture-and-operating-system-support)
    - [2. Resource Requirements](#2-resource-requirements)
    - [3. GPU Support](#3-gpu-support)
  - [Docker Configuration](#docker-configuration)
    - [0. Software Dependencies](#0-software-dependencies)
    - [1. Dockerfile Specifications](#1-dockerfile-specifications)
    - [2. Setup Scripts](#2-setup-scripts)
    - [3. Docker Compose](#3-docker-compose)
  - [Network Configuration](#network-configuration)
    - [1. Port Requirements](#1-port-requirements)
    - [2. Bandwidth Estimation](#2-bandwidth-estimation)
  - [Hardware Interfaces](#hardware-interfaces)
    - [1. USB Devices](#1-usb-devices)
    - [2. Network Devices](#2-network-devices)
  - [Security Considerations](#security-considerations)
    - [1. Permission Requirements](#1-permission-requirements)
    - [2. Authentication](#2-authentication)
  - [Monitoring & Debugging](#monitoring-debugging)
    - [1. Logging Configuration](#1-logging-configuration)
    - [2. Health Check Endpoints](#2-health-check-endpoints)
  - [Appendix](#appendix)
    - [A. Docker Container Dev Info](#a-docker-container-dev-info)
    - [B. Known Issues](#b-known-issues)
    - [C. Performance Benchmarks](#c-performance-benchmarks)
    - [D. Docker Utils](#d-docker-utils)


# Package Description

> list of pakcage name and simple function description

- [x] interfaces (common)
- [x] demp_publisher (only for test)
- data_collection
- sensor_runner
- visualizer
- pose_estimation
- motion_transformation
- robot_controller
- resource_monitor (common)
- deployment (common)

# System/Hardware Requirements

## 1. Architecture and Operating System Support
> Only an example:

| Package | Architecture | OS-Version | Notes |
|----|---------|-------|---------|
| DataCollection | amd64 (x86_64) | Ubuntu-22.04 LTS (Recommended) | ROS2 Humble native support |
| DataCollection | arm64 (aarch64, e.g. Raspberry Pi cluster) | Ubuntu-22.04 LTS (Recommended) | ROS2 Humble native support |

## 2. Resource Requirements
> Only an example:

| Package | Minimal RAM | Recommended RAM | CPU Cores | Storage |
|--------------|------------|-----------------|-----------|---------|
| Robot Controller | 2GB | 4GB+ | 4 threads | 5GB |
| Sensor Driver | 1GB | 2GB | 2 cores | 2GB |
| Learning-based Pose Estimator | 8GB | 16GB+ | 8 cores+ | 20GB+ |
| VR Hand Pose Estimator | 6GB | 12GB+ | 6 cores+ | 15GB |

## 3. GPU Support
> Only an example:

| Package | Supported GPUs | Driver Requirements | Toolkit Dependencies |
|--------------|-----------------|-----------|---------|
| Learning-based Pose Estimator |  NVIDIA (Jetson TX2/Xavier, RTX 20/30/40 series), AMD (RDNA2+) | NVIDIA: ≥515.65.01, CUDA ≥11.7 | cuDNN ≥8.5, TensorRT ≥8.4 |

# Docker Configuration

## 0. Software Dependencies

> Package-specific:

> python requirement (installed via dockerfile)

> extra pkgs (installed via dockerfile)

> repo (installed via sh file)

| Package | Docker image | Python req | Extra Packages | Third-party Repo |
|--------------|-----------------|-----------|---------|---------|
| DataCollection |  osrf/ros:humble-ros-core | numpy>=1.21.0 <br> torch>=2.0.0 | `ros-humble-rosidl-generator-py` <br> `python3-rosdep` | Robo-DM |
| SensorDriver: RealSense | ... | ... | ... | Intel RealSense SDK (for depth cameras) |
| EnvDeployer: Gazebo with xArm | ... | ... | ... | xArm ROS2 Driver (for xarm controller) |

## 1. Dockerfile Specifications
> Template and links to package-specific dockerfile

see the base Dockerfile [here](https://git.tu-berlin.de/shuai_ws/students/yeqiao_ws/teleop_dev/-/blob/main/deployment/dockerfiles/Dockerfile.base?ref_type=heads) 


## 2. Setup Scripts

> Template and links to package-specific sh file

see the base bash file [here](https://git.tu-berlin.de/shuai_ws/students/yeqiao_ws/teleop_dev/-/blob/main/deployment/shfiles/base.sh?ref_type=heads) 


## 3. Docker Compose

> Template and links to package-specific docker compose file

**Docker compose file**

see the base docker-compose.yaml [here](https://git.tu-berlin.de/shuai_ws/students/yeqiao_ws/teleop_dev/-/blob/main/deployment/dockerfiles/Dockerfile.base?ref_type=heads) 

**Docker run command**

```bash

# TODO: change the following according to your case
# container_name, dockerfiles/Dockerfile.base, /src/deployment/shfiles/base.sh

docker run -it \
  --network=host \
  --ipc=host \
  --security-opt seccomp=unconfined \
  --privileged \
  --workdir=/root \
  -e DEV=true \
  -v /var/run/docker.sock:/var/run/docker.sock \
  -v $(dirname $(pwd)):/src \
  -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /run/udev:/run/udev \
  -v /dev:/dev \
  --cap-add=SYS_PTRACE \
  --device=/dev \
  --device-cgroup-rule='c *:* rmw' \
  --name container_name \
  $(docker build -q -f dockerfiles/Dockerfile.base .) \
  /src/deployment/shfiles/data.sh
```

# Network Configuration

## 1. Port Requirements
> only an example

| Port | Protocol | Purpose |
|------|----------|---------|
| 8888 | TCP | Web visualization |
| 11311 | TCP/UDP | ROS2 default |
| 9090 | TCP | ROS Bridge |

## 2. Bandwidth Estimation
> only an example, not sure if it will be included in the future

| Component | Min Bandwidth | Recommended |
|-----------|---------------|-------------|
| Camera Stream | 5 Mbps | 50 Mbps (4K) |
| Point Cloud | 20 Mbps | 100 Mbps |

# Hardware Interfaces

## 1. USB Devices
> only an example, need to investigate

- VID/PID Whitelist:
  - Intel RealSense: 8086:0b07
  - OAK-D: 03e7:2485

## 2. Network Devices
> only an example, need to investigate

- Ethernet: ≥1Gbps recommended for robot control
- WiFi: 802.11ax (WiFi 6) for mobile platforms

# Security Considerations

## 1. Permission Requirements
- Docker: Requires --privileged flag for hardware access, root user?

## 2. Authentication
- API Keys: For cloud-based services (optional)
- Device Tokens (ssh): For IoT device communication
- X11 authority: For visualization display

# Monitoring & Debugging

## 1. Logging Configuration

```python
# Recommended ROS2 logging setup
rclpy.logging.set_logger_level('my_package', rclpy.logging.LoggingSeverity.INFO)
```

## 2. Health Check Endpoints
> need to investigate

- HTTP: /health (returns 200 OK)
- ROS2 Service: /diagnostics

# Appendix

## A. Docker Container Dev Info
> on which device, you have which docker image and container running for which package dev.

| Device | Host IP (in lab) | Docker Image | Docker Container | Open SSH setup | Dev Details | Docker Run CMD |
|-----------|---------------|-------------|-------------|-------------|-------------|-------------|
| Peronal Mac | ... |  teleop_data_dev | teleop_data_dev_container | ... | interface, demo_publisher, DataCollectionNode | /teleop_dev/deployment/dev_containers/teleop_data_dev.sh
| Working laptop | ... | ... | ... | container ip? port?  | ... | ... |
| Rendering PC | ... |  ... | ... | container ip? port?  | ... | ... |
| Jetson-xxx | ...|  ... | ... | container ip? port?  | ... | ... |
| Pi5-xxx | ... | ... | ... | container ip? port?  | ... | ... |

## B. Known Issues

## C. Performance Benchmarks

## D. Docker Utils

### Commit a docker container as image

```bash
apt-get clean && rm -rf /var/lib/apt/lists/*
rm -rf /tmp/* /var/tmp/*
pip cache purge

docker commit contianer_name image_name
```