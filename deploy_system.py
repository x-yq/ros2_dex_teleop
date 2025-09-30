#!/usr/bin/env python3
"""
Deployment script for distributed ROS2 system.
Reads deployment.yaml and hardware.yaml to deploy containers across multiple platforms.
"""

import yaml
import subprocess
import os
import sys
from typing import Dict, List, Optional
import argparse


class RemoteExecutor:
    """Handles command execution on remote or local machines."""
    
    def __init__(self, ip_address: str, port: int = 22, local_mode: bool = True):
        self.ip_address = ip_address
        self.port = port
        self.local_mode = local_mode
    
    def execute(self, command: str, check: bool = True) -> subprocess.CompletedProcess:
        """
        Execute a command either locally or remotely via SSH.
        
        Args:
            command: The command to execute
            check: Whether to raise exception on non-zero exit code
        
        Returns:
            CompletedProcess object
        """
        if self.local_mode:
            # Execute locally for now
            print(f"[LOCAL] Executing: {command}")
            return subprocess.run(
                command,
                shell=True,
                check=check,
                capture_output=True,
                text=True
            )
        else:
            # Future: Execute via SSH
            ssh_command = f"ssh -p {self.port} {self.ip_address} '{command}'"
            print(f"[SSH {self.ip_address}] Executing: {command}")
            return subprocess.run(
                ssh_command,
                shell=True,
                check=check,
                capture_output=True,
                text=True
            )
    
    def execute_interactive(self, command: str):
        """Execute a command in interactive mode (for docker exec)."""
        if self.local_mode:
            print(f"[LOCAL INTERACTIVE] Executing: {command}")
            return subprocess.run(command, shell=True)
        else:
            # Future: Execute via SSH with TTY allocation
            ssh_command = f"ssh -t -p {self.port} {self.ip_address} '{command}'"
            print(f"[SSH {self.ip_address} INTERACTIVE] Executing: {command}")
            return subprocess.run(ssh_command, shell=True)


def load_yaml_config(file_path: str) -> Dict:
    """Load and parse a YAML configuration file."""
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        sys.exit(1)


def get_hardware_info(hardware_id: str, hardware_config: Dict) -> Optional[Dict]:
    """
    Get hardware information from hardware.yaml.
    
    Args:
        hardware_id: The hardware identifier
        hardware_config: Parsed hardware.yaml content
    
    Returns:
        Dictionary with hardware info or None if not found
    """
    # Check execution platforms
    for platform in hardware_config.get('execution_plattforms', []):
        if platform['hardware_id'] == hardware_id:
            return platform
    
    # Check sensors
    for sensor in hardware_config.get('sensors', []):
        if sensor['hardware_id'] == hardware_id:
            return sensor
    
    # Check robots
    for robot in hardware_config.get('robots', []):
        if robot['hardware_id'] == hardware_id:
            return robot
    
    return None


def build_docker_image(executor: RemoteExecutor, dockerfile_path: str, image_tag: str) -> bool:
    """
    Build a Docker image from the specified Dockerfile.
    
    Args:
        executor: RemoteExecutor instance
        dockerfile_path: Path to the Dockerfile
        image_tag: Tag for the built image
    
    Returns:
        True if successful, False otherwise
    """
    # Get the context directory (directory containing the Dockerfile)
    context_dir = os.path.dirname(dockerfile_path) or '.'
    
    build_command = f"docker build -t {image_tag} -f {dockerfile_path} {context_dir}"
    
    try:
        result = executor.execute(build_command)
        print(f"✓ Successfully built Docker image: {image_tag}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to build Docker image {image_tag}")
        print(f"Error: {e.stderr}")
        return False


def run_docker_container(
    executor: RemoteExecutor,
    container_config: Dict,
    image_tag: str
) -> bool:
    """
    Run a Docker container with the specified configuration.
    
    Args:
        executor: RemoteExecutor instance
        container_config: Container configuration from deployment.yaml
        image_tag: Docker image tag to run
    
    Returns:
        True if successful, False otherwise
    """
    container_name = container_config['name']
    
    # Check if container already exists and remove it
    check_cmd = f"docker ps -aq -f name={container_name}"
    try:
        result = executor.execute(check_cmd, check=False)
        if result.stdout.strip():
            print(f"Removing existing container: {container_name}")
            executor.execute(f"docker rm -f {container_name}", check=False)
    except Exception as e:
        print(f"Warning: Error checking for existing container: {e}")
    
    # Build docker run command
    docker_cmd_parts = [
        "docker run -d",
        f"--name {container_name}",
        
        # Security and capability options
        "--security-opt seccomp=unconfined",
        "--cap-add SYS_PTRACE",
        
        # Working directory
        "-w /root",
        
        # Volume mounts
        "-v /tmp/argus_socket:/tmp/argus_socket",
        "-v /var/run/docker.sock:/var/run/docker.sock",
        "-v /run/udev:/run/udev",
        "-v /dev:/dev",
        
        # Device cgroup rule
        "--device-cgroup-rule='c *:* rmw'",
    ]
    
    # Add environment variables
    env_vars = container_config.get('env', {})
    for key, value in env_vars.items():
        docker_cmd_parts.append(f"-e {key}={value}")
    
    # Add network settings
    network_config = container_config.get('network', {})
    if network_config.get('ipc'):
        docker_cmd_parts.append(f"--ipc={network_config['ipc']}")
    if network_config.get('net'):
        docker_cmd_parts.append(f"--network={network_config['net']}")
    
    # Add permissions
    permissions = container_config.get('permissions', {})
    if permissions.get('privileged'):
        docker_cmd_parts.append("--privileged")
    
    # Add image tag
    docker_cmd_parts.append(image_tag)
    
    # Keep container running
    docker_cmd_parts.append("tail -f /dev/null")
    
    docker_cmd = " ".join(docker_cmd_parts)
    
    try:
        result = executor.execute(docker_cmd)
        print(f"✓ Successfully started container: {container_name}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to start container {container_name}")
        print(f"Error: {e.stderr}")
        return False


def launch_ros_nodes(
    executor: RemoteExecutor,
    container_name: str,
    launch_file_path: str,
    ros_domain_id: str = "0"
) -> bool:
    """
    Execute the ROS2 launch file inside the container.
    
    Args:
        executor: RemoteExecutor instance
        container_name: Name of the running container
        launch_file_path: Path to the launch file
        ros_domain_id: ROS 2 Domain ID
    
    Returns:
        True if successful, False otherwise
    """
    # Prepare the launch command
    launch_cmd = f"ros2 launch {launch_file_path} ros_domain_id:={ros_domain_id}"
    
    # Execute inside container
    docker_exec_cmd = f"docker exec -d {container_name} bash -c 'source /opt/ros/*/setup.bash && {launch_cmd}'"
    
    try:
        result = executor.execute(docker_exec_cmd)
        print(f"✓ Successfully launched ROS nodes in container: {container_name}")
        print(f"  Launch file: {launch_file_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to launch ROS nodes in container {container_name}")
        print(f"Error: {e.stderr}")
        return False


def deploy_platform(
    platform_config: Dict,
    hardware_config: Dict,
    deployment_config: Dict,
    local_mode: bool = True,
    ros_domain_id: str = "0"
):
    """
    Deploy all containers for a specific hardware platform.
    
    Args:
        platform_config: Platform configuration from deployment.yaml
        hardware_config: Full hardware.yaml configuration
        deployment_config: Full deployment.yaml configuration
        local_mode: Whether to execute locally or via SSH
        ros_domain_id: ROS 2 Domain ID
    """
    hardware_id = platform_config['hardware_id']
    print(f"\n{'='*60}")
    print(f"Deploying platform: {hardware_id}")
    print(f"{'='*60}")
    
    # Get hardware information
    hw_info = get_hardware_info(hardware_id, hardware_config)
    if not hw_info:
        print(f"✗ Hardware {hardware_id} not found in hardware.yaml")
        return
    
    # Check if this is an execution platform
    is_execution_platform = hw_info.get('execution_plattform', False)
    if not is_execution_platform:
        print(f"✗ Hardware {hardware_id} is not an execution platform, skipping deployment")
        return
    
    ip_address = hw_info.get('ip', '127.0.0.1')
    access_port = hw_info.get('access_port', 22)
    
    print(f"Target IP: {ip_address}:{access_port}")
    
    # Create remote executor
    executor = RemoteExecutor(ip_address, access_port, local_mode)
    
    # Deploy each container
    containers = platform_config.get('containers', [])
    for container_config in containers:
        container_name = container_config['name']
        print(f"\n--- Deploying container: {container_name} ---")
        
        # Get Dockerfile path
        dockerfile_path = container_config.get('dockerfile')
        if not dockerfile_path:
            print(f"✗ No dockerfile specified for container {container_name}")
            continue
        
        # Build Docker image
        image_tag = f"{container_name}:latest"
        if not build_docker_image(executor, dockerfile_path, image_tag):
            print(f"✗ Skipping container {container_name} due to build failure")
            continue
        
        # Run Docker container
        if not run_docker_container(executor, container_config, image_tag):
            print(f"✗ Skipping launch for container {container_name} due to run failure")
            continue
        
        # Launch ROS nodes
        launch_file_path = container_config.get('launch_file_path')
        if launch_file_path:
            launch_ros_nodes(executor, container_name, launch_file_path, ros_domain_id)
        else:
            print(f"Warning: No launch_file_path specified for container {container_name}")
        
        print(f"✓ Container {container_name} deployed successfully")


def main():
    parser = argparse.ArgumentParser(description='Deploy distributed ROS2 system')
    parser.add_argument(
        '--ros-domain-id',
        default='0',
        help='ROS 2 Domain ID'
    )
    parser.add_argument(
        '--local-mode',
        action='store_true',
        default=True,
        help='Execute locally instead of via SSH (default: True)'
    )
    
    args = parser.parse_args()
    
    # Hardcoded config file paths
    deployment_config_path = './config_files/deployment.yaml'
    hardware_config_path = './config_files/hardware.yaml'
    
    # Load configurations
    print("Loading configuration files...")
    print(f"  Deployment config: {deployment_config_path}")
    print(f"  Hardware config: {hardware_config_path}")
    
    deployment_config = load_yaml_config(deployment_config_path)
    hardware_config = load_yaml_config(hardware_config_path)
    
    print(f"Loaded {len(deployment_config.get('platforms', []))} platforms from deployment config")
    
    # Deploy platforms
    platforms = deployment_config.get('platforms', [])
    
    for platform_config in platforms:
        deploy_platform(
            platform_config,
            hardware_config,
            deployment_config,
            args.local_mode,
            args.ros_domain_id
        )
    
    print(f"\n{'='*60}")
    print("Deployment complete!")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()