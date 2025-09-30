#!/usr/bin/env python3
"""
Deployment script for distributed ROS2 system using docker-compose.
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


def verify_docker_compose_services(compose_config: Dict, containers: List[str]) -> bool:
    """
    Verify that all requested containers exist in docker-compose.yaml.
    
    Args:
        compose_config: Parsed docker-compose.yaml content
        containers: List of container names to deploy
    
    Returns:
        True if all containers exist, False otherwise
    """
    services = compose_config.get('services', {})
    missing_containers = []
    
    for container in containers:
        if container not in services:
            missing_containers.append(container)
    
    if missing_containers:
        print(f"✗ Missing containers in docker-compose.yaml: {', '.join(missing_containers)}")
        return False
    
    return True


def deploy_containers(
    executor: RemoteExecutor,
    containers: List[str],
    ros_domain_id: str,
    compose_file_path: str = "config_files/docker-compose.yaml"
) -> bool:
    """
    Deploy specified containers using docker-compose.
    
    Args:
        executor: RemoteExecutor instance
        containers: List of container names to deploy
        ros_domain_id: ROS 2 Domain ID
        compose_file_path: Path to docker-compose.yaml file
    
    Returns:
        True if all containers deployed successfully, False otherwise
    """
    success = True
    
    for container in containers:
        print(f"\n--- Deploying container: {container} ---")
        
        # Set ROS_DOMAIN_ID environment variable for docker-compose
        env_vars = f"ROS_DOMAIN_ID={ros_domain_id}"
        
        # Docker compose up command for specific service
        compose_cmd = f"{env_vars} docker compose -f {compose_file_path} up -d {container}"
        
        try:
            result = executor.execute(compose_cmd)
            print(f"✓ Successfully deployed container: {container}")
            
            # Show container logs (last 10 lines) to verify startup
            logs_cmd = f"docker logs --tail 10 {container}"
            try:
                log_result = executor.execute(logs_cmd, check=False)
                if log_result.stdout:
                    print(f"Container logs:\n{log_result.stdout}")
            except Exception as e:
                print(f"Warning: Could not fetch logs for {container}: {e}")
                
        except subprocess.CalledProcessError as e:
            print(f"✗ Failed to deploy container: {container}")
            print(f"Error: {e.stderr}")
            success = False
            continue
    
    return success


def stop_containers(
    executor: RemoteExecutor,
    containers: List[str],
    compose_file_path: str = "config_files/docker-compose.yaml"
) -> bool:
    """
    Stop specified containers using docker compose.
    
    Args:
        executor: RemoteExecutor instance
        containers: List of container names to stop
        compose_file_path: Path to docker-compose.yaml file
    
    Returns:
        True if successful, False otherwise
    """
    print(f"\n--- Stopping containers: {', '.join(containers)} ---")
    
    # Docker compose down command for specific services
    container_list = ' '.join(containers)
    compose_cmd = f"docker compose -f {compose_file_path} stop {container_list}"
    
    try:
        result = executor.execute(compose_cmd)
        print(f"✓ Successfully stopped containers")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to stop containers")
        print(f"Error: {e.stderr}")
        return False


def deploy_platform(
    platform_config: Dict,
    hardware_config: Dict,
    compose_config: Dict,
    local_mode: bool = True,
    ros_domain_id: str = "0",
    compose_file_path: str = "config_files/docker-compose.yaml",
    stop_only: bool = False
):
    """
    Deploy all containers for a specific hardware platform.
    
    Args:
        platform_config: Platform configuration from deployment.yaml
        hardware_config: Full hardware.yaml configuration
        compose_config: Parsed docker-compose.yaml content
        local_mode: Whether to execute locally or via SSH
        ros_domain_id: ROS 2 Domain ID
        compose_file_path: Path to docker-compose.yaml file
        stop_only: If True, only stop containers without starting them
    """
    hardware_id = platform_config['hardware_id']
    print(f"\n{'='*60}")
    print(f"Platform: {hardware_id}")
    print(f"{'='*60}")
    
    # Get hardware information
    hw_info = get_hardware_info(hardware_id, hardware_config)
    if not hw_info:
        print(f"✗ Hardware {hardware_id} not found in hardware.yaml")
        return
    
    # Check if this is an execution platform
    is_execution_platform = hw_info.get('execution_plattform', False)
    if not is_execution_platform:
        print(f"✗ Hardware {hardware_id} is not an execution platform, skipping")
        return
    
    ip_address = hw_info.get('ip', '127.0.0.1')
    access_port = hw_info.get('access_port', 22)
    
    print(f"Target: {ip_address}:{access_port}")
    
    # Get containers to deploy
    containers = platform_config.get('containers', [])
    if not containers:
        print(f"✗ No containers specified for platform {hardware_id}")
        return
    
    print(f"Containers: {', '.join(containers)}")
    
    # Verify containers exist in docker-compose.yaml
    if not verify_docker_compose_services(compose_config, containers):
        print(f"✗ Cannot deploy platform {hardware_id} due to missing container definitions")
        return
    
    # Create remote executor
    executor = RemoteExecutor(ip_address, access_port, local_mode)
    
    if stop_only:
        # Stop containers
        stop_containers(executor, containers, compose_file_path)
    else:
        # Deploy containers using docker-compose
        deploy_containers(executor, containers, ros_domain_id, compose_file_path)
    
    print(f"✓ Platform {hardware_id} {'stopped' if stop_only else 'deployed'} successfully")


def main():
    parser = argparse.ArgumentParser(description='Deploy distributed ROS2 system using docker-compose')
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
    parser.add_argument(
        '--stop',
        action='store_true',
        help='Stop containers instead of starting them'
    )
    
    args = parser.parse_args()
    
    # Hardcoded config file paths
    compose_file = 'config_files/docker-compose.yaml'
    deployment_config_path = 'config_files/deployment.yaml'
    hardware_config_path = 'config_files/hardware.yaml'
    
    # Load configurations
    print("Loading configuration files...")
    print(f"  Deployment config: {deployment_config_path}")
    print(f"  Hardware config: {hardware_config_path}")
    print(f"  Docker Compose config: {compose_file}")
    
    deployment_config = load_yaml_config(deployment_config_path)
    hardware_config = load_yaml_config(hardware_config_path)
    compose_config = load_yaml_config(compose_file)
    
    platforms = deployment_config.get('platforms', [])
    print(f"Loaded {len(platforms)} platforms from deployment config")
    
    # Deploy or stop platforms
    for platform_config in platforms:
        deploy_platform(
            platform_config,
            hardware_config,
            compose_config,
            args.local_mode,
            args.ros_domain_id,
            compose_file,
            args.stop
        )
    
    print(f"\n{'='*60}")
    print(f"{'Stop' if args.stop else 'Deployment'} complete!")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()