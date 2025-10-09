#!/usr/bin/env python3
"""
Deployment script for distributed ROS2 system using docker-compose.
Extended to support launching simulator container (--launch_env).
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
        """Execute a command either locally or remotely via SSH."""
        if self.local_mode:
            print(f"[LOCAL] Executing: {command}")
            return subprocess.run(command, shell=True, check=check, capture_output=True, text=True)
        else:
            ssh_command = f"ssh -p {self.port} {self.ip_address} '{command}'"
            print(f"[SSH {self.ip_address}] Executing: {command}")
            return subprocess.run(ssh_command, shell=True, check=check, capture_output=True, text=True)


def load_yaml_config(file_path: str) -> Dict:
    """Load and parse a YAML configuration file."""
    try:
        with open(file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        sys.exit(1)


def get_hardware_info(hardware_id: str, hardware_config: Dict) -> Optional[Dict]:
    """Get hardware information from hardware.yaml."""
    for platform in hardware_config.get('execution_plattforms', []):
        if platform['hardware_id'] == hardware_id:
            return platform
    for sensor in hardware_config.get('sensors', []):
        if sensor['hardware_id'] == hardware_id:
            return sensor
    for robot in hardware_config.get('robots', []):
        if robot['hardware_id'] == hardware_id:
            return robot
    return None


def verify_docker_compose_services(compose_config: Dict, containers: List[str]) -> bool:
    """Verify that all requested containers exist in docker-compose.yaml."""
    services = compose_config.get('services', {})
    missing = [c for c in containers if c not in services]
    if missing:
        print(f"✗ Missing containers in docker-compose.yaml: {', '.join(missing)}")
        return False
    return True


def deploy_containers(
    executor: RemoteExecutor,
    containers: List[str],
    ros_domain_id: str,
    compose_file_path: str = "config_files/docker-compose.yaml",
    extra_env: Optional[Dict[str, str]] = None
) -> bool:
    """Deploy specified containers using docker compose."""
    success = True
    extra_env = extra_env or {}

    for container in containers:
        print(f"\n--- Deploying container: {container} ---")
        
        # Combine environment variables
        env_vars = f"ROS_DOMAIN_ID={ros_domain_id}"
        for k, v in extra_env.items():
            env_vars += f" {k}={v}"

        # Docker compose command
        compose_cmd = f"{env_vars} docker compose -f {compose_file_path} up -d {container}"
        
        try:
            result = executor.execute(compose_cmd)
            print(f"✓ Successfully deployed container: {container}")

            # Show logs tail
            logs_cmd = f"docker logs --tail 10 {container}"
            log_result = executor.execute(logs_cmd, check=False)
            if log_result.stdout:
                print(f"Container logs:\n{log_result.stdout}")

        except subprocess.CalledProcessError as e:
            print(f"✗ Failed to deploy container: {container}")
            print(f"Error: {e.stderr}")
            success = False
            continue

    return success


def stop_containers(executor: RemoteExecutor, containers: List[str], compose_file_path: str = "config_files/docker-compose.yaml") -> bool:
    """Stop specified containers."""
    print(f"\n--- Stopping containers: {', '.join(containers)} ---")
    container_list = ' '.join(containers)
    compose_cmd = f"docker compose -f {compose_file_path} stop {container_list}"
    try:
        executor.execute(compose_cmd)
        print("✓ Successfully stopped containers")
        return True
    except subprocess.CalledProcessError as e:
        print(f"✗ Failed to stop containers: {e.stderr}")
        return False


def deploy_platform(
    platform_config: Dict,
    hardware_config: Dict,
    compose_config: Dict,
    local_mode: bool = True,
    ros_domain_id: str = "0",
    compose_file_path: str = "config_files/docker-compose.yaml",
    stop_only: bool = False,
    launch_env: bool = False
):
    """Deploy all containers for a specific hardware platform."""
    hardware_id = platform_config['hardware_id']
    containers = platform_config.get('containers', [])
    if not containers:
        print(f"✗ No containers specified for platform {hardware_id}")
        return

    # Identify environment container
    env_containers = [c for c in containers if isinstance(c, dict) and c.get('name') == 'simulator']
    normal_containers = [c for c in containers if not (isinstance(c, dict) and c.get('name') == 'simulator')]

    # Determine which to deploy
    if launch_env:
        if not env_containers:
            print(f"✗ No simulator container defined in {hardware_id}")
            return
        target_containers = env_containers
        print(f"Launching simulator on host (ignoring hardware_id {hardware_id})")
    else:
        target_containers = normal_containers
        if not target_containers:
            print(f"No non-simulator containers for {hardware_id}, skipping")
            return

    container_names = [c.get('name') if isinstance(c, dict) else c for c in target_containers]
    print(f"Containers to deploy: {', '.join(container_names)}")

    # Verify existence in compose file
    if not verify_docker_compose_services(compose_config, container_names):
        print(f"✗ Cannot deploy {hardware_id}: container missing in compose file")
        return

    # Create executor
    # Simulator always runs locally
    local_executor = True if launch_env else local_mode
    ip_address = "127.0.0.1" if launch_env else "127.0.0.1"
    executor = RemoteExecutor(ip_address, 22, local_executor)

    # Add simulator arguments if launching simulator
    extra_env = {}
    if launch_env and env_containers:
        env_info = env_containers[0]
        robot = env_info.get('robot', 'unknown')
        gripper = env_info.get('eef', 'unknown')
        extra_env['ROBOT'] = robot
        extra_env['GRIPPER'] = gripper
        print(f"→ Passing simulator args: ROBOT={robot}, GRIPPER={gripper}")

    if stop_only:
        stop_containers(executor, container_names, compose_file_path)
    else:
        deploy_containers(executor, container_names, ros_domain_id, compose_file_path, extra_env)


def main():
    parser = argparse.ArgumentParser(description='Deploy distributed ROS2 system using docker-compose')
    parser.add_argument('--ros-domain-id', default='0', help='ROS 2 Domain ID')
    parser.add_argument('--local-mode', action='store_true', default=True, help='Run locally (default)')
    parser.add_argument('--stop', action='store_true', help='Stop containers')
    parser.add_argument('--launch-env', action='store_true', help='Launch simulator/environment container on host')

    args = parser.parse_args()

    compose_file = 'config_files/docker-compose.yaml'
    deployment_config_path = 'config_files/deployment.yaml'
    hardware_config_path = 'config_files/hardware.yaml'

    print("Loading configuration files...")
    deployment_config = load_yaml_config(deployment_config_path)
    hardware_config = load_yaml_config(hardware_config_path)
    compose_config = load_yaml_config(compose_file)

    platforms = deployment_config.get('platforms', [])
    print(f"Loaded {len(platforms)} platforms")

    # Deploy platforms based on launch_env flag
    for platform_config in platforms:
        deploy_platform(
            platform_config,
            hardware_config,
            compose_config,
            args.local_mode,
            args.ros_domain_id,
            compose_file,
            args.stop,
            args.launch_env
        )

    print("\n" + "="*60)
    print(f"{'Stop' if args.stop else 'Deployment'} complete!")
    print("="*60)


if __name__ == "__main__":
    main()
