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

    def __init__(
        self,
        ip_address: str,
        local_mode: bool = True,
        ssh_user: str = "user",
        ssh_key_path: Optional[str] = None,
        ssh_password: Optional[str] = None,
    ):
        self.ip_address = ip_address
        self.local_mode = local_mode
        self.ssh_user = ssh_user
        self.ssh_key_path = ssh_key_path    # e.g. ~/.ssh/id_rsa
        self.ssh_password = ssh_password    # only used when no key is provided

    def _build_ssh_command(self, remote_command: str) -> str:
        """
        Build an SSH command that avoids interactive password prompts.

        Priority:
          1. Identity file  ->  ssh -i <key> ...
          2. Password        ->  sshpass -p <pw> ssh ...  (requires sshpass installed)
          3. Neither         ->  plain ssh (relies on ssh-agent / default ~/.ssh/id_* key)
        """
        base_opts = (
            "-o StrictHostKeyChecking=no "
            "-o ConnectTimeout=10 "
        )

        if self.ssh_key_path:
            key = os.path.expanduser(self.ssh_key_path)
            return (
                f"ssh {base_opts}-o BatchMode=yes "
                f"-i {key} {self.ssh_user}@{self.ip_address} '{remote_command}'"
            )
        elif self.ssh_password:
            # sshpass passes the password non-interactively; must be installed on host
            return (
                f"sshpass -p '{self.ssh_password}' "
                f"ssh {base_opts}-o PasswordAuthentication=yes "
                f"-o BatchMode=no "
                f"{self.ssh_user}@{self.ip_address} '{remote_command}'"
            )
        else:
            # Rely on ssh-agent or default ~/.ssh/id_* keys
            return (
                f"ssh {base_opts}-o BatchMode=yes "
                f"{self.ssh_user}@{self.ip_address} '{remote_command}'"
            )

    def execute(self, command: str, check: bool = True) -> subprocess.CompletedProcess:
        """Execute a command either locally or remotely via SSH."""
        if self.local_mode:
            print(f"[LOCAL] Executing: {command}")
            return subprocess.run(command, shell=True, check=check, capture_output=True, text=True)
        else:
            ssh_command = self._build_ssh_command(command)
            print(f"[SSH {self.ssh_user}@{self.ip_address}] Executing: {command}")
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

def sync_repo_to_remote(executor: RemoteExecutor, remote_dest: str, force_update: bool = False) -> bool:
    """
    Copy the local repository root to the remote machine.

    - Detects the repo root as the directory that contains this script.
    - Checks whether the destination folder already exists on the remote
      and prints a notice if it will be overwritten.
    - Uses rsync when available (incremental, fast), falls back to scp -r.
    """
    repo_root = os.path.dirname(os.path.abspath(__file__))
    repo_name = os.path.basename(repo_root)
    remote_path = os.path.join(remote_dest, repo_name)

    print(f"\n--- Syncing repository to {executor.ssh_user}@{executor.ip_address}:{remote_path} ---")
    print(f"    Local source : {repo_root}")

    # ── Check if destination already exists ──────────────────────────────────
    try:
        check_result = executor.execute(f"test -d {remote_path} && echo EXISTS || echo MISSING", check=False)
        if "EXISTS" in check_result.stdout:
            if not force_update:
                return remote_path
            print(f"    Remote folder already exists — will overwrite/update contents")
        else:
            print(f"    Remote folder not found — creating fresh copy")
            executor.execute(f"mkdir -p {remote_dest}", check=True)
    except subprocess.CalledProcessError as e:
        print(f"X Failed to check remote destination: {e.stderr}")
        return False

    # ── Build transfer command (rsync preferred, scp fallback) ───────────────
    ssh_base = (
        f"-o StrictHostKeyChecking=no "
        f"-o ConnectTimeout=10 "
    )

    if executor.ssh_password:
        ssh_bin = f"sshpass -p '{executor.ssh_password}' ssh"
    else:
        ssh_bin = "ssh"

    # Folders to exclude from transfer
    exclude_dirs = ["data"]

    # Try rsync first
    rsync_check = subprocess.run("which rsync", shell=True, capture_output=True)
    if rsync_check.returncode == 0:
        excludes = " ".join(f"--exclude '{d}/'" for d in exclude_dirs)
        transfer_cmd = (
            f"rsync -az --delete {excludes} "
            f"-e \"{ssh_bin} {ssh_base}\" "
            f"{repo_root}/ "
            f"{executor.ssh_user}@{executor.ip_address}:{remote_path}/"
        )
        tool = "rsync"
    else:
        # tar + ssh: pipe the archive through SSH, excluding unwanted dirs
        excludes = " ".join(f"--exclude='./{d}'" for d in exclude_dirs)
        transfer_cmd = (
            f"tar -czf - -C {repo_root} {excludes} . "
            f"| {ssh_bin} -o StrictHostKeyChecking=no "
            f"{executor.ssh_user}@{executor.ip_address} "
            f"'mkdir -p {remote_path} && tar -xzf - -C {remote_path}'"
        )
        tool = "tar+ssh"

    print(f"    Transfer tool: {tool}")

    try:
        result = subprocess.run(transfer_cmd, shell=True, check=True, capture_output=True, text=True)
        print(f"OK Repository synced successfully ({tool})")
        if result.stdout:
            print(result.stdout)
        return remote_path
    except subprocess.CalledProcessError as e:
        print(f"X Repository sync failed: {e.stderr}")
        return None
    
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
        print("Successfully stopped containers")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to stop containers: {e.stderr}")
        return False


def deploy_platform(
    platform_config: Dict,
    hardware_config: Dict,
    compose_config: Dict,
    local_mode: bool = True,
    ros_domain_id: str = "0",
    compose_file_path: str = "config_files/docker-compose.yaml",
    stop_only: bool = False,
    launch_env: bool = False,
    force_update: bool = False
):
    """Deploy all containers for a specific hardware platform."""
    hardware_id = platform_config['hardware_id']
    containers = platform_config.get('containers', [])
    if not containers:
        print(f"No containers specified for platform {hardware_id}")
        return

    # Identify environment container
    env_containers = [c for c in containers if isinstance(c, dict) and c.get('name') == 'simulator']
    normal_containers = [c for c in containers if not (isinstance(c, dict) and c.get('name') == 'simulator')]

    # Determine which to deploy
    if launch_env:
        if not env_containers:
            print(f"No simulator container defined in {hardware_id}")
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
        print(f"Cannot deploy {hardware_id}: container missing in compose file")
        return

    if launch_env:
        ip_address = "127.0.0.1"
        local_executor = True
        ssh_user = None
        ssh_password = None
    else:
        # Create executor
        hw_info = get_hardware_info(hardware_id, hardware_config)
        if hw_info is None:
            print(f"hardware_id '{hardware_id}' not found in hardware.yaml")
            return
        ip_address = hw_info.get('ip', '').strip()
        if not ip_address:
            print(f"No IP address defined for hardware_id '{hardware_id}'")
            return
        # Read SSH credentials from hardware.yaml
        ssh_user = hw_info.get('ssh_user') or None
        ssh_password = hw_info.get('ssh_password') or None
        local_executor = local_mode
        if not local_executor:
            print(f"→ Remote mode: will SSH to {hardware_id} at {ip_address}")

    executor = RemoteExecutor(ip_address, local_executor, ssh_user=ssh_user or "root", ssh_password=ssh_password)

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
        if not local_executor:
            remote_home = f"/home/{ssh_user or 'root'}"
            remote_repo_path = sync_repo_to_remote(executor, remote_home)
            if not remote_repo_path:
                print(f"Aborting deployment of {hardware_id}: repo sync failed")
                return
            # Make compose file path absolute on the remote
            compose_file_path = os.path.join(remote_repo_path, compose_file_path)
            print(f"-> Using remote compose file: {compose_file_path}")
        deploy_containers(executor, container_names, ros_domain_id, compose_file_path, extra_env)

def main():
    parser = argparse.ArgumentParser(description='Deploy distributed ROS2 system using docker-compose')
    parser.add_argument('--ros-domain-id', default='0', help='ROS 2 Domain ID')
    parser.add_argument('--local-mode', action='store_true', default=False, help='Run locally instead of SSH to remote hosts')
    parser.add_argument('--stop', action='store_true', help='Stop containers')
    parser.add_argument('--launch-env', action='store_true', help='Launch simulator/environment container on host')
    parser.add_argument('--force-update', action='store_true', help='If remote exits the folder, force update')

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
            args.launch_env,
            args.force_update
        )

    print("\n" + "="*60)
    print(f"{'Stop' if args.stop else 'Deployment'} complete!")
    print("="*60)


if __name__ == "__main__":
    main()