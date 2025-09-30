from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
import os
import subprocess


def get_exec_platform_id(config_path, launch_file_path):
    """
    Get the hardware_id (execution platform) for the given launch file path.
    
    Args:
        config_path: Path to the deployment.yaml file
        launch_file_path: Path to the current launch file (relative or absolute)
    
    Returns:
        hardware_id (str) or None if not found
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Normalize the launch file path for comparison
        normalized_launch_path = os.path.normpath(launch_file_path)
        
        # Search through platforms and containers
        for platform in config.get('platforms', []):
            hardware_id = platform.get('hardware_id')
            for container in platform.get('containers', []):
                container_launch_path = os.path.normpath(container.get('launch_file_path', ''))
                
                # Check if paths match (handle both relative and absolute paths)
                if container_launch_path.endswith(normalized_launch_path) or \
                   normalized_launch_path.endswith(container_launch_path):
                    return hardware_id
        
        return None
    except Exception as e:
        print(f"Error reading deployment config: {e}")
        return None

def check_monitor_node_exists(monitor_node_name, ros_domain_id):
    """
    Check if a monitor node with the given name already exists in the ROS2 network.
    
    Args:
        monitor_node_name: Name of the monitor node to check (e.g., "monitor_pi4-37")
        ros_domain_id: ROS 2 Domain ID
    
    Returns:
        True if node exists, False otherwise
    """
    try:
        # Set ROS_DOMAIN_ID for the subprocess
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(ros_domain_id)
        
        # Use ros2 node list to check for existing nodes
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=env
        )
        
        # Check if monitor node is in the list
        node_list = result.stdout.strip().split('\n')
        full_node_name = f'/{monitor_node_name}'
        
        return full_node_name in node_list
    except Exception as e:
        print(f"Error checking for existing monitor node: {e}")
        # If we can't check, assume it doesn't exist (safe to launch)
        return False

def count_nodes_from_config(config_path: str) -> int:
    """
    Count the total number of nodes defined in container.yaml
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        total_nodes = 0
        
        if 'platforms' in config:
            for platform in config['platforms']:
                if 'containers' in platform:
                    total_nodes += 1  # each platform has one monitor node
                    for container in platform['containers']:
                        if 'nodes' in container:
                            total_nodes += len(container['nodes'])
        
        return total_nodes
    
    except FileNotFoundError:
        print(f"Error: Config file not found at {config_path}")
        return 0
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return 0
