import yaml
import os


def get_exec_platform_id(config_path, container_name=None):
    """
    Get the hardware_id (execution platform) for the given container name.
    If container_name is not provided, it will try to detect it from the environment.
    
    Args:
        config_path: Path to the deployment.yaml file
        container_name: Name of the container (optional, will auto-detect if not provided)
    
    Returns:
        hardware_id (str) or None if not found
    """
    try:
        # If container name not provided, try to get it from hostname
        if container_name is None:
            container_name = os.environ.get('CONTAINER_NAME')
            if not container_name:
                # Try reading from /etc/hostname as fallback
                try:
                    with open('/etc/hostname', 'r') as f:
                        container_name = f.read().strip()
                except:
                    print("Warning: Could not determine container name")
                    return None
        
        # Load deployment config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Search through platforms and containers
        for platform in config.get('platforms', []):
            hardware_id = platform.get('hardware_id')
            containers = platform.get('containers', [])
            
            # Check if container_name matches any container in this platform
            for container in containers:
                # Handle both formats: dict with 'name' field or plain string
                if isinstance(container, dict):
                    if container.get('name') == container_name:
                        return hardware_id, container_name
                elif isinstance(container, str):
                    if container == container_name:
                        return hardware_id, container_name
        
        print(f"Warning: Container '{container_name}' not found in deployment config")
        return None
        
    except FileNotFoundError:
        print(f"Error: Config file not found at {config_path}")
        return None
    except Exception as e:
        print(f"Error reading deployment config: {e}")
        return None


def count_nodes_from_config(config_path: str) -> int:
    """
    Count the total number of nodes defined in deployment.yaml.
    
    Expected deployment.yaml structure example:
    
    platforms:
      - hardware_id: mac_pc
        containers:
          - name: teleop-system
            node_count: 3
      - hardware_id: pi4-197
        containers:
          - name: teleop-components
            node_count: 8
    
    The node_count for each container includes ALL nodes (monitor + component nodes).
    
    Returns:
        int: Total number of nodes across all platforms
    """
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        total_nodes = 0
        
        if 'platforms' in config:
            for platform in config['platforms']:
                # Sum up node counts from all containers in this platform
                if 'containers' in platform:
                    for container in platform['containers']:
                        if isinstance(container, dict) and 'node_count' in container:
                            total_nodes += container['node_count']
                        elif isinstance(container, str):
                            # If container is just a string name without node_count
                            print(f"Warning: Container '{container}' has no node_count specified")
        
        return total_nodes
    
    except FileNotFoundError:
        print(f"Error: Config file not found at {config_path}")
        return 10
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
        return 10
    except Exception as e:
        print(f"Error counting nodes: {e}")
        return 10


def check_monitor_node_exists(monitor_node_name, ros_domain_id):
    """
    Check if a monitor node already exists in the ROS network.
    
    Args:
        monitor_node_name: Name of the monitor node to check
        ros_domain_id: ROS 2 Domain ID
    
    Returns:
        bool: True if node exists, False otherwise
    """
    import subprocess
    
    try:
        # Set ROS_DOMAIN_ID for this check
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(ros_domain_id)
        
        # Run ros2 node list command
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5,
            env=env
        )
        
        # Check if monitor node is in the list
        nodes = result.stdout.strip().split('\n')
        full_node_name = f'/{monitor_node_name}'
        
        return full_node_name in nodes
        
    except subprocess.TimeoutExpired:
        print("Warning: Timeout while checking for existing monitor node")
        return False
    except Exception as e:
        print(f"Warning: Could not check for existing monitor node: {e}")
        return False