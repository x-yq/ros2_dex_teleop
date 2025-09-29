from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

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

def launch_setup(context, *args, **kwargs):
    # Resolve LaunchConfiguration values
    config_folder_path = context.launch_configurations['config_folder_path']
    ros_domain_id = context.launch_configurations['ros_domain_id']
    
    # Count nodes from config file
    config_file_path = f"{config_folder_path}/container.yaml"
    node_count = count_nodes_from_config(config_file_path)
    
    nodes = []
    
    # System manager node with resolved node count
    nodes.append(
        Node(
            package="system_manager",
            executable="system_manager_node",
            name="system_manager",
            output="screen",
            arguments=[
                "--config_folder_path", config_folder_path, 
                "--ros2_domain_id", ros_domain_id, 
                "--node_num", str(node_count)
            ]
        )
    )
    
    # Other nodes
    nodes.extend([
        Node(
            package="kpi_monitor",
            executable="kpi_monitor_node",
            name="monitor_sysManager",
            output="screen",
            arguments=["--layer_cn_id", "monitor_sysManager", "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="data_collector",
            executable="data_collector_node",
            name="data_collector",
            output="screen",
            arguments=["--layer_cn_id", "data_collector", "--ros2_domain_id", ros_domain_id]
        ),
    ])
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("ros_domain_id", default_value="0", description="ROS 2 Domain ID"),
        DeclareLaunchArgument("config_folder_path", default_value="/root/dev_ws/src/config_files"),
        
        # Set ROS domain
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),

        # Use OpaqueFunction to resolve LaunchConfiguration values at runtime
        OpaqueFunction(function=launch_setup)
    ])