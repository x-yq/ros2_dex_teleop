from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
import os
import subprocess

from utils.utils import count_nodes_from_config, get_exec_platform_id, check_monitor_node_exists

def launch_setup(context, *args, **kwargs):
    # Resolve LaunchConfiguration values
    config_folder_path = context.launch_configurations['config_folder_path']
    ros_domain_id = context.launch_configurations['ros_domain_id']
    
    # Count nodes from config file
    config_file_path = f"{config_folder_path}/deployment.yaml"
    node_count = count_nodes_from_config(config_file_path)

    # Path to deployment.yaml (adjust this path as needed for your setup)
    deployment_config_path = os.path.join(config_folder_path, "deployment.yaml")
    
    # # Get the current launch file path dynamically
    # current_launch_file = os.path.abspath(__file__)
    
    # Get hardware_id for this launch file
    hardware_id = get_exec_platform_id(deployment_config_path)
    
    nodes = []
    
    # Determine monitor node name
    if hardware_id:
        hardware_id = hardware_id.replace('-', '_')
        print("hardware id!!!!!!!!!!", hardware_id)
        monitor_node_name = f"monitor_{hardware_id}"
    else:
        # Fallback to default if hardware_id not found
        monitor_node_name = "monitor_0"
        print(f"Warning: Could not determine hardware_id, using default monitor name: {monitor_node_name}")
    
    # Check if monitor node already exists in the network
    monitor_exists = check_monitor_node_exists(monitor_node_name, ros_domain_id)
    
    if not monitor_exists:
        print(f"Launching monitor node: {monitor_node_name}")
        nodes.append(
            Node(
                package="kpi_monitor",
                executable="kpi_monitor_node",
                name=monitor_node_name,
                output="screen",
                arguments=["--layer_cn_id", monitor_node_name, "--ros2_domain_id", ros_domain_id]
            ),
        )
    else:
        print(f"Monitor node {monitor_node_name} already exists in the network, skipping launch")
    
        
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
        # Node(
        #     package="kpi_monitor",
        #     executable="kpi_monitor_node",
        #     name="monitor_sysManager",
        #     output="screen",
        #     arguments=["--layer_cn_id", "monitor_sysManager", "--ros2_domain_id", ros_domain_id]
        # ),
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