from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml

from utils.utils import count_nodes_from_config

def launch_setup(context, *args, **kwargs):
    # Resolve LaunchConfiguration values
    config_folder_path = context.launch_configurations['config_folder_path']
    ros_domain_id = context.launch_configurations['ros_domain_id']
    
    # Count nodes from config file
    config_file_path = f"{config_folder_path}/deployment.yaml"
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