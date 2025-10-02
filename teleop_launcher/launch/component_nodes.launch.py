from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml
import os
import subprocess

from utils.utils import get_exec_platform_id, check_monitor_node_exists


def launch_setup(context, *args, **kwargs):
    # Resolve LaunchConfiguration values
    ros_domain_id = context.launch_configurations['ros_domain_id']
    config_folder_path = context.launch_configurations['config_folder_path']
    
    
    # Path to deployment.yaml (adjust this path as needed for your setup)
    deployment_config_path = os.path.join(config_folder_path, "deployment.yaml")
    
    # # Get the current launch file path dynamically
    # current_launch_file = os.path.abspath(__file__)
    
    # Get hardware_id for this launch file
    hardware_id = get_exec_platform_id(deployment_config_path)
    hardware_id = hardware_id.replace('-', '_')

    print("hardware id!!!!!!!!!!", hardware_id)
    
    nodes = []
    
    # Determine monitor node name
    if hardware_id:
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
    
    # Extend your nodes here
    nodes.extend([
        Node(
            package="input_sensor_driver",
            executable="input_sensor_driver_node",
            name="input_sensor_driver",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['input_sensor_driver'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="input_data_processor",
            executable="input_data_processor_node",
            name="input_data_processor",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['input_control_data_processor'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="motion_retarget",
            executable="motion_retarget_node",
            name="motion_retarget",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['motion_retarget'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="robot_driver",
            executable="robot_driver_node",
            name="robot_driver",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['robot_driver'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_sensor_driver",
            executable="feedback_sensor_driver_node",
            name="feedback_sensor_driver",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['feedback_sensor_driver'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_data_processor",
            executable="feedback_data_processor_node",
            name="feedback_data_processor",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['feedback_data_processor'], "--ros2_domain_id", ros_domain_id]
        ),
        Node(
            package="feedback_projector_driver",
            executable="feedback_projector_driver_node",
            name="feedback_projector_driver",
            output="screen",
            arguments=["--layer_cn_id", context.launch_configurations['feedback_projector_driver'], "--ros2_domain_id", ros_domain_id]
        ),
    ])
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument("ros_domain_id", default_value="0", description="ROS 2 Domain ID"),
        DeclareLaunchArgument("config_folder_path", default_value="/root/dev_ws/src/config_files"),
        
        DeclareLaunchArgument("input_sensor_driver", default_value="input_sensor_driver_rococo"),
        DeclareLaunchArgument("input_control_data_processor", default_value="input_control_data_processor"),
        DeclareLaunchArgument("motion_retarget", default_value="motion_retarget"),
        DeclareLaunchArgument("robot_driver", default_value="robot_driver"),
        DeclareLaunchArgument("feedback_sensor_driver", default_value="feedback_sensor_driver_realsense"),
        DeclareLaunchArgument("feedback_data_processor", default_value="feedback_data_processor"),
        DeclareLaunchArgument("feedback_projector_driver", default_value="feedback_projector_driver"),

        # Set ROS domain
        SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),

        # Use OpaqueFunction to resolve LaunchConfiguration values at runtime
        OpaqueFunction(function=launch_setup)
    ])