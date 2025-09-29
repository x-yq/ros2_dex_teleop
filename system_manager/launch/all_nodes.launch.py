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
        Node(
            package="kpi_monitor",
            executable="kpi_monitor_node",
            name="monitor_0",
            output="screen",
            arguments=["--layer_cn_id", "monitor_0", "--ros2_domain_id", ros_domain_id]
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