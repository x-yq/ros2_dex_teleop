# system_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directories
    config_dir = '/root/dev_ws/src/config_files'
    component_file_path = '/root/dev_ws/src/config_files/component_node'
    
    # Define all component nodes with their Hydra configurations
    nodes = [
        # System Manager (doesn't use Hydra)
        Node(
            package='system_manager',
            executable='system_manager_node',
            # name='system_manager',
            output='screen',
            parameters=[{
                'config_folder_path': config_dir
            }]
        ),
        
        # Input Sensor Driver (Rococo) - FIXED: Hydra format
        Node(
            package='input_sensor_driver',
            executable='input_sensor_driver_node',
            # name='input_sensor_driver_rococo',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'input_sensor_driver_rococo'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Input Data Processor - FIXED: Hydra format
        Node(
            package='input_data_processor',
            executable='input_data_processor_node',
            # name='input_control_data_processor',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'input_control_data_processor'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Motion Retarget - FIXED: Hydra format
        Node(
            package='motion_retarget',
            executable='motion_retarget_node',
            # name='motion_retarget',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'motion_retarget'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Robot Driver - FIXED: Hydra format
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            # name='robot_driver',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'robot_driver'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Feedback Sensor Driver (Realsense) - FIXED: Hydra format
        Node(
            package='feedback_sensor_driver',
            executable='feedback_sensor_driver_node',
            # name='feedback_sensor_driver_realsense',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'feedback_sensor_driver_realsense'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Feedback Data Processor - FIXED: Hydra format
        Node(
            package='feedback_data_processor',
            executable='feedback_data_processor_node',
            # name='feedback_data_processor',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'feedback_data_processor'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Feedback Projector Driver - FIXED: Hydra format
        Node(
            package='feedback_projector_driver',
            executable='feedback_projector_driver_node',
            # name='feedback_projector_driver',  # Removed - let Hydra handle node naming
            output='screen',
            # parameters=[
            #     'config_path', component_file_path,
            #     'config_name', 'feedback_projector_driver'
            # ],
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # KPI Monitor (doesn't use Hydra - keep as is)
        Node(
            package='kpi_monitor',
            executable='monitor_0',
            # name='monitor_0',
            output='screen',
            env={'ROS_DOMAIN_ID': '0'}
        ),
        
        # Data Collector (if needed)
        # Node(
        #     package='data_collector',
        #     executable='data_collector_node',
        #     name='data_collection',
        #     output='screen',
        #     env={'ROS_DOMAIN_ID': '0'}
        # ),
    ]
    
    # # Add event handler to ensure system_manager starts first
    # system_manager_node = nodes[0]
    # other_nodes = nodes[1:]
    
    # # Create launch description
    ld = LaunchDescription()
    # ld.add_action(system_manager_node)
    
    # Add other nodes with dependency on system_manager
    for node in nodes:
        ld.add_action(node)
    
    return ld