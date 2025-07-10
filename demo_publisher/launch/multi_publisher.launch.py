from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('demo_publisher')
    config_file = os.path.join(pkg_dir, 'config', 'publisher_config.yaml')
    
    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )
    
    # Multi-topic publisher node
    multi_publisher_node = Node(
        package='demo_publisher',
        executable='multi_topic_publisher',
        name='multi_topic_publisher',
        parameters=[LaunchConfiguration('config_file')],
        output='screen'
    )
    
    return LaunchDescription([
        config_arg,
        multi_publisher_node
    ])