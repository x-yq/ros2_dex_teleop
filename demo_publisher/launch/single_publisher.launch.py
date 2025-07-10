from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='10.0',
        description='Publishing frequency in Hz'
    )
    
    topic_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/arm_joint_states',
        description='Topic name to publish to'
    )
    
    # Single topic publisher node
    single_publisher_node = Node(
        package='demo_publisher',
        executable='frequency_controlled_publisher',
        name='frequency_controlled_publisher',
        parameters=[
            {'publish_frequency': LaunchConfiguration('frequency')},
            {'topic_name': LaunchConfiguration('topic_name')},
            {'joint_count': 6},
            {'simulation_mode': True}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        frequency_arg,
        topic_arg,
        single_publisher_node
    ])