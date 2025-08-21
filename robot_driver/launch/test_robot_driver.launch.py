# test_robot_driver.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            output='screen',
            env={'ROS_DOMAIN_ID': '0'}
        )
    ])