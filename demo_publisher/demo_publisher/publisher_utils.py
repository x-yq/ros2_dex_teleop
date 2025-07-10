"""
Utility functions for publisher nodes
"""
import numpy as np
from geometry_msgs.msg import Point, Transform, Vector3, Quaternion
from std_msgs.msg import Header


def create_header(node, frame_id="base_link"):
    """Create a standard ROS header"""
    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame_id
    return header


def generate_sinusoidal_joint_positions(joint_count, time_value, amplitude=0.5, frequency=1.0):
    """Generate sinusoidal joint positions for simulation"""
    positions = []
    for i in range(joint_count):
        phase = i * (2 * np.pi / joint_count)
        pos = amplitude * np.sin(frequency * time_value + phase)
        positions.append(pos)
    return positions


def generate_hand_keypoints(time_value, hand_center=(0.5, 0.5, 0.0), num_keypoints=21):
    """Generate simulated hand keypoints"""
    keypoints = []
    for i in range(num_keypoints):
        point = Point()
        angle = i * (2 * np.pi / num_keypoints)
        radius = 0.1
        
        point.x = hand_center[0] + radius * np.cos(angle + time_value)
        point.y = hand_center[1] + radius * np.sin(angle + time_value)
        point.z = hand_center[2]
        keypoints.append(point)
    
    return keypoints


def create_camera_transform(x=0.5, y=0.0, z=1.0):
    """Create a camera transform"""
    transform = Transform()
    transform.translation = Vector3(x=x, y=y, z=z)
    transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return transform