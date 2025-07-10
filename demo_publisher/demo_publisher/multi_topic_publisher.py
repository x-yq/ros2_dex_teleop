#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from interfaces.msg import (
    ArmJointStates, 
    HandJointStates, 
    HandKeypoint, 
    VizSensorData
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import numpy as np
import time
from .publisher_utils import (
    create_header,
    generate_sinusoidal_joint_positions,
    generate_hand_keypoints,
    create_camera_transform
)


class MultiTopicPublisher(Node):
    """Multi-topic publisher with frequency control for each topic"""
    
    def __init__(self):
        super().__init__('multi_topic_publisher')
        
        # Declare parameters with descriptions
        self.declare_parameter(
            'arm_joint_states_freq', 
            50.0,
            ParameterDescriptor(description='Arm joint states publishing frequency (Hz)')
        )
        self.declare_parameter(
            'hand_joint_states_freq', 
            50.0,
            ParameterDescriptor(description='Hand joint states publishing frequency (Hz)')
        )
        self.declare_parameter(
            'hand_keypoint_freq', 
            30.0,
            ParameterDescriptor(description='Hand keypoint publishing frequency (Hz)')
        )
        self.declare_parameter(
            'viz_sensor_freq', 
            10.0,
            ParameterDescriptor(description='Visualization sensor data publishing frequency (Hz)')
        )
        self.declare_parameter(
            'sensor_id', 
            'camera_1',
            ParameterDescriptor(description='Sensor ID for visualization data')
        )
        self.declare_parameter(
            'simulation_mode', 
            True,
            ParameterDescriptor(description='Enable simulation mode with generated data')
        )
        
        # Get parameters
        self.arm_freq = self.get_parameter('arm_joint_states_freq').get_parameter_value().double_value
        self.hand_freq = self.get_parameter('hand_joint_states_freq').get_parameter_value().double_value
        self.keypoint_freq = self.get_parameter('hand_keypoint_freq').get_parameter_value().double_value
        self.viz_freq = self.get_parameter('viz_sensor_freq').get_parameter_value().double_value
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().string_value
        self.simulation_mode = self.get_parameter('simulation_mode').get_parameter_value().bool_value
        
        # Create publishers
        self.arm_pub = self.create_publisher(ArmJointStates, '/arm_joint_states', 10)
        self.hand_pub = self.create_publisher(HandJointStates, '/hand_joint_states', 10)
        self.keypoint_pub = self.create_publisher(HandKeypoint, '/hand_keypoint', 10)
        self.viz_pub = self.create_publisher(VizSensorData, f'/viz_{self.sensor_id}', 10)
        
        # Create timers
        self.arm_timer = self.create_timer(1.0/self.arm_freq, self.publish_arm_states)
        self.hand_timer = self.create_timer(1.0/self.hand_freq, self.publish_hand_states)
        self.keypoint_timer = self.create_timer(1.0/self.keypoint_freq, self.publish_hand_keypoints)
        self.viz_timer = self.create_timer(1.0/self.viz_freq, self.publish_viz_data)
        
        # Initialize counters
        self.arm_count = 0
        self.hand_count = 0
        self.keypoint_count = 0
        self.viz_count = 0
        
        # Parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Log initialization
        self.get_logger().info('Multi-topic publisher initialized:')
        self.get_logger().info(f'  Arm joints: {self.arm_freq} Hz')
        self.get_logger().info(f'  Hand joints: {self.hand_freq} Hz')
        self.get_logger().info(f'  Hand keypoints: {self.keypoint_freq} Hz')
        self.get_logger().info(f'  Viz sensor: {self.viz_freq} Hz')
        self.get_logger().info(f'  Sensor ID: {self.sensor_id}')
        self.get_logger().info(f'  Simulation mode: {self.simulation_mode}')
        
    def parameter_callback(self, params):
        """Handle parameter changes"""
        for param in params:
            if param.name == 'arm_joint_states_freq':
                self.arm_freq = param.value
                self.arm_timer.cancel()
                self.arm_timer = self.create_timer(1.0/self.arm_freq, self.publish_arm_states)
                self.get_logger().info(f'Arm frequency updated to {self.arm_freq} Hz')
            elif param.name == 'hand_joint_states_freq':
                self.hand_freq = param.value
                self.hand_timer.cancel()
                self.hand_timer = self.create_timer(1.0/self.hand_freq, self.publish_hand_states)
                self.get_logger().info(f'Hand frequency updated to {self.hand_freq} Hz')
            elif param.name == 'hand_keypoint_freq':
                self.keypoint_freq = param.value
                self.keypoint_timer.cancel()
                self.keypoint_timer = self.create_timer(1.0/self.keypoint_freq, self.publish_hand_keypoints)
                self.get_logger().info(f'Keypoint frequency updated to {self.keypoint_freq} Hz')
            elif param.name == 'viz_sensor_freq':
                self.viz_freq = param.value
                self.viz_timer.cancel()
                self.viz_timer = self.create_timer(1.0/self.viz_freq, self.publish_viz_data)
                self.get_logger().info(f'Viz sensor frequency updated to {self.viz_freq} Hz')
                
        return rclpy.parameter.SetParametersResult(successful=True)
    
    def publish_arm_states(self):
        """Publish arm joint states"""
        msg = ArmJointStates()
        msg.header = create_header(self, "base_link")
        
        msg.joint_names = [
            "joint1", "joint2", "joint3", 
            "joint4", "joint5"
        ]
        
        if self.simulation_mode:
            t = self.arm_count * (1.0/self.arm_freq)
            msg.positions = generate_sinusoidal_joint_positions(5, t, amplitude=0.5, frequency=0.5)
            msg.velocities = [0.1 * np.cos(t + i) for i in range(5)]
            msg.efforts = [0.5 * np.sin(t + i * 0.5) for i in range(5)]
        else:
            msg.positions = [0.0] * 5
            msg.velocities = [0.0] * 5
            msg.efforts = [0.0] * 5
        
        self.arm_pub.publish(msg)
        self.arm_count += 1
        
    def publish_hand_states(self):
        """Publish hand joint states"""
        msg = HandJointStates()
        msg.header = create_header(self, "hand_link")
        
        msg.joint_names = [
            "thumb_mcp", "thumb_pip", "thumb_dip", "thumb_tip",
            "index_mcp", "index_pip", "index_dip", "index_tip",
            "middle_mcp", "middle_pip", "middle_dip", "middle_tip",
            "ring_mcp", "ring_pip", "ring_dip", "ring_tip",
            "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"
        ]
        
        if self.simulation_mode:
            t = self.hand_count * (1.0/self.hand_freq)
            msg.positions = generate_sinusoidal_joint_positions(20, t, amplitude=0.3, frequency=0.8)
            msg.velocities = [0.05 * np.cos(t + i * 0.1) for i in range(20)]
            msg.efforts = [0.1 * np.sin(t + i * 0.2) for i in range(20)]
        else:
            # Here you would integrate with actual hardware
            msg.positions = [0.0] * 20
            msg.velocities = [0.0] * 20
            msg.efforts = [0.0] * 20
        
        self.hand_pub.publish(msg)
        self.hand_count += 1
        
    def publish_hand_keypoints(self):
        """Publish hand keypoints"""
        msg = HandKeypoint()
        msg.header = create_header(self, "camera_link")
        
        msg.hand_id = 0  # Right hand
        
        if self.simulation_mode:
            t = self.keypoint_count * (1.0/self.keypoint_freq)
            msg.keypoints = generate_hand_keypoints(t)
        else:
            # Here you would integrate with actual hand tracking (Oculus API)
            msg.keypoints = [Point(x=0.0, y=0.0, z=0.0) for _ in range(21)]
        
        self.keypoint_pub.publish(msg)
        self.keypoint_count += 1
        
    def publish_viz_data(self):
        """Publish visualization sensor data"""
        msg = VizSensorData()
        msg.header = create_header(self, "camera_link")
        
        msg.sensor_id = self.sensor_id
        
        # Create image message
        img_msg = Image()
        img_msg.header = msg.header
        img_msg.height = 480
        img_msg.width = 640
        img_msg.encoding = "rgb8"
        img_msg.step = img_msg.width * 3
        
        if self.simulation_mode:
            # Generate colored pattern
            data = []
            for y in range(img_msg.height):
                for x in range(img_msg.width):
                    r = int(127 * (1 + np.sin(x * 0.01 + time.time())))
                    g = int(127 * (1 + np.cos(y * 0.01 + time.time())))
                    b = int(127 * (1 + np.sin((x + y) * 0.01 + time.time())))
                    data.extend([r, g, b])
            img_msg.data = data
        else:
            # Here you would integrate with actual camera
            img_msg.data = [0] * (img_msg.height * img_msg.step)
        
        msg.image = img_msg
        msg.sensor_pose = create_camera_transform()
        
        self.viz_pub.publish(msg)
        self.viz_count += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiTopicPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()