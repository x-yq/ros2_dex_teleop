from core_bases.motion_retarget_base import MotionRetargetBase
from typing import Dict, List, Any, Optional
import rclpy
import yaml
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import tf_transformations


# Import message types based on the interface definitions
from component_interfaces.msg import (
    ProcessedHandKeypoint,
    JointAngles
)
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Transform
from builtin_interfaces.msg import Time, Duration

class MotionRetargetOT(MotionRetargetBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific retargeter
        self.processing_frequency = 90  # Hz
        self.is_retargeting_active = True
        self.retargeting_counter = 0
        
        # Robot-specific parameters
        self.robot_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint_eef"]
        self.robot_joint_limits = [
            (-3.14, 3.14), (-2.0, 2.0), (-3.14, 3.14), 
            (-3.14, 3.14), (-3.14, 3.14)
        ]
        
        # Initialize retargeting parameters
        self._initialize_node()
        # self._wait_for_start_cmd()

    def _initialize_node(self):
        """Override the base initialization for MotionRetargetOT specific setup"""
        self.get_logger().info(f"Initializing MotionRetargetOT: {self.layer_cn_id}")

        self._load_config()
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        if self.check_plugins():
            self.set_state("ready")
            
        self.get_logger().info("MotionRetargetOT initialized successfully")

    def _load_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'motion_retarget_ot' or plugin.get('_target_', '').endswith('MotionRetargetOT'):
                        plugin_params = plugin.get('params', {})
                        self.processing_frequency = plugin_params.get("freq", self.processing_frequency)
                        self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
                        break
            
            # Load calibration configuration
            self.calibration_config = self.load_calibration_config()
            if self.calibration_config:
                self.get_logger().info("Loaded calibration config")
            
            # Load data stream configuration
            self.data_stream_config = self.load_data_stream_config()
            if self.data_stream_config:
                self.get_logger().info("Loaded data stream config")
                
                # Extract topic information from data stream config
                if 'topics' in self.data_stream_config:
                    self.get_logger().info(f"Found {len(self.data_stream_config['topics'])} topics in data stream config")
            
            # # Load hardware configuration (optional)
            # hardware_config = self.load_hardware_config(['robots'])
            # if hardware_config and 'robots' in hardware_config:
            #     robot_config = hardware_config['robots'].get('xarm5_robot', {})
            #     if 'joint_names' in robot_config:
            #         self.robot_joint_names = robot_config['joint_names']
            #     if 'joint_limits' in robot_config:
            #         self.robot_joint_limits = robot_config['joint_limits']
            #     self.get_logger().info(f"Loaded hardware config for robot: {self.robot_joint_names}")
                
        except Exception as e:
            self.handle_error(f"Failed to load configurations: {str(e)}")

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of loading again
            if not self.data_stream_config or 'topics' not in self.data_stream_config:
                self.get_logger().warn("No data stream configuration available")
                return
            
            # Find topics where this node is a subscriber
            for topic_config in self.data_stream_config['topics']:
                topic_id = topic_config["id"]
                
                # Subscribe to topics where this node is a subscriber
                if self.layer_cn_id in topic_config.get("subscribers", []):
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
                        self.control_data_subscribers[topic_id] = self.create_subscription(
                            ProcessedHandKeypoint,
                            topic_id,
                            self.hand_keypoint_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to hand keypoints: {topic_id}")
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/JointAngles":
                        self.cmd_publishers[topic_id] = self.create_publisher(
                            JointAngles,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for joint angles: {topic_id}")
        
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: {str(e)}")

    # def publish_cmd(self, topic_id: str, msg: Any) -> bool:
    #     """
    #     Publish command message to the specified topic.
        
    #     Args:
    #         topic_id: The ID of the topic to publish to
    #         msg: The message to publish
            
    #     Returns:
    #         bool: True if successful, False otherwise
    #     """
    #     try:
    #         # Check if we have a publisher for this topic
    #         if topic_id not in self.cmd_publishers:
    #             self.get_logger().warn(f"No publisher found for topic: {topic_id}")
    #             return False
            
    #         # Get the publisher
    #         publisher = self.cmd_publishers[topic_id]
            
    #         # Publish the message
    #         publisher.publish(msg)
            
    #         self.get_logger().debug(f"Published command to {topic_id}")
    #         return True
            
    #     except Exception as e:
    #         self.handle_error(f"Error publishing command to {topic_id}: {str(e)}")
    #         return False
            
    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""
        if self.state!='running':
            return  

        self.get_logger().debug(f"Received processed hand keypoints with {len(msg.joint_names)} joints")
        
        # Retarget the hand data to robot commands
        if self.is_retargeting_active:
            self.retarget_hand_data(msg)

    def retarget_hand_data(self, hand_data: ProcessedHandKeypoint):
        """Retarget hand keypoints to robot joint angles"""
        try:
            self.retargeting_counter += 1
            self.get_logger().debug(f"Retargeting hand data #{self.retargeting_counter}")
            
            # Get static transforms (dummy implementation)
            transforms = self._get_required_transforms()
            
            # Retarget control data to robot space
            joint_angles_msg = self.retarget_control_to_robot_space(hand_data, transforms)
            
            # Publish robot command
            self.publish_cmd("joint_angles_cmd", joint_angles_msg)
            
            self.get_logger().debug("Hand data retargeting completed successfully")
            
        except Exception as e:
            self.handle_error(f"Error retargeting hand data: {str(e)}")
    
    def quaternion_to_euler(self, quat):
        """Debug version to see what format is expected"""
        self.get_logger().debug(f"quaternion_to_euler received: {quat}, type: {type(quat)}")
        
        try:
            # Log the length if it's a sequence
            if hasattr(quat, '__len__'):
                self.get_logger().debug(f"Quaternion length: {len(quat)}")
            
            q = [quat.x, quat.y, quat.z, quat.w]
            return tf_transformations.euler_from_quaternion(q)
            
        except Exception as e:
            self.handle_error(f"quaternion_to_euler failed: {e}")
            raise

    def retarget_control_to_robot_space(
        self, control_data: ProcessedHandKeypoint, transforms: Dict[str, Any]
    ) -> JointAngles:
        """
        Simple dummy implementation that returns basic joint angles.
        """
        joint_angles_msg = JointAngles()
        joint_angles_msg.header = control_data.header
        joint_angles_msg.header.stamp = self.get_clock().now().to_msg()
        joint_angles_msg.robot_id = "xarm5_robot"
        joint_angles_msg.joint_names = self.robot_joint_names
        joint_angles_msg.timeout = Duration(sec=1, nanosec=0)

        # Simple dummy joint angles (in radians)
        # Adjust the number based on your robot_joint_names length
        if len(self.robot_joint_names) == 5:
            # For 5-DOF arm
            joint_angles_msg.joint_angles = [0.1, -0.2, 0.3, 0.0, 0.1]
        elif len(self.robot_joint_names) == 6:
            # For 6-DOF arm  
            joint_angles_msg.joint_angles = [0.1, -0.2, 0.3, 0.0, 0.1, -0.05]
        else:
            # Generic: create zeros for however many joints
            joint_angles_msg.joint_angles = [0.0] * len(self.robot_joint_names)
        
        # Set joint velocities (same length as joint_angles)
        joint_angles_msg.joint_velocities = [0.5] * len(joint_angles_msg.joint_angles)

        self.logging(f"Sent dummy joint angles: {joint_angles_msg.joint_angles}", "INFO")
        
        return joint_angles_msg

    def _apply_joint_limit(self, angle: float, joint_index: int) -> float:
        """Apply joint limits to a given angle"""
        lower, upper = self.robot_joint_limits[joint_index]
        return np.clip(angle, lower, upper)

    def _get_required_transforms(self) -> Dict[str, Any]:
        """Get required transforms for retargeting (dummy implementation)"""
        # In a real implementation, this would fetch actual transforms
        transforms = {}
        
        # Dummy hand-to-robot transform
        hand_to_robot = Transform()
        hand_to_robot.translation.x = 0.5
        hand_to_robot.translation.y = 0.2
        hand_to_robot.translation.z = 0.3
        hand_to_robot.rotation.x = 0.0
        hand_to_robot.rotation.y = 0.0
        hand_to_robot.rotation.z = 0.0
        hand_to_robot.rotation.w = 1.0
        
        transforms["hand_to_robot"] = hand_to_robot
        
        # Dummy camera-to-world transform
        cam_to_world = Transform()
        cam_to_world.translation.x = 0.0
        cam_to_world.translation.y = 0.0
        cam_to_world.translation.z = 1.5
        cam_to_world.rotation.x = 0.707
        cam_to_world.rotation.y = 0.0
        cam_to_world.rotation.z = 0.0
        cam_to_world.rotation.w = 0.707
        
        transforms["cam_to_world"] = cam_to_world
        
        return transforms
    
    # def _get_required_transforms(self) -> Dict[str, Any]:
    #     """Load VR/Glove → Robot calibration transform"""
    #     transforms = {}
    #     try:
    #         config_data = self.get_config("calibration.yaml")
    #         if config_data:
    #             calib = yaml.safe_load(config_data)
    #             t = calib.get("vr_to_robot", {})
                
    #             hand_to_robot = Transform()
    #             hand_to_robot.translation.x = t["translation"]["x"]
    #             hand_to_robot.translation.y = t["translation"]["y"]
    #             hand_to_robot.translation.z = t["translation"]["z"]
    #             hand_to_robot.rotation.x = t["rotation"]["x"]
    #             hand_to_robot.rotation.y = t["rotation"]["y"]
    #             hand_to_robot.rotation.z = t["rotation"]["z"]
    #             hand_to_robot.rotation.w = t["rotation"]["w"]
    #             transforms["vr_to_robot"] = hand_to_robot
    #     except Exception as e:
    #         self.get_logger().warn(f"Using default calibration: {e}")
    #     return transforms
    
    def _apply_transform(self, point: Point, transform: Transform) -> Point:
        """Apply a transform to a 3D point"""
        t = np.array([point.x, point.y, point.z, 1.0])
        trans = np.array([transform.translation.x,
                        transform.translation.y,
                        transform.translation.z])
        rot = [transform.rotation.x, transform.rotation.y,
            transform.rotation.z, transform.rotation.w]
        mat = tf_transformations.quaternion_matrix(rot)
        mat[0:3, 3] = trans
        result = mat.dot(t)
        return Point(x=result[0], y=result[1], z=result[2])

    def check_plugins(self):
        """Verify that required methods are implemented"""
        required_methods = [
            'retarget_control_to_robot_space',
            'hand_keypoint_callback',
            'register_pub_sub'
        ]
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.handle_error(f"Required method '{method_name}' not implemented")
                return False
        
        self.get_logger().info("All required plugins/methods are available")
        return True

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for MotionRetargetOT")
        self.is_retargeting_active = False
        
        # Stop the dummy retargeting timer
        if hasattr(self, 'dummy_retargeting_timer'):
            self.destroy_timer(self.dummy_retargeting_timer)
        
        # Send stop command to robot (all zeros)
        stop_msg = JointAngles()
        stop_msg.header = Header()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.robot_id = "xarm5_robot"
        stop_msg.joint_names = self.robot_joint_names
        stop_msg.joint_angles = [0.0] * len(self.robot_joint_names)
        stop_msg.timeout = Duration(sec=0, nanosec=0)
        
        self.publish_cmd("joint_angles_cmd", stop_msg)

        self.set_state("disconnected")
        
        self.get_logger().info("MotionRetargetOT emergency stopped successfully")

    def disable_transform(self):
        """Disable motion retargeting"""
        self.is_retargeting_active = False
        self.get_logger().info("Motion retargeting disabled")

    def enable_transform(self):
        """Enable motion retargeting"""
        self.is_retargeting_active = True
        self.get_logger().info("Motion retargeting enabled")
