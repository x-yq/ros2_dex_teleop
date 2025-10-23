from core_bases.motion_retarget_base import MotionRetargetBase
from typing import Dict, List, Any, Optional
import rclpy
import yaml
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import tf2_ros
import tf2_geometry_msgs
import math

# Import message types based on the interface definitions
from component_interfaces.msg import (
    ProcessedHandKeypoint,
    ControlCmd
)
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Transform, TransformStamped
from builtin_interfaces.msg import Time, Duration

class MotionRetargetOT(MotionRetargetBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific retargeter
        self.processing_frequency = 90  # Hz
        self.retargeting_counter = 0
        
        # Robot-specific parameters
        self.robot_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint_eef"]
        self.robot_joint_limits = [
            (-3.14, 3.14), (-2.0, 2.0), (-3.14, 3.14), 
            (-3.14, 3.14), (-3.14, 3.14)
        ]
        
        # Initialize retargeting parameters
        self._load_my_config()


    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'motion_retarget_ot':
                        plugin_params = plugin.get('params', {})
                        self.processing_frequency = plugin_params.get("freq", self.processing_frequency)
                        self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
                        break
                
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
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            ProcessedHandKeypoint,
                            topic_id,
                            self.hand_keypoint_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to hand keypoints: {topic_id}")
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ControlCmd":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            ControlCmd,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for joint angles: {topic_id}")
        
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: {str(e)}")
            
    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""

        self.get_logger().debug(f"Received processed hand keypoints with {len(msg.joint_names)} joints")

        self.pong_reply("right_hand_keypoint_processed", msg)
        
        # Retarget the hand data to robot commands
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
            self.publish_msg("joint_angles_cmd", joint_angles_msg)
            
            self.get_logger().debug("Hand data retargeting completed successfully")
            
        except Exception as e:
            self.handle_error(f"Error retargeting hand data: {str(e)}")
    
    def quaternion_to_euler(self, quat):
        """Convert quaternion to Euler angles using tf2-compatible method"""
        self.get_logger().debug(f"quaternion_to_euler received: {quat}, type: {type(quat)}")
        
        try:
            # Extract quaternion components
            x, y, z, w = quat.x, quat.y, quat.z, quat.w
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            t0 = 2.0 * (w * x + y * z)
            t1 = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(t0, t1)
            
            t2 = 2.0 * (w * y - z * x)
            t2 = 1.0 if t2 > 1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = math.asin(t2)
            
            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(t3, t4)
            
            return roll, pitch, yaw
            
        except Exception as e:
            self.handle_error(f"quaternion_to_euler failed: {e}")
            raise

    def retarget_control_to_robot_space(
        self, control_data: ProcessedHandKeypoint, transforms: Dict[str, Any]
    ) -> ControlCmd:
        """
        Simple dummy implementation that returns basic joint angles.
        """
        joint_angles_msg = ControlCmd()
        joint_angles_msg.header = control_data.header
        joint_angles_msg.header.stamp = self.get_clock().now().to_msg()
        joint_angles_msg.robot_id = "xarm5_robot"
        joint_angles_msg.joint_names = self.robot_joint_names
        joint_angles_msg.timeout = Duration(sec=1, nanosec=0)

        # Simple dummy joint angles (in radians)
        if len(self.robot_joint_names) == 5:
            joint_angles_msg.joint_angles = [0.1, -0.2, 0.3, 0.0, 0.1]
        elif len(self.robot_joint_names) == 6:
            joint_angles_msg.joint_angles = [0.1, -0.2, 0.3, 0.0, 0.1, -0.05]
        else:
            joint_angles_msg.joint_angles = [0.0] * len(self.robot_joint_names)
        
        joint_angles_msg.joint_velocities = [0.5] * len(joint_angles_msg.joint_angles)
        
        return joint_angles_msg

    def _apply_joint_limit(self, angle: float, joint_index: int) -> float:
        """Apply joint limits to a given angle"""
        lower, upper = self.robot_joint_limits[joint_index]
        return np.clip(angle, lower, upper)

    def _get_required_transforms(self) -> Dict[str, Any]:
        """Get required transforms for retargeting (dummy implementation)"""
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

    def _apply_transform(self, point: Point, transform: Transform) -> Point:
        """Apply a transform to a 3D point using tf2 method"""
        try:
            # Create a PointStamped for tf2 transformation
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.frame_id = "source_frame"
            point_stamped.point = point
            
            # Create a TransformStamped
            transform_stamped = TransformStamped()
            transform_stamped.header.frame_id = "source_frame"
            transform_stamped.child_frame_id = "target_frame"
            transform_stamped.transform = transform
            
            # Apply the transform using tf2
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform_stamped)
            
            return transformed_point.point
            
        except Exception as e:
            self.handle_error(f"Failed to apply transform: {e}")
            # Fallback to manual transformation
            return self._apply_transform_manual(point, transform)

    def _apply_transform_manual(self, point: Point, transform: Transform) -> Point:
        """Manual transformation implementation as fallback"""
        # Extract translation
        tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
        
        # Extract rotation quaternion
        qx, qy, qz, qw = transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
        
        # Convert quaternion to rotation matrix
        # Rotation matrix from quaternion
        xx = qx * qx
        xy = qx * qy
        xz = qx * qz
        xw = qx * qw
        yy = qy * qy
        yz = qy * qz
        yw = qy * qw
        zz = qz * qz
        zw = qz * qw
        
        rot_matrix = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
            [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
            [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)]
        ])
        
        # Apply rotation and translation
        original_vec = np.array([point.x, point.y, point.z])
        rotated_vec = np.dot(rot_matrix, original_vec)
        translated_vec = rotated_vec + np.array([tx, ty, tz])
        
        return Point(x=translated_vec[0], y=translated_vec[1], z=translated_vec[2])

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for MotionRetargetOT")
        
        # Send stop command to robot (all zeros)
        stop_msg = ControlCmd()
        stop_msg.header = Header()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.robot_id = "xarm5_robot"
        stop_msg.joint_names = self.robot_joint_names
        stop_msg.joint_angles = [0.0] * len(self.robot_joint_names)
        stop_msg.timeout = Duration(sec=0, nanosec=0)
        
        self.publish_msg("joint_angles_cmd", stop_msg)
        self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("MotionRetargetOT emergency stopped successfully")