from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional
import rclpy
import json
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

# Import message types based on the interface definitions
from component_interfaces.msg import (
    RawHandKeypoint,
    ProcessedHandKeypoint
)
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from builtin_interfaces.msg import Time

class RococoHandKeypointProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific processor
        self.processing_frequency = 90  # Hz
        self.processing_counter = 0

        self.left_hand_joint_names = ["leftHand",
                                     'leftThumbProximal', 'leftThumbMedial', 'leftThumbDistal', 'leftThumbTip',
                                     'leftIndexProximal', 'leftIndexMedial', 'leftIndexDistal', 'leftIndexTip',
                                     'leftMiddleProximal', 'leftMiddleMedial', 'leftMiddleDistal', 'leftMiddleTip',
                                     'leftRingProximal', 'leftRingMedial', 'leftRingDistal', 'leftRingTip',
                                     'leftLittleProximal', 'leftLittleMedial', 'leftLittleDistal', 'leftLittleTip']
        
        self.right_hand_joint_names = ["rightHand",
                                      'rightThumbProximal', 'rightThumbMedial', 'rightThumbDistal', 'rightThumbTip',
                                      'rightIndexProximal', 'rightIndexMedial', 'rightIndexDistal', 'rightIndexTip',
                                      'rightMiddleProximal', 'rightMiddleMedial', 'rightMiddleDistal', 'rightMiddleTip',
                                      'rightRingProximal', 'rightRingMedial', 'rightRingDistal', 'rightRingTip',
                                      'rightLittleProximal', 'rightLittleMedial', 'rightLittleDistal', 'rightLittleTip']
        
        self._load_my_config()


    def _load_my_config(self):
        """Load processor-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                # Extract processing frequency from plugin parameters if available
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'rococo_handkeypoint_processor' or plugin.get('_target_', '').endswith('RococoHandKeypointProcessor'):
                        plugin_params = plugin.get('params', {})
                        self.processing_frequency = plugin_params.get("frequency", self.processing_frequency)
                        self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load processor config: ", e)

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config from _load_processor_config()
            if not self.data_stream_config:
                self.handle_error("Data stream configuration not loaded")
                return
                
            # Find topics where this node is a subscriber or publisher
            topics = self.data_stream_config.get("topics", [])
            if not topics:
                self.get_logger().warn("No topics found in data stream configuration")
                return
                
            for topic_config in topics:
                topic_id = topic_config["id"]
                
                # Subscribe to topics where this node is a subscriber
                if self.layer_cn_id in topic_config.get("subscribers", []):
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/RawHandKeypoint":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            RawHandKeypoint,
                            topic_id,
                            self.raw_data_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to raw data: {topic_id}")
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            ProcessedHandKeypoint,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for processed data: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: ", e)

    def raw_data_callback(self, msg: RawHandKeypoint):
        """Callback for raw hand keypoint data"""

        self.get_logger().debug(f"Received raw hand keypoint data with {len(msg.fingertips)} fingertips")
        
        self.pong_reply("right_hand_keypoint_raw", msg)

        self.process_right_hand_data(msg)

    def process_right_hand_data(self, raw_data: RawHandKeypoint):
        """
        Process raw hand keypoint data into processed format
        """
        try:
            self.processing_counter += 1
            self.get_logger().debug(f"Processing hand keypoint data #{self.processing_counter}")
            
            # Process the raw data
            processed_msg = self.process_hand(raw_data, is_left=False)
            
            # Publish processed data
            self.publish_msg("right_hand_keypoint_processed", processed_msg)
            
            self.get_logger().debug("Data processing completed successfully")
            
        except Exception as e:
            self.handle_error("Error processing data:", e)

    def normalize_poses_wrt_middle_proximal(self, hand_positions, hand_orientations, is_left=True):
        """
        Normalize hand poses with respect to middle proximal joint
        Returns normalized positions and original orientations
        """
        if is_left:
            middle_proximal_idx = self.left_hand_joint_names.index('leftMiddleProximal')
        else:
            middle_proximal_idx = self.right_hand_joint_names.index('rightMiddleProximal')
        
        wrist_position = hand_positions[0]
        middle_proximal_position = hand_positions[middle_proximal_idx]
        bone_length = np.linalg.norm(wrist_position - middle_proximal_position)
        
        if bone_length > 0:
            normalized_hand_positions = (middle_proximal_position - hand_positions) / bone_length
        else:
            normalized_hand_positions = hand_positions  # Fallback if bone_length is 0
        
        # Return normalized positions and original orientations
        return normalized_hand_positions, hand_orientations
    
    def poses_to_numpy_arrays(self, poses):
        """Convert list of geometry_msgs/Pose to numpy arrays for positions and orientations"""
        positions = np.zeros((len(poses), 3))
        orientations = np.zeros((len(poses), 4))  # quaternion [x, y, z, w]
        
        for i, pose in enumerate(poses):
            positions[i] = [pose.position.x, pose.position.y, pose.position.z]
            orientations[i] = [pose.orientation.x, pose.orientation.y, 
                              pose.orientation.z, pose.orientation.w]
        
        return positions, orientations
    
    def numpy_arrays_to_poses(self, positions, orientations):
        """Convert numpy arrays to list of geometry_msgs/Pose"""
        poses = []
        for i in range(len(positions)):
            pose = Pose()
            pose.position.x = float(positions[i][0])
            pose.position.y = float(positions[i][1])
            pose.position.z = float(positions[i][2])
            
            pose.orientation.x = float(orientations[i][0])
            pose.orientation.y = float(orientations[i][1])
            pose.orientation.z = float(orientations[i][2])
            pose.orientation.w = float(orientations[i][3])
            
            poses.append(pose)
        return poses
    
    def extract_finger_poses(self, hand_positions, hand_orientations, is_left=True):
        """Extract poses for each finger from hand positions and orientations"""
        if is_left:
            joint_names = self.left_hand_joint_names
            prefix = "left"
        else:
            joint_names = self.right_hand_joint_names
            prefix = "right"
        
        # Extract finger poses (skip wrist at index 0)
        thumb_poses = self.create_poses_from_arrays(hand_positions[1:5], hand_orientations[1:5])
        index_poses = self.create_poses_from_arrays(hand_positions[5:9], hand_orientations[5:9])
        middle_poses = self.create_poses_from_arrays(hand_positions[9:13], hand_orientations[9:13])
        ring_poses = self.create_poses_from_arrays(hand_positions[13:17], hand_orientations[13:17])
        pinky_poses = self.create_poses_from_arrays(hand_positions[17:21], hand_orientations[17:21])
        
        return thumb_poses, index_poses, middle_poses, ring_poses, pinky_poses
    
    def create_poses_from_arrays(self, positions, orientations):
        """Create Pose messages from position and orientation arrays"""
        poses = []
        for i in range(len(positions)):
            pose = Pose()
            pose.position.x = float(positions[i][0])
            pose.position.y = float(positions[i][1])
            pose.position.z = float(positions[i][2])
            
            pose.orientation.x = float(orientations[i][0])
            pose.orientation.y = float(orientations[i][1])
            pose.orientation.z = float(orientations[i][2])
            pose.orientation.w = float(orientations[i][3])
            
            poses.append(pose)
        return poses
    
    def create_wrist_pose(self, wrist_position, wrist_orientation=None):
        """Create a Pose message for the wrist with both position and orientation"""
        pose = Pose()
        pose.position.x = float(wrist_position[0])
        pose.position.y = float(wrist_position[1])
        pose.position.z = float(wrist_position[2])
        
        if wrist_orientation is not None:
            pose.orientation.x = float(wrist_orientation[0])
            pose.orientation.y = float(wrist_orientation[1])
            pose.orientation.z = float(wrist_orientation[2])
            pose.orientation.w = float(wrist_orientation[3])
        else:
            # Default orientation (no rotation)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
        
        return pose
    
    def points_to_numpy_array(self, points):
        """Convert list of geometry_msgs/Point to numpy array"""
        positions = np.zeros((len(points), 3))
        for i, point in enumerate(points):
            positions[i] = [point.x, point.y, point.z]
        return positions
    
    def numpy_array_to_points(self, positions):
        """Convert numpy array to list of geometry_msgs/Point"""
        points = []
        for i in range(len(positions)):
            point = Point()
            point.x = float(positions[i][0])
            point.y = float(positions[i][1])
            point.z = float(positions[i][2])
            points.append(point)
        return points
    
    def normalize_wrt_middle_proximal(self, hand_positions, is_left=True):
        """Normalize hand positions with respect to middle proximal joint (Point version)"""
        if is_left:
            middle_proximal_idx = self.left_hand_joint_names.index('leftMiddleProximal')
        else:
            middle_proximal_idx = self.right_hand_joint_names.index('rightMiddleProximal')
        
        if len(hand_positions) <= middle_proximal_idx:
            self.get_logger().warn(f"Not enough hand positions for middle proximal joint at index {middle_proximal_idx}")
            return hand_positions
        
        wrist_position = hand_positions[0]
        middle_proximal_position = hand_positions[middle_proximal_idx]
        bone_length = np.linalg.norm(wrist_position - middle_proximal_position)
        
        if bone_length > 0:
            normalized_positions = (middle_proximal_position - hand_positions) / bone_length
        else:
            normalized_positions = hand_positions  # Fallback if bone_length is 0
        
        return normalized_positions
    
    def extract_finger_joints(self, hand_positions, is_left=True):
        """Extract joint positions for each finger from hand positions (Point version)"""
        if is_left:
            joint_names = self.left_hand_joint_names
        else:
            joint_names = self.right_hand_joint_names
        
        # Ensure we have enough positions
        if len(hand_positions) < len(joint_names):
            self.get_logger().warn(f"Not enough hand positions: {len(hand_positions)} < {len(joint_names)}")
            # Pad with zeros if needed
            padded_positions = np.zeros((len(joint_names), 3))
            padded_positions[:len(hand_positions)] = hand_positions
            hand_positions = padded_positions
        
        # Extract finger joints (skip wrist at index 0)
        thumb_joints = hand_positions[1:5]    # indices 1-4
        index_joints = hand_positions[5:9]    # indices 5-8
        middle_joints = hand_positions[9:13]  # indices 9-12
        ring_joints = hand_positions[13:17]   # indices 13-16
        pinky_joints = hand_positions[17:21]  # indices 17-20
        
        return thumb_joints, index_joints, middle_joints, ring_joints, pinky_joints
    
    def process_hand(self, raw_data: RawHandKeypoint, is_left=False):
        """Process hand raw keypoints"""
        try:
            # Convert poses to numpy arrays
            hand_positions, hand_orientations = self.poses_to_numpy_arrays(raw_data.fingertips)
            
            # Normalize positions (orientations remain unchanged)
            normalized_positions, orientations = self.normalize_poses_wrt_middle_proximal(
                hand_positions, hand_orientations, is_left=is_left)
            
            # Extract finger poses
            thumb_poses, index_poses, middle_poses, ring_poses, pinky_poses = self.extract_finger_poses(
                normalized_positions, orientations, is_left=is_left)
            
            # Create processed message
            processed_msg = ProcessedHandKeypoint()
            processed_msg.header = raw_data.header
            processed_msg.header.stamp = self.get_clock().now().to_msg()
            
            # Set finger poses
            processed_msg.thumb_joints = thumb_poses
            processed_msg.index_joints = index_poses
            processed_msg.middle_joints = middle_poses
            processed_msg.ring_joints = ring_poses
            processed_msg.pinky_joints = pinky_poses
            
            # Set wrist pose
            processed_msg.wrist = self.create_wrist_pose(normalized_positions[0], orientations[0])
            
            # Set joint names
            processed_msg.joint_names = self.left_hand_joint_names

            return processed_msg
                                    
        except Exception as e:
            self.handle_error("Error processing hand data: ", e)

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RococoHandKeypointProcessor")

        self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("RococoHandKeypointProcessor emergency stopped successfully")
