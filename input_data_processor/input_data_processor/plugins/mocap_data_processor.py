from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional
import rclpy
import yaml
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

# Import message types based on the interface definitions
from component_interfaces.msg import (
    RawHandKeypoint,
    ProcessedHandKeypoint
)
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time

class RococoHandKeypointProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific processor
        self.processing_frequency = 90  # Hz
        self.is_processing_active = True
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
        
        
        # Initialize processing parameters
        self._initialize_node()

    def _initialize_node(self):
        """Override the base initialization for Rococo specific setup"""
        self.get_logger().info(f"Initializing RococoHandKeypointProcessor: {self.layer_cn_id}")

        self._load_processor_config()
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        ## Start a timer for periodic dummy processing
        # self.dummy_processing_timer = self.create_timer(1.0/self.processing_frequency, self.dummy_processing_callback)
        
        self.lifecycle = "ready"
        self.get_logger().info("RococoHandKeypointProcessor initialized successfully")

    def _load_processor_config(self):
        """Load processor-specific configuration"""
        try:
            config_data = self.get_config("input_control_data_processor.yaml")
            if config_data:
                config = yaml.safe_load(config_data) if isinstance(config_data, str) else config_data
                if "plugins" in config and len(config["plugins"]) > 0:
                    plugin_params = config["plugins"][0].get("params", {})
                    self.processing_frequency = plugin_params.get("frequency", self.processing_frequency)
                    self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
        except Exception as e:
            self.get_logger().error(f"Failed to load processor config: {str(e)}")

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream.yaml configuration"""
        try:
            # Get data stream configuration
            config_data = self.get_config("data_stream.yaml")
            if config_data:
                data_config = yaml.safe_load(config_data) if isinstance(config_data, str) else data_config
                
                # Find topics where this node is a subscriber
                if "topics" in data_config:
                    for topic_config in data_config["topics"]:
                        topic_id = topic_config["id"]
                        
                        # Subscribe to topics where this node is a subscriber
                        if self.layer_cn_id in topic_config.get("subscribers", []):
                            msg_type = topic_config["msg_type"]
                            
                            if msg_type == "component_interfaces/msg/RawHandKeypoint":
                                self.raw_data_subscribers[topic_id] = self.create_subscription(
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
                                self.processed_data_publishers[topic_id] = self.create_publisher(
                                    ProcessedHandKeypoint,
                                    topic_id,
                                    10
                                )
                                self.get_logger().info(f"Created publisher for processed data: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to register publishers/subscribers: {str(e)}")

    def raw_data_callback(self, msg: RawHandKeypoint):
        """Callback for raw hand keypoint data"""
        self.get_logger().debug(f"Received raw hand keypoint data with {len(msg.fingertips)} fingertips")
        
        # Process the data immediately when received
        if self.is_processing_active:
            self.process_data(msg)

    def process_data(self, raw_data: RawHandKeypoint):
        """
        Process raw hand keypoint data into processed format
        """
        try:
            self.processing_counter += 1
            self.get_logger().debug(f"Processing hand keypoint data #{self.processing_counter}")
            
            # Process the raw data
            processed_msg = self.process_right_hand(raw_data)
            
            # Publish processed data
            self.publish_processed_data("right_hand_keypoint_processed", processed_msg)
            
            self.get_logger().debug("Data processing completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error processing data: {str(e)}")

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
    
    def create_wrist_pose(self, wrist_position, wrist_orientation):
        """Create a Pose message for the wrist with both position and orientation"""
        pose = Pose()
        pose.position.x = float(wrist_position[0])
        pose.position.y = float(wrist_position[1])
        pose.position.z = float(wrist_position[2])
        
        pose.orientation.x = float(wrist_orientation[0])
        pose.orientation.y = float(wrist_orientation[1])
        pose.orientation.z = float(wrist_orientation[2])
        pose.orientation.w = float(wrist_orientation[3])
        
        return pose
    
    def process_left_hand(self, raw_data: RawHandKeypoint):
        """Process left hand raw keypoints"""
        try:
            # Convert poses to numpy arrays
            hand_positions, hand_orientations = self.poses_to_numpy_arrays(raw_data.fingertips)
            
            # Normalize positions (orientations remain unchanged)
            normalized_positions, orientations = self.normalize_poses_wrt_middle_proximal(
                hand_positions, hand_orientations, is_left=True)
            
            # Extract finger poses
            thumb_poses, index_poses, middle_poses, ring_poses, pinky_poses = self.extract_finger_poses(
                normalized_positions, orientations, is_left=True)
            
            # Create processed message
            processed_msg = ProcessedHandKeypoint()
            processed_msg.header = raw_data.header
            processed_msg.header.stamp = rospy.Time.now()
            
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
            self.logging(f"Error processing right hand data: {e}", "ERROR")
    
    def process_right_hand(self, raw_data: RawHandKeypoint):
        """Process right hand raw keypoints"""
        try:
            # Convert fingertips to numpy array
            hand_positions = self.points_to_numpy_array(raw_data.fingertips)
            
            # Normalize positions
            normalized_positions = self.normalize_wrt_middle_proximal(hand_positions, is_left=False)
            
            # Extract finger joints
            thumb_joints, index_joints, middle_joints, ring_joints, pinky_joints = self.extract_finger_joints(
                normalized_positions, is_left=False)
            
            # Create processed message
            processed_msg = ProcessedHandKeypoint()
            processed_msg.header = raw_data.header
            processed_msg.header.stamp = rospy.Time.now()
            
            # Convert numpy arrays back to Point messages
            processed_msg.thumb_joints = self.numpy_array_to_points(thumb_joints)
            processed_msg.index_joints = self.numpy_array_to_points(index_joints)
            processed_msg.middle_joints = self.numpy_array_to_points(middle_joints)
            processed_msg.ring_joints = self.numpy_array_to_points(ring_joints)
            processed_msg.pinky_joints = self.numpy_array_to_points(pinky_joints)
            
            # Set wrist pose
            processed_msg.wrist = self.create_wrist_pose(normalized_positions[0])
            
            # Set joint names
            processed_msg.joint_names = self.right_hand_joint_names
            
            return processed_msg
                        
        except Exception as e:
            self.logging(f"Error processing right hand data: {e}", "ERROR")


    def check_plugins(self):
        """Verify that required methods are implemented"""
        required_methods = ['process_data', 'raw_data_callback', 'register_pub_sub']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().error(f"Required method '{method_name}' not implemented")
                return False
        
        self.get_logger().info("All required plugins/methods are available")
        return True

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RococoHandKeypointProcessor")
        self.is_processing_active = False
        
        # Stop the dummy processing timer
        if hasattr(self, 'dummy_processing_timer'):
            self.destroy_timer(self.dummy_processing_timer)
        
        self.get_logger().info("RococoHandKeypointProcessor emergency stopped successfully")

    def disable_processing(self):
        """Disable data processing"""
        self.is_processing_active = False
        self.get_logger().info("Data processing disabled")

    def enable_processing(self):
        """Enable data processing"""
        self.is_processing_active = True
        self.get_logger().info("Data processing enabled")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    processor = RococoHandKeypointProcessor(layer_cn_id="input_control_data_processor")
    
    try:
        # Spin the node
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info("Keyboard interrupt received")
    finally:
        # Cleanup
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()