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
from copy import copy

class VRHandKeypointProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        self._load_my_config()
        if self.simulate_latency > 0:
            self._setup_latency_simulation()

        OCULUS_JOINTS = {
            'metacarpals': [2, 6, 9, 12, 15],
            'knuckles': [6, 9, 12, 16],
            'thumb': [2, 3, 4, 5, 19],
            'index': [6, 7, 8, 20],
            'middle': [9, 10, 11, 21],
            'ring': [12, 13, 14, 22],
            'pinky': [15, 16, 17, 18, 23]
        }

        self.knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
        # Moving average queue
        self.moving_average_limit = 5
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []

    def _load_my_config(self):
        """Load processor-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'vr_handkeypoint_processor':
                        plugin_params = plugin.get('params', {})
                        self.simulate_latency = plugin_params.get("simulate_latency", self.simulate_latency)
                        self.get_logger().info(f"Loaded processor config")
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

        self.get_logger().debug(f"Received raw hand keypoint data with {len(msg.keypoints)} fingertips")
        
        self.pong_reply("right_hand_keypoint_raw", msg)

        if msg.mode == 0:
            data_type = 'absolute'
        else:
            data_type = 'relative'
        
        if len(msg.keypoints) != 72:
            self.get_logger().error(
                f"Invalid keypoints length: expected 72 floats, got {len(msg.keypoints)}"
            )
            return
         
        transformed_hand_coords, translated_hand_coord_frame = self.transform_keypoints(np.asanyarray(msg.keypoints).reshape(24, 3))
        
        # Passing the transformed coords into a moving average
        self.averaged_hand_coords = self._moving_average(
            transformed_hand_coords, 
            self.coord_moving_average_queue, 
            self.moving_average_limit
        )

        self.averaged_hand_frame = self._moving_average(
            translated_hand_coord_frame, 
            self.frame_moving_average_queue, 
            self.moving_average_limit
        )

        processed_msg = ProcessedHandKeypoint()
        processed_msg.header = msg.header
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.data_type = data_type
        processed_msg.hand_coords = self.averaged_hand_coords.flatten().tolist()
        processed_msg.hand_frame = np.array(self.averaged_hand_frame).flatten().tolist()

        self.publish_msg("right_hand_keypoint_processed", processed_msg)

    def _normalize_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def _moving_average(self, vector, moving_average_queue, limit):
        moving_average_queue.append(vector)

        if len(moving_average_queue) > limit:
            moving_average_queue.pop(0)

        mean_vector = np.mean(moving_average_queue, axis = 0)
        return mean_vector
    
    # Function to find hand coordinates with respect to the wrist
    def _translate_coords(self, hand_coords):
        return copy(hand_coords) - hand_coords[0]

    # Create a coordinate frame for the hand
    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord):
        palm_normal = self._normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Current Z
        palm_direction = self._normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Current Y
        cross_product = self._normalize_vector(np.cross(palm_direction, palm_normal))              # Current X
        return [cross_product, palm_direction, palm_normal]

    # Create a coordinate frame for the arm 
    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord):

        palm_normal = self._normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Unity space - Y
        palm_direction = self._normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = self._normalize_vector(index_knuckle_coord - pinky_knuckle_coord)              # Unity space - X
        
        return [origin_coord, cross_product, palm_normal, palm_direction]

    def transform_keypoints(self, hand_coords):
        translated_coords = self._translate_coords(hand_coords)
        original_coord_frame = self._get_coord_frame(
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        # Finding the rotation matrix and rotating the coordinates
        rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        
        hand_dir_frame = self._get_hand_dir_frame(
            hand_coords[0],
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        return transformed_hand_coords, hand_dir_frame

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for VRHandKeypointProcessor")

        # self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("VRHandKeypointProcessor emergency stopped successfully")
