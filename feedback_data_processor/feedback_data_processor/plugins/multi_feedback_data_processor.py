from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional
import rclpy
import yaml
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

# Import message types based on the interface definitions
from component_interfaces.msg import (
    SceneImage,
    RobotState,
    Hand2DPlotData,
    ProcessedHandKeypoint
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class MultiFeedbackDataProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific processor
        self.processing_types = ["hand_2d_plot", "scene_img", "robot_state_plot"]
        self.is_processing_active = True
        self.processing_counter = 0
        
        # Store latest received data
        self.latest_hand_keypoints = None
        self.latest_scene_image = None
        self.latest_robot_state = None
        
        # Initialize processing parameters
        self._load_processor_config()

        self._initialize_node()

    def _load_processor_config(self):
        """Load processor-specific configuration"""
        try:
            config_data = self.get_config("feedback_data_processor.yaml")
            if config_data:
                config = yaml.safe_load(config_data) if isinstance(config_data, str) else config_data
                if "plugins" in config and len(config["plugins"]) > 0:
                    plugin_params = config["plugins"][0].get("params", {})
                    self.processing_types = plugin_params.get("types", self.processing_types)
                    self.get_logger().info(f"Loaded processor config: {self.processing_types}")
        except Exception as e:
            self.get_logger().error(f"Failed to load processor config: {str(e)}")

    def _initialize_node(self):
        """Override the base initialization for MultiFeedback specific setup"""
        self.get_logger().info(f"Initializing MultiFeedbackDataProcessor: {self.layer_cn_id}")
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # # Start a timer for periodic dummy processing
        # self.dummy_processing_timer = self.create_timer(0.1, self.dummy_processing_callback)
        
        self.lifecycle = "ready"
        self.get_logger().info("MultiFeedbackDataProcessor initialized successfully")

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
                            
                            if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
                                self.raw_data_subscribers[topic_id] = self.create_subscription(
                                    ProcessedHandKeypoint,
                                    topic_id,
                                    self.hand_keypoint_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to hand keypoints: {topic_id}")
                            
                            elif msg_type == "sensor_msgs/msg/Image":
                                self.raw_data_subscribers[topic_id] = self.create_subscription(
                                    Image,
                                    topic_id,
                                    self.scene_image_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to scene image: {topic_id}")
                            
                            elif msg_type == "component_interfaces/msg/RobotState":
                                self.raw_data_subscribers[topic_id] = self.create_subscription(
                                    RobotState,
                                    topic_id,
                                    self.robot_state_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to robot state: {topic_id}")
                        
                        # Create publishers for topics where this node is the publisher
                        if topic_config.get("publisher") == self.layer_cn_id:
                            msg_type = topic_config["msg_type"]
                            
                            if msg_type == "component_interfaces/msg/SceneImage":
                                self.processed_data_publishers[topic_id] = self.create_publisher(
                                    SceneImage,
                                    topic_id,
                                    10
                                )
                                self.get_logger().info(f"Created publisher for processed scene image: {topic_id}")
                            
                            elif msg_type == "component_interfaces/msg/Hand2DPlotData":
                                self.processed_data_publishers[topic_id] = self.create_publisher(
                                    Hand2DPlotData,
                                    topic_id,
                                    10
                                )
                                self.get_logger().info(f"Created publisher for hand 2D plot: {topic_id}")
                            
                            elif msg_type == "component_interfaces/msg/RobotState":
                                self.processed_data_publishers[topic_id] = self.create_publisher(
                                    RobotState,
                                    topic_id,
                                    10
                                )
                                self.get_logger().info(f"Created publisher for processed robot state: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to register publishers/subscribers: {str(e)}")

    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""
        self.latest_hand_keypoints = msg
        self.get_logger().debug(f"Received processed hand keypoints with {len(msg.joint_names)} joints")
        
        # Process hand data for 2D plotting
        if self.is_processing_active and "hand_2d_plot" in self.processing_types:
            self.process_hand_data(msg)

    def scene_image_callback(self, msg: Image):
        """Callback for raw scene image data"""
        self.latest_scene_image = msg
        self.get_logger().debug("Received raw scene image")
        
        # Process scene image
        if self.is_processing_active and "scene_img" in self.processing_types:
            self.process_scene_image(msg)

    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state data"""
        self.latest_robot_state = msg
        self.get_logger().debug(f"Received robot state for {msg.robot_id}")
        
        # Process robot state for plotting
        if self.is_processing_active and "robot_state_plot" in self.processing_types:
            self.process_robot_state(msg)

    def process_hand_data(self, hand_data: ProcessedHandKeypoint):
        """Process hand keypoints into 2D plot data"""
        try:
            self.processing_counter += 1
            self.get_logger().debug(f"Processing hand data #{self.processing_counter}")
            
            # Create 2D plot data from 3D hand keypoints
            plot_msg = self._generate_2d_plot_data(hand_data)
            
            # Publish processed data
            self.publish_processed_data("handkeypoint_plot_2d", plot_msg)
            
            self.get_logger().debug("Hand data processing completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error processing hand data: {str(e)}")

    def process_scene_image(self, image_data: Image):
        """Process raw scene image"""
        try:
            self.processing_counter += 1
            self.get_logger().debug(f"Processing scene image #{self.processing_counter}")
            
            # Create processed scene image
            processed_image = self._generate_processed_image(image_data)
            
            # Publish processed data
            self.publish_processed_data("scene_image_processed", processed_image)
            
            self.get_logger().debug("Scene image processing completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error processing scene image: {str(e)}")

    def process_robot_state(self, robot_data: RobotState):
        """Process robot state for visualization"""
        try:
            self.processing_counter += 1
            self.get_logger().debug(f"Processing robot state #{self.processing_counter}")
            
            # Create processed robot state
            processed_state = self._generate_processed_robot_state(robot_data)
            
            # Publish processed data
            self.publish_processed_data("robot_state_processed", processed_state)
            
            self.get_logger().debug("Robot state processing completed successfully")
            
        except Exception as e:
            self.get_logger().error(f"Error processing robot state: {str(e)}")

    # def dummy_processing_callback(self):
    #     """Generate and process dummy feedback data"""
    #     if self.is_processing_active:
    #         self.get_logger().debug("Generating and processing dummy feedback data")
            
    #         # Process dummy data for each enabled processing type
    #         if "hand_2d_plot" in self.processing_types:
    #             dummy_hand = self._create_dummy_hand_keypoints()
    #             self.process_hand_data(dummy_hand)
            
    #         if "scene_img" in self.processing_types:
    #             dummy_image = self._create_dummy_scene_image()
    #             self.process_scene_image(dummy_image)
            
    #         if "robot_state_plot" in self.processing_types:
    #             dummy_robot = self._create_dummy_robot_state()
    #             self.process_robot_state(dummy_robot)

    def _generate_2d_plot_data(self, hand_data: ProcessedHandKeypoint) -> Hand2DPlotData:
        """Generate 2D plot data from 3D hand keypoints"""
        plot_msg = Hand2DPlotData()
        plot_msg.header = hand_data.header
        plot_msg.header.stamp = self.get_clock().now().to_msg()
        plot_msg.hand_id = "right_hand"
        
        # Convert 3D points to 2D (simple projection)
        all_joints = []
        all_joints.extend(hand_data.thumb_joints)
        all_joints.extend(hand_data.index_joints)
        all_joints.extend(hand_data.middle_joints)
        all_joints.extend(hand_data.ring_joints)
        all_joints.extend(hand_data.pinky_joints)
        all_joints.append(hand_data.wrist)
        
        # Simple 3D to 2D projection (ignore Z coordinate with some scaling)
        plot_msg.x_coords = [joint.x * 100 + 320 for joint in all_joints]  # Scale and center
        plot_msg.y_coords = [joint.y * 100 + 240 for joint in all_joints]  # Scale and center
        
        plot_msg.joint_names = hand_data.joint_names
        
        return plot_msg

    def _generate_processed_image(self, image_data: Image) -> SceneImage:
        """Generate processed scene image"""
        processed_msg = SceneImage()
        processed_msg.header = image_data.header
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.image_source = "realsense_camera"
        processed_msg.feedback_image = image_data  # In real implementation, would process the image
        
        return processed_msg

    def _generate_processed_robot_state(self, robot_data: RobotState) -> RobotState:
        """Generate processed robot state"""
        processed_msg = RobotState()
        processed_msg.header = robot_data.header
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.robot_id = robot_data.robot_id
        processed_msg.joint_names = robot_data.joint_names
        processed_msg.joint_states = robot_data.joint_states
        processed_msg.end_effector_pose = robot_data.end_effector_pose
        
        # Add some dummy processing (e.g., calculate velocity, etc.)
        return processed_msg

    def check_plugins(self):
        """Verify that required methods are implemented"""
        required_methods = [
            'register_pub_sub'
        ]
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().error(f"Required method '{method_name}' not implemented")
                return False
        
        self.get_logger().info("All required plugins/methods are available")
        return True

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for MultiFeedbackDataProcessor")
        self.is_processing_active = False
        
        # Stop the dummy processing timer
        if hasattr(self, 'dummy_processing_timer'):
            self.destroy_timer(self.dummy_processing_timer)
        
        # Clear data
        self.latest_hand_keypoints = None
        self.latest_scene_image = None
        self.latest_robot_state = None
        
        self.get_logger().info("MultiFeedbackDataProcessor emergency stopped successfully")

    def disable_processing(self):
        """Disable data processing"""
        self.is_processing_active = False
        self.get_logger().info("Data processing disabled")

    def enable_processing(self):
        """Enable data processing"""
        self.is_processing_active = True
        self.get_logger().info("Data processing enabled")