from typing import Dict, List, Any, Optional, Union
import json
import yaml
import rclpy
from rclpy.subscription import Subscription
import numpy as np

from core_bases.feedback_projector_driver_base import FeedbackProjectorDriverBase

# Import message types based on the interface definitions
from component_interfaces.msg import (
    SceneImage,
    RobotState, 
    Hand2DPlotData
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

class RenderPCDriver(FeedbackProjectorDriverBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific driver
        self.width = 1280
        self.height = 720
        self.is_projection_active = False
        
        # Store latest received data
        self.latest_scene_image = None
        self.latest_robot_state = None
        self.latest_hand_plot_data = None
        
        # Dummy projection parameters
        self.projection_interval = 1.0  # seconds
        self.last_projection_time = self.get_clock().now()
        self._initialize_node()

    def _initialize_node(self):
        """Override the base initialization for RenderPC specific setup"""
        self.get_logger().info(f"Initializing RenderPCDriver: {self.layer_cn_id}")
        
        # Load specific configuration from YAML
        config_data = self.get_config("feedback_projector_driver.yaml")
        if config_data:
            try:
                config = yaml.safe_load(config_data) if isinstance(config_data, str) else config_data
                if "plugins" in config and len(config["plugins"]) > 0:
                    plugin_params = config["plugins"][0].get("params", {})
                    self.width = plugin_params.get("width", self.width)
                    self.height = plugin_params.get("height", self.height)
                    self.get_logger().info(f"Loaded display config: {self.width}x{self.height}")
            except Exception as e:
                self.get_logger().error(f"Failed to parse plugin config: {str(e)}")
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Start a timer for periodic dummy projection
        self.projection_timer = self.create_timer(0.1, self.periodic_projection)
        
        self.lifecycle = "ready"
        self.get_logger().info("RenderPCDriver initialized successfully")

    def register_pub_sub(self):
        """Register subscribers based on data_stream.yaml configuration"""
        try:
            # Get data stream configuration
            config_data = self.get_config("data_stream.yaml")
            if config_data:
                data_config = yaml.safe_load(config_data) if isinstance(config_data, str) else config_data
                
                # Find topics where this node is a subscriber
                if "topics" in data_config:
                    for topic_config in data_config["topics"]:
                        if self.layer_cn_id in topic_config.get("subscribers", []):
                            topic_id = topic_config["id"]
                            msg_type = topic_config["msg_type"]
                            
                            # Create subscription based on message type
                            if msg_type == "component_interfaces/msg/SceneImage":
                                self.create_subscription(
                                    SceneImage,
                                    topic_id,
                                    self.scene_image_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to {topic_id}")
                                
                            elif msg_type == "component_interfaces/msg/RobotState":
                                self.create_subscription(
                                    RobotState,
                                    topic_id,
                                    self.robot_state_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to {topic_id}")
                                
                            elif msg_type == "component_interfaces/msg/Hand2DPlotData":
                                self.create_subscription(
                                    Hand2DPlotData,
                                    topic_id,
                                    self.hand_plot_callback,
                                    10
                                )
                                self.get_logger().info(f"Subscribed to {topic_id}")
            
            self.get_logger().info("All subscriptions registered successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to register subscriptions: {str(e)}")

    def scene_image_callback(self, msg: SceneImage):
        """Callback for processed scene image data"""
        self.latest_scene_image = msg
        self.get_logger().debug(f"Received scene image from {msg.image_source}")

    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state data"""
        self.latest_robot_state = msg
        self.get_logger().debug(f"Received robot state for {msg.robot_id}")

    def hand_plot_callback(self, msg: Hand2DPlotData):
        """Callback for hand 2D plot data"""
        self.latest_hand_plot_data = msg
        self.get_logger().debug(f"Received hand plot data for {msg.hand_id}")

    def project_feedback(self, projector_id: str, data: Dict[str, Any]):
        """
        Dummy implementation of projection method
        """
        self.get_logger().info(f"Projecting data on {projector_id}")
        self.get_logger().info(f"Data keys: {list(data.keys())}")
        
        # Simulate some projection work
        if "image" in data:
            self.get_logger().info(f"Projecting image with shape {data['image'].shape if hasattr(data['image'], 'shape') else 'unknown'}")
        
        if "robot_state" in data:
            self.get_logger().info(f"Projecting robot state: {data['robot_state'].get('robot_id', 'unknown')}")
        
        if "hand_plot" in data:
            self.get_logger().info(f"Projecting hand plot with {len(data['hand_plot'].get('joints', []))} joints")
        
        # Simulate projection success
        return True

    def periodic_projection(self):
        """Periodically project dummy data for testing"""
        current_time = self.get_clock().now()
        if (current_time - self.last_projection_time).nanoseconds / 1e9 >= self.projection_interval:
            
            # Create dummy projection data
            projection_data = {
                "timestamp": current_time.to_msg(),
                "projector_id": "render_pc_display",
                "content": self.generate_dummy_projection_content()
            }
            
            # Project the data
            success = self.project_feedback("render_pc", projection_data)
            
            if success:
                self.get_logger().debug("Dummy projection completed successfully")
            
            self.last_projection_time = current_time

    def generate_dummy_projection_content(self) -> Dict[str, Any]:
        """Generate dummy projection content for testing"""
        # Create a dummy image
        dummy_image = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
        
        # Create dummy robot state
        dummy_robot_state = {
            "robot_id": "xarm5_robot",
            "joint_positions": [0.1, -0.2, 0.3, -0.4, 0.5],
            "end_effector_pose": [0.5, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        }
        
        # Create dummy hand plot data
        dummy_hand_plot = {
            "hand_id": "right_hand",
            "joints": np.random.rand(16, 2).tolist(),
            "joint_names": [f"joint_{i}" for i in range(16)]
        }
        
        return {
            "image": dummy_image,
            "robot_state": dummy_robot_state,
            "hand_plot": dummy_hand_plot,
            "timestamp": self.get_clock().now().to_msg().sec
        }

    def check_plugins(self):
        """Verify that required methods are implemented"""
        required_methods = ['project_feedback', 'register_pub_sub']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().error(f"Required method '{method_name}' not implemented")
                return False
        
        self.get_logger().info("All required plugins/methods are available")
        return True

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RenderPCDriver")
        self.is_projection_active = False
        
        # Stop the projection timer
        if hasattr(self, 'projection_timer'):
            self.destroy_timer(self.projection_timer)
        
        # Clear all data
        self.latest_scene_image = None
        self.latest_robot_state = None
        self.latest_hand_plot_data = None
        
        self.get_logger().info("RenderPCDriver emergency stopped successfully")