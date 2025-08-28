from typing import Dict, List, Any, Optional, Union
import json
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
        
        # Store latest received data
        self.latest_scene_image = None
        self.latest_robot_state = None
        self.latest_hand_plot_data = None
        
        # Dummy projection parameters
        self.projection_interval = 1.0  # seconds
        self.last_projection_time = self.get_clock().now()

        self._load_my_config()

    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if not self.my_config:
                self.get_logger().warn(f"No component config found for {self.layer_cn_id}")
                self.my_config = {}
            
            # Override display parameters from component config if available
            if self.my_config and "plugins" in self.my_config:
                for plugin in self.my_config["plugins"]:
                    if plugin.get("id") == "render_pc_driver":
                        plugin_params = plugin.get("params", {})
                        self.width = plugin_params.get("width", self.width)
                        self.height = plugin_params.get("height", self.height)
                        self.projection_interval = plugin_params.get("projection_interval", self.projection_interval)
                        self.get_logger().info(f"Loaded display config: {self.width}x{self.height}, interval: {self.projection_interval}s")
                        break
            self.get_logger().info("All configurations loaded successfully")
            
        except Exception as e:
            self.handle_error("Failed to load configurations", e)

    def register_pub_sub(self):
        """Register subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of manually parsing YAML
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return
                
            # Find topics where this node is a subscriber
            for topic_config in self.data_stream_config["topics"]:
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
            self.handle_error("Failed to register subscriptions", e)

    def scene_image_callback(self, msg: SceneImage):
        """Callback for processed scene image data"""
        ##NOTE
        # if self.state!='running':
        #     return        
        self.latest_scene_image = msg
        self.get_logger().debug(f"Received scene image from {msg.image_source}")

    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state data"""
        ##NOTE
        # if self.state!='running':
        #     return        
        self.latest_robot_state = msg
        self.get_logger().debug(f"Received robot state for {msg.robot_id}")

    def hand_plot_callback(self, msg: Hand2DPlotData):
        """Callback for hand 2D plot data"""
        ##NOTE
        # if self.state!='running':
        #     return        
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
        
        ##NOTE
        # if self.state != "running":
        #     return 
            
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

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RenderPCDriver")
        
        # Clear all data
        self.latest_scene_image = None
        self.latest_robot_state = None
        self.latest_hand_plot_data = None
        
        self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("RenderPCDriver emergency stopped successfully")