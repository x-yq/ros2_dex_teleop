# core_bases/feedback_projector_driver_base.py

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional, Union
import json
import yaml
import rclpy
from rclpy.subscription import Subscription

from .component_node import ComponentNode

from std_msgs.msg import Header


class FeedbackProjectorDriverBase(ComponentNode, ABC):
    """
    Abstract base class for feedback projector driver nodes.
    Handles subscription to processed feedback data and provides
    projection interface for hardware-specific implementations.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Core attributes
        self.feedback_subscribers: Dict[str, Subscription] = {}
        self.projector_configs: Dict[str, Any] = {}
        

    @overwritten
    def _initialize_node(self):
        """Initialize the feedback projector driver node."""
        self.get_logger().info(f"Initializing FeedbackProjectorDriverBase: {self.layer_cn_id}")
        
        # Load projector configurations
        self._load_projector_configs()
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Verify plugins
        if self.check_plugins():
            self.lifecycle = "ready"
            self.get_logger().info(f"{self.layer_cn_id} initialized successfully")
        else:
            self.get_logger().warn(f"{self.layer_cn_id} unsuccessful")
        
        self.get_logger().info("FeedbackProjectorDriverBase initialized successfully")

    def _load_projector_configs(self):
        """Load projector configurations from hardware.yaml."""
        try:
            # Get hardware configuration
            config_data = self.get_config("hardware.yaml")
            if config_data:
                hardware_config = yaml.safe_load(config_data) if isinstance(config_data, str) else config_data
                
                # Extract projector-related hardware configurations
                if isinstance(hardware_config, dict) and "hardware" in hardware_config:
                    for hw_item in hardware_config["hardware"]:
                        if isinstance(hw_item, dict) and "hardware_id" in hw_item:
                            # Store all hardware configs that might be projectors
                            # The actual filtering can be done in concrete implementations
                            self.projector_configs[hw_item["hardware_id"]] = hw_item
                            
                self.get_logger().info(f"Loaded {len(self.projector_configs)} hardware configurations")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load projector configs: {str(e)}")
            self.projector_configs = {}


    @abstractmethod
    def project_feedback(self, projector_id: str, data: Dict[str, Any]):
        """
        Project feedback data using the specified projector.
        
        Args:
            projector_id: Hardware ID of the projector (from config files)
            data: Feedback data to be projected
            
        This method must be implemented by concrete projector driver classes.
        """
        pass

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")


    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        self.get_logger().debug("Checking plugins for FeedbackProjectorDriverBase")
        
        # Check if concrete implementation provides required projection capabilities
        required_methods = ['project_feedback']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().warn(f"Required method '{method_name}' not properly implemented")

    def urgent_stop(self):
        """Emergency stop all projectors."""
        self.logging("Emergency stop triggered - stopping all projectors", "WARN")
        for robot_id in self.robot_config.keys():
            try:
                # Try to send stop command to each robot
                self.logging(f"Emergency stop sent to robot: {robot_id}", "INFO")
            except Exception as e:
                self.logging(f"Error stopping robot {robot_id}: {str(e)}", "ERROR")

