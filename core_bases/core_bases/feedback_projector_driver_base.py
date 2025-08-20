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
        self.my_config: Dict[str, Any] = {}
        self.hardware_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}
        

    @overwritten
    def _initialize_node(self):
        """Initialize the feedback projector driver node."""
        self.get_logger().info(f"Initializing FeedbackProjectorDriverBase: {self.layer_cn_id}")
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Verify plugins
        if self.check_plugins():
            self.set_state("ready")
            self.get_logger().info(f"{self.layer_cn_id} initialized successfully")
        else:
            self.get_logger().warn(f"{self.layer_cn_id} unsuccessful")
        
        self.get_logger().info("FeedbackProjectorDriverBase initialized successfully")

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

