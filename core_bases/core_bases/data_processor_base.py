# core_bases/data_processor_base.py

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from .component_node import ComponentNode


class DataProcessorBase(ComponentNode, ABC):
    """
    Abstract base class for data processor nodes.
    Processes raw data and publishes processed results.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Attributes
        
        self.my_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}

        self._initialize_node() # definded in the ComponentNode

    def _load_root_configs(self):
                        
        # Load data stream configuration
        self.data_stream_config = self.load_data_stream_config()
        if not self.data_stream_config:
            self.get_logger().error("Failed to load data stream configuration")
            self.data_stream_config = {}

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")
    
    def register_timer(self):
        """Register timers based on configuration."""
        self.logging("Base register_timer called - should be overridden", "WARN")

    def urgent_stop(self):
        """Emergency stop data processing."""
        self.logging("Base urgent_stop called - should be overridden", "WARN")
    
    def _load_my_config(self):
        """Load the component plugin configuration"""
        self.logging("Base _load_my_config called - should be overridden", "WARN")
    
    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        self.get_logger().debug(f"Checking plugins for {self.layer_cn_id}")
        
        # Check if concrete implementation provides required projection capabilities
        required_methods = ['register_pub_sub', 'register_timer', 'urgent_stop', '_load_my_config']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().warn(f"Required method '{method_name}' not properly implemented")
                return False
        return True
