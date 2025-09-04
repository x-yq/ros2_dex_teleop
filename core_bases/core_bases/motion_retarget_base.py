# core_bases/motion_retarget_base.py

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from .component_node import ComponentNode


class MotionRetargetBase(ComponentNode, ABC):
    """
    Abstract base class for motion transform nodes.
    Transforms control data using dynamic transforms and publishes robot commands.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        
        # Transform data storage
        self.static_transforms_cache: Dict[tuple, Any] = {}

        self.my_config: Dict[str, Any] = {}
        self.calibration_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}

        self._initialize_node() # definded in the ComponentNode

    def _load_root_configs(self):
        
        # Load calibration configuration
        self.calibration_config = self.load_calibration_config()
        if self.calibration_config:
            self.get_logger().info("Loaded calibration config")
        
        # Load data stream configuration
        self.data_stream_config = self.load_data_stream_config()
        if self.data_stream_config:
            self.get_logger().info("Loaded data stream config")

    @abstractmethod
    def retarget_control_to_robot_space(self, control_data: Any, transforms: Dict[str, Any]) -> Any:
        """
        Transform control data to robot command space using available transforms.
        This method should be implemented by subclasses to define specific transformation logic.
        
        Args:
            control_data: Input control data
            transforms: Available dynamic and static transforms
            
        Returns:
            Any: Transformed command in robot space
        """
        pass

    def clear_static_transform_cache(self):
        """Clear the static transform cache."""
        self.static_transforms_cache.clear()
        self.logging("Static transform cache cleared", "INFO")

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")

    def register_timer(self):
        """Register timers based on configuration."""
        self.logging("Base register_timer called - should be overridden", "WARN")

    def urgent_stop(self):
        """Emergency stop motion retarget processing."""
        self.logging("Emergency stop triggered - disabling motion retarget", "WARN")
    
    def _load_my_config(self):
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