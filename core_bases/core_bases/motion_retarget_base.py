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
        
        # Attributes
        self.control_data_subscribers: Dict[str, Subscription] = {}
        self.cmd_publishers: Dict[str, Publisher] = {}
        
        # Transform data storage
        self.static_transforms_cache: Dict[tuple, Any] = {}

        self.my_config: Dict[str, Any] = {}
        self.calibration_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}


    def publish_cmd(self, topic_id: str, msg: Any):
        """
        Publish robot command to a specific topic.
        
        Args:
            topic_id: Topic identifier for publishing
            msg: Command message to publish in robot space
        """
        if topic_id in self.cmd_publishers:
            try:
                publisher = self.cmd_publishers[topic_id]
                publisher.publish(msg)
                # self.logging(f"Published robot command to topic: {topic_id}", "DEBUG")
            except Exception as e:
                self.logging(f"Failed to publish command to {topic_id}: {str(e)}", "ERROR")
        else:
            self.logging(f"Publisher for topic {topic_id} not found", "ERROR")

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

    def urgent_stop(self):
        """Emergency stop motion transform processing."""
        self.logging("Emergency stop triggered - disabling motion transform", "WARN")
        self.disable_transform()

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        required_methods = [
            'control_data_sub_callback', 
            'retarget_control_to_robot_space',
        ]
        for plugin_name, plugin in self.plugins.items():
            for method_name in required_methods:
                if not hasattr(plugin, method_name):
                    self.logging(f"Plugin {plugin_name} missing method: {method_name}", "ERROR")
                    return False
        return True