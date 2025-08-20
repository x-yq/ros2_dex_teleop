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
        self.raw_data_subscribers: Dict[str, Subscription] = {}
        self.processed_data_publishers: Dict[str, Publisher] = {}
        
        self.my_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}
        

    def publish_processed_data(self, topic_id: str, msg: Any):
        """
        Publish processed data. Multiple topics possible.
        
        Args:
            topic_id: Topic identifier for publishing
            msg: Processed message to publish
        """
        if topic_id in self.processed_data_publishers:
            try:
                publisher = self.processed_data_publishers[topic_id]
                publisher.publish(msg)
                self.logging(f"Published processed data to topic: {topic_id}", "DEBUG")
            except Exception as e:
                self.logging(f"Failed to publish processed data to {topic_id}: {str(e)}", "ERROR")
        else:
            self.logging(f"Publisher for topic {topic_id} not found", "ERROR")

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")

    def urgent_stop(self):
        """Emergency stop data processing."""
        self.logging("Emergency stop triggered - disabling data processing", "WARN")
        self.disable_processing()

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        required_methods = ['raw_data_sub_callback', 'process_data']
        for plugin_name, plugin in self.plugins.items():
            for method_name in required_methods:
                if not hasattr(plugin, method_name):
                    self.logging(f"Plugin {plugin_name} missing method: {method_name}", "ERROR")
                    return False
        return True
