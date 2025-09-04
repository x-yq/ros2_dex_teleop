# core_bases/sensor_driver_base.py

from abc import ABC, abstractmethod
import json
import yaml
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.publisher import Publisher

from .component_node import ComponentNode


class SensorDriverBase(ComponentNode, ABC):
    """
    Abstract base class for sensor driver nodes.
    Handles both input devices and feedback devices.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Attributes
        self.sensor_config: Dict[str, Dict[str, str]] = {}

        self.my_config: Dict[str, Any] = {}
        self.hardware_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}

        self.sensor_config = {}
        
        self._initialize_node() # definded in the ComponentNode

    def _load_root_configs(self):
        
        self.hardware_config = self.load_hardware_config()
        if self.hardware_config:
            self.get_logger().info("Loaded hardware config")
            
            # Extract robot configuration
            if 'sensors' in self.hardware_config:
                self.sensor_config = self.hardware_config['sensors']
        
        # Load data stream configuration
        self.data_stream_config = self.load_data_stream_config()
        if not self.data_stream_config:
            self.get_logger().warn("Failed to load data stream configuration")

    @abstractmethod
    def start_sensor(self, sensor_id: str) -> bool:
        """
        Start a specific sensor.
        
        Args:
            sensor_id: Identifier of the sensor to start
            
        Returns:
            bool: True if sensor started successfully, False otherwise
        """
        pass

    @abstractmethod
    def stop_sensor(self, sensor_id: str) -> bool:
        """
        Stop a specific sensor.
        
        Args:
            sensor_id: Identifier of the sensor to stop
            
        Returns:
            bool: True if sensor stopped successfully, False otherwise
        """
        pass

    @abstractmethod
    def get_data_from_sensor(self, sensor_id: str) -> Any:
        """
        Get data from a specific sensor.
        
        Args:
            sensor_id: Identifier of the sensor
            
        Returns:
            Any: Sensor data in appropriate format
        """
        pass

    @abstractmethod
    def stream(self):
        """
        Main streaming function that orchestrates the sensor data flow.
        Should implement the logic from start_sensor to publish_msg to stop_sensor.
        This method should be called in a loop or timer callback.
        """
        pass

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")

    
    def register_timer(self):
        """Register timers based on configuration."""
        self.logging("Base register_timer called - should be overridden", "WARN")

    def urgent_stop(self):
        self.logging("Emergency stop triggered - should be overridden", "WARN")

    def _load_my_config(self):
        self.logging("Base _load_my_config called - should be overridden", "WARN")

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        self.get_logger().debug(f"Checking plugins for {self.layer_cn_id}")
        
        # Check if concrete implementation provides required projection capabilities
        required_methods = ['register_pub_sub', 'register_timer', 'urgent_stop', '_load_my_config', 'start_sensor', 'stop_sensor']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().warn(f"Required method '{method_name}' not properly implemented")
                return False
        return True