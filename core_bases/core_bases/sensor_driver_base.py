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
        self.data_publishers: Dict[str, Publisher] = {}

        self.my_config: Dict[str, Any] = {}
        self.hardware_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}
        
    @overwritten
    def _initialize_node(self):
        """Initialize the driver node."""
        self.get_logger().info(f"Initializing FeedbackProjectorDriverBase: {self.layer_cn_id}")
        
        # # Load projector configurations
        # self._load_sensor_config()
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Verify plugins
        if self.check_plugins():
            self.set_state("ready")
            self.get_logger().info(f"{self.layer_cn_id} initialized successfully")
        else:
            self.get_logger().warn(f"{self.layer_cn_id} unsuccessful")
        
        self.get_logger().info("SensorDriverBase initialized successfully")

    # def _load_sensor_config(self):
    #     """Load sensor configuration from hardware config files."""
    #     try:
    #         config_data = self.get_config("hardware.yaml")
    #         if config_data:
    #             # Handle both YAML and JSON formats
    #             if config_data.strip().startswith('{'):
    #                 hardware_config = json.loads(config_data)
    #             else:
    #                 hardware_config = yaml.safe_load(config_data)
                
    #             # Convert list format to dictionary for easier access
    #             hardware_list = hardware_config.get('hardware', [])
    #             self.sensor_config = {}
                
    #             for hw_item in hardware_list:
    #                 if isinstance(hw_item, dict) and 'hardware_id' in hw_item:
    #                     hardware_id = hw_item['hardware_id']
    #                     # Create sensor config entry with IP and access info
    #                     sensor_info = {
    #                         'ip': hw_item.get('ip', ''),
    #                         'access_port': hw_item.get('access_port', 22),
    #                         'description': f"Sensor on {hardware_id}",
    #                         'execution_platform': hw_item.get('execution_plattform', False)
    #                     }
                        
    #                     # Add SSH info if available
    #                     if 'ssh' in hw_item:
    #                         sensor_info.update(hw_item['ssh'])
                            
    #                     self.sensor_config[hardware_id] = sensor_info
                
    #             self.logging(f"Loaded sensor config for hardware: {list(self.sensor_config.keys())}", "INFO")
    #         else:
    #             self.logging("Failed to load sensor configuration", "ERROR")
    #     except Exception as e:
    #         self.logging(f"Error loading sensor config: {str(e)}", "ERROR")
    #         self.sensor_config = {}
    
    # def get_sensor_ip_port(self, sensor_id: str) -> Optional[tuple]:
    #     """Get sensor IP and port by sensor ID."""
    #     if sensor_id in self.sensor_config:
    #         sensor_info = self.sensor_config[sensor_id]
    #         ip = sensor_info.get('ip')
    #         port = sensor_info.get('access_port')
    #         if ip and port:
    #             return (ip, port)
    #     return None

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

    def publish_data(self, topic_id: str, data: Any):
        """
        Publish data to a specific topic.
        
        Args:
            publisher: ROS2 publisher instance
            topic_id: Topic identifier
            data: Data to publish
        """
        try:
            publisher = self.data_publishers[topic_id]
            publisher.publish(data)
            self.logging(f"Published data to topic: {topic_id}", "DEBUG")
        except Exception as e:
            self.logging(f"Failed to publish data to {topic_id}: {str(e)}", "ERROR")

    @abstractmethod
    def stream(self):
        """
        Main streaming function that orchestrates the sensor data flow.
        Should implement the logic from start_sensor to publish_data to stop_sensor.
        This method should be called in a loop or timer callback.
        """
        pass

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")

    def urgent_stop(self):
        """Emergency stop all sensors."""
        self.logging("Emergency stop triggered - stopping all sensors", "WARN")
        for sensor_id in self.sensor_config.keys():
            try:
                self.stop_sensor(sensor_id)
            except Exception as e:
                self.logging(f"Error stopping sensor {sensor_id}: {str(e)}", "ERROR")

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        # Base implementation - can be overridden by subclasses
        required_methods = ['start_sensor', 'stop_sensor', 'get_data_from_sensor', 'stream']
        for plugin_name, plugin in self.plugins.items():
            for method_name in required_methods:
                if not hasattr(plugin, method_name):
                    self.logging(f"Plugin {plugin_name} missing method: {method_name}", "ERROR")
                    return False
        return True