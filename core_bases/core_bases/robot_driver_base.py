# core_bases/robot_driver_base.py

from abc import ABC, abstractmethod
import json
import yaml
from typing import Dict, List, Any, Optional
import rclpy
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from .component_node import ComponentNode


class RobotDriverBase(ComponentNode, ABC):
    """
    Abstract base class for robot driver nodes.
    Handles both physical and simulated robots.
    """

    def __init__(self, layer_cn_id: str, simulated: bool = False, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Attributes
        self.simulated = simulated

        self.my_config: Dict[str, Any] = {}
        self.hardware_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}

        self.robot_config: Dict[str, Any] = {}
        
        self._initialize_node() # definded in the ComponentNode
        
    def _load_root_configs(self):
        # Load hardware configuration
        self.hardware_config = self.load_hardware_config()
        if self.hardware_config:
            self.get_logger().info("Loaded hardware config")
            
            # Extract robot configuration
            if 'robots' in self.hardware_config:
                self.robot_config = self.hardware_config['robots']
        
        # Load data stream configuration
        self.data_stream_config = self.load_data_stream_config()
        if self.data_stream_config:
            self.get_logger().info("Loaded data stream config")

    @abstractmethod
    def get_state(self, robot_id: str) -> Any:
        """
        Get robot state. Implementation depends on robot type or SDK.
        For simulated robots, this could be a callback for subscribing to Gazebo state.
        
        Args:
            robot_id: Identifier of the robot
            
        Returns:
            Any: Robot state in appropriate format
        """
        pass

    @abstractmethod
    def home(self):
        """Move robot to home position."""
        pass

    @abstractmethod
    def execute_command(self, robot_id: str, command: Any) -> None:
        """
        Execute robot command. Implementation depends on robot type.
        
        For physical robots: Call robot SDK
        For simulated robots: 
            - Call Gazebo services, or
            - Import simulator Python API to create scene and render robot
            
        Args:
            robot_id: Identifier of the robot
            command: Command to execute
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
        """Emergency stop motion retarget processing."""
        self.logging("Emergency stop triggered - should be overridden", "WARN")

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
