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
        self.cmd_subscribers: Dict[str, Subscription] = {}
        self.state_publishers: Dict[str, Publisher] = {}

        self.my_config: Dict[str, Any] = {}
        self.hardware_config: Dict[str, Any] = {}
        self.data_stream_config: Dict[str, Any] = {}
        
    def _initialize_node(self):
        """Initialize the driver node."""
        self.get_logger().info(f"Initializing RobotDriverBase: {self.layer_cn_id}")
        
        # # Load projector configurations
        # self._load_robot_config()
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Verify plugins
        if self.check_plugins():
            self.set_state("ready")
            self.get_logger().info(f"{self.layer_cn_id} initialized successfully")
        else:
            self.get_logger().warn(f"{self.layer_cn_id} unsuccessful")

        self.get_logger().info("RobotDriverBase initialized successfully")

    # def _load_robot_config(self):
    #     """Load robot configuration from hardware config files."""
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
    #             self.robot_config = {}
                
    #             for hw_item in hardware_list:
    #                 if isinstance(hw_item, dict) and 'hardware_id' in hw_item:
    #                     hardware_id = hw_item['hardware_id']
    #                     # Create robot config entry with IP and access info
    #                     robot_info = {
    #                         'ip': hw_item.get('ip', ''),
    #                         'access_port': hw_item.get('access_port', 22),
    #                         'description': f"Robot on {hardware_id}",
    #                         'execution_platform': hw_item.get('execution_plattform', False)
    #                     }
                        
    #                     # Add SSH info if available
    #                     if 'ssh' in hw_item:
    #                         robot_info.update(hw_item['ssh'])
                            
    #                     self.robot_config[hardware_id] = robot_info
                
    #             self.logging(f"Loaded robot config for hardware: {list(self.robot_config.keys())}", "INFO")
    #             self.logging(f"Running in {'simulated' if self.simulated else 'physical'} mode", "INFO")
    #         else:
    #             self.logging("Failed to load robot configuration", "ERROR")
    #     except Exception as e:
    #         self.logging(f"Error loading robot config: {str(e)}", "ERROR")
    #         self.robot_config = {}
    
    # def get_robot_ip_port(self, robot_id: str) -> Optional[tuple]:
    #     """Get robot IP and port by robot ID."""
    #     if robot_id in self.robot_config:
    #         robot_info = self.robot_config[robot_id]
    #         ip = robot_info.get('ip')
    #         port = robot_info.get('access_port')
    #         if ip and port:
    #             return (ip, port)
    #     return None

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

    def publish_state(self, topic_id: str, state: Any):
        """
        Publish robot state to a specific topic.
        
        Args:
            publisher: ROS2 publisher instance
            topic_id: Topic identifier
            state: Robot state data to publish
        """
        try:
            publisher = self.state_publishers[topic_id]
            publisher.publish(state)
            self.logging(f"Published robot state to topic: {topic_id}", "DEBUG")
        except Exception as e:
            self.logging(f"Failed to publish state to {topic_id}: {str(e)}", "ERROR")

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

    def urgent_stop(self):
        """Emergency stop all robots."""
        self.logging("Emergency stop triggered - stopping all robots", "WARN")
        for robot_id in self.robot_config.keys():
            try:
                # Try to send stop command to each robot
                self.execute_command(robot_id, {"type": "stop", "urgent": True})
                self.logging(f"Emergency stop sent to robot: {robot_id}", "INFO")
            except Exception as e:
                self.logging(f"Error stopping robot {robot_id}: {str(e)}", "ERROR")

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        required_methods = ['get_state', 'cmd_sub_callback', 'home', 'execute_command']
        for plugin_name, plugin in self.plugins.items():
            for method_name in required_methods:
                if not hasattr(plugin, method_name):
                    self.logging(f"Plugin {plugin_name} missing method: {method_name}", "ERROR")
                    return False
        return True
