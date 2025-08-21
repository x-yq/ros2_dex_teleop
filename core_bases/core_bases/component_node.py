# src/core_bases/core_bases/component_node.py

from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from typing import Dict, List, Any, Optional
from builtin_interfaces.msg import Time
import json

from system_interfaces.srv import Logging, GetConfig
from system_interfaces.msg import ComponentStatus, SystemCommand
import time

class ComponentNode(Node, ABC):
    """
    Abstract base class for all component nodes in the system.
    Provides unified attributes, service clients, plugin management,
    and communication setup via SystemManager.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id)  # node name = layer_cn_id
        self.layer_cn_id = layer_cn_id
        self.ros2_domain_id = ros2_domain_id
        self.state = "disconnected"
        self.valid_states = ["disconnected", "ready", "running"]

        # Clients to SystemManager
        self.logging_client = self.create_client(Logging, "logging")
        self.get_config_client = self.create_client(GetConfig, "get_config")

        # Component communication topics
        self._setup_component_communication()

        # Ensure services are ready
        self._wait_for_services()
        
    # ------------------------------------------------------------------
    # Component Communication Setup
    # ------------------------------------------------------------------
    
    def _setup_component_communication(self):
        """Setup component communication topics"""
        # Subscribe to system channel for commands from SystemManager
        self.system_channel_sub = self.create_subscription(
            SystemCommand,
            'system_channel',
            self._system_command_callback,
            10
        )
        
        # Publisher for component status to SystemManager
        self.component_channel_pub = self.create_publisher(
            ComponentStatus,
            'component_channel',
            10
        )
        
        self.get_logger().info(f"Component communication setup for {self.layer_cn_id}")
    
    def _system_command_callback(self, msg):
        """Handle commands from SystemManager"""
        try:
            command = msg.command.lower()
            
            if command == "get_status":
                self._publish_status()
                self.get_logger().debug(f"Status requested by {msg.sender}")
                
            elif command == "urgent_stop":
                self.get_logger().warn(f"URGENT STOP command received from {msg.sender}")
                self.urgent_stop()
                self.set_state("disconnected")
                self._publish_status()

            elif command == "start":
                self.get_logger().warn(f"START command received from {msg.sender}")
                self.start_component()
                self._publish_status()

            elif command == "pause":
                self.get_logger().warn(f"PAUSE command received from {msg.sender}")
                self.pause_component()
                self._publish_status()
                
            else:
                self.get_logger().warn(f"Unknown command received: {command}")
                
        except Exception as e:
            self.get_logger().error(f"Error handling system command: {e}")
    
    def _publish_status(self):
        """Publish current component status"""
        try:
            status_msg = ComponentStatus()
            status_msg.layer_cn_id = self.layer_cn_id
            status_msg.status = self.state
            status_msg.timestamp = self.get_clock().now().to_msg()
            
            self.component_channel_pub.publish(status_msg)
            self.get_logger().debug(f"Published status: {self.state}")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    # ------------------------------------------------------------------
    # Service handling
    # ------------------------------------------------------------------

    def _initialize_node(self):
        """Initialize the node. Can be overwritten"""
        self.get_logger().info(f"Initializing: {self.layer_cn_id}")
        
        # Register publishers and subscribers
        self.register_pub_sub()
        
        # Verify plugins
        if self.check_plugins():
            self.set_state("ready")
            self.get_logger().info(f"{self.layer_cn_id} initialized successfully")
            # Publish initial status
            self._publish_status()
        else:
            self.set_state("disconnected")
            self.get_logger().warn(f"{self.layer_cn_id} initialization failed")
            # Publish error status
            self._publish_status()

    def _wait_for_services(self):
        """Block until SystemManager services are available."""
        self.get_logger().info("Waiting for SystemManager services...")
        for client, name in [
            (self.logging_client, "logging"),
            (self.get_config_client, "get_config"),
        ]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f"{name} service not available, retrying...")
        self.get_logger().info("All SystemManager services available")

    def get_config(self, config_file_name: str, config_keys: Optional[List[str]] = None):
        """Fetch config data from SystemManager."""
        req = GetConfig.Request()
        req.config_file_name = config_file_name
        req.config_keys = config_keys if config_keys else []

        future = self.get_config_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"Config received from {config_file_name}")
            return future.result().config_data  # JSON/YAML string
        else:
            self.get_logger().error(f"Failed to get config: {future.result().message if future.result() else 'unknown error'}")
            return None

    def logging(self, msg: str, log_level: str = "INFO"):
        """Send log message to SystemManager."""
        req = Logging.Request()
        req.layer_cn_id = self.layer_cn_id
        req.log_level = log_level
        req.message = msg
        req.timestamp = self.get_clock().now().to_msg()

        future = self.logging_client.call_async(req)
        return future

    # ------------------------------------------------------------------
    # State management methods
    # ------------------------------------------------------------------
    
    def set_state(self, new_state: str, publish_status: bool = True):
        """Set component state and optionally publish status update"""
        if new_state not in self.valid_states:
            self.get_logger().error(f"State not valid: {new_state}")
            return
        old_state = self.state
        self.state = new_state
        
        if old_state != new_state:
            self.get_logger().info(f"State changed: {old_state} -> {new_state}")
            
            if publish_status:
                self._publish_status()
    
    def get_state(self) -> str:
        """Get current component state"""
        return self.state

    # ------------------------------------------------------------------
    # Component lifecycle methods
    # ------------------------------------------------------------------
    

    # If DataProcessorBase doesn't inherit from ComponentNode, you need these methods:
    def _wait_for_start_cmd(self):
        """Non-blocking wait for start command using ROS2 timer"""
        self.get_logger().info(f"Waiting for start command... Current state: {self.state}")
        
        # Cancel existing timer if any
        if hasattr(self, 'wait_timer') and self.wait_timer is not None:
            self.wait_timer.cancel()
        
        # Create a timer that checks periodically (non-blocking)
        self.wait_timer = self.create_timer(1.0, self._check_start_condition)
    
    def _check_start_condition(self):
        """Timer callback to check if we should continue waiting"""
        if self.state == 'running':
            # Stop waiting, cancel timer
            if hasattr(self, 'wait_timer') and self.wait_timer is not None:
                self.wait_timer.cancel()
                self.wait_timer = None
            self.get_logger().info("Start condition met - component is now running!")
        else:
            # Still waiting
            self.get_logger().debug(f"Still waiting for start command... State: {self.state}")
    
    def start_component(self):
        """Start triggered by SystemManager. Need to change the state"""
        self.get_logger().info(f"Starting component: {self.layer_cn_id}")
        self.set_state('running')
        
        # Cancel wait timer if it exists
        if hasattr(self, 'wait_timer') and self.wait_timer is not None:
            self.wait_timer.cancel()
            self.wait_timer = None
            self.get_logger().info("Wait timer cancelled - component now running")

    def pause_component(self):
        """Pause the component (running -> ready)"""
        self.get_logger().info(f"Pausing component: {self.layer_cn_id}")
        self.set_state("ready")
        self._wait_for_start_cmd()

    # ------------------------------------------------------------------
    # Error handling methods
    # ------------------------------------------------------------------
    
    def handle_error(self, error_msg: str, exception: Optional[Exception] = None):
        """Handle component errors"""
        self.set_state("disconnected")
        
        error_log = f"Component {self.layer_cn_id} error: {error_msg}"
        if exception:
            error_log += f" - Exception: {str(exception)}"
        
        self.get_logger().error(error_log)
        self.logging(error_log, "ERROR")

    # ------------------------------------------------------------------
    # Abstract methods – enforce subclasses to define
    # ------------------------------------------------------------------

    @abstractmethod
    def register_pub_sub(self):
        """Register publishers and subscribers (based on config)."""
        pass

    @abstractmethod
    def urgent_stop(self):
        """Emergency stop triggered by SystemManager. Need to change the state"""
        pass

    @abstractmethod
    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        pass

    # ------------------------------------------------------------------
    # Optional utilities
    # ------------------------------------------------------------------
    
    def __del__(self):
        """Destructor - ensure cleanup"""
        pass

    # ------------------------------------------------------------------
    # Config Loader Methods
    # ------------------------------------------------------------------
    
    def load_calibration_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load calibration configuration containing transform matrices and frame definitions.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'frame_ids', 'general_calibration_transforms', 
                        'scenario_0_transforms', 'scenario_1_transforms', 'scenario_2_transforms'
        
        Returns:
            Dictionary containing calibration data or None if failed
        """
        try:
            config_data_str = self.get_config("calibration", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded calibration config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration config: {e}")
            return None
    
    def load_data_stream_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load data stream configuration containing topic definitions.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'topics'
        
        Returns:
            Dictionary containing data stream configuration or None if failed
        """
        try:
            config_data_str = self.get_config("data_stream", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded data stream config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load data stream config: {e}")
            return None
    
    def load_hardware_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load hardware configuration containing device definitions and network settings.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'execution_plattforms', 'sensors', 'robots'
        
        Returns:
            Dictionary containing hardware configuration or None if failed
        """
        try:
            config_data_str = self.get_config("hardware", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded hardware config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load hardware config: {e}")
            return None
    
    def load_container_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load container configuration for Docker deployment.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'platforms'
        
        Returns:
            Dictionary containing container configuration or None if failed
        """
        try:
            config_data_str = self.get_config("container", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded container config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load container config: {e}")
            return None
    
    def load_ros2_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load ROS2 middleware configuration.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'middleware'
        
        Returns:
            Dictionary containing ROS2 configuration or None if failed
        """
        try:
            config_data_str = self.get_config("ros2", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded ROS2 config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load ROS2 config: {e}")
            return None
    
    def load_demo_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load demo configuration for data collection and recording.
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys: 'demo_id', 'demo_root_path', 'description', 
                        'topics', 'video_codec', 'storage_format'
        
        Returns:
            Dictionary containing demo configuration or None if failed
        """
        try:
            config_data_str = self.get_config("demo", config_keys)
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded demo config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load demo config: {e}")
            return None
    
    def load_component_node_config(self, component_name: Optional[str] = None, 
                                  config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Load component node specific configuration from component_node folder.
        
        Args:
            component_name: Name of the component config file (without extension).
                           If None, uses self.layer_cn_id
            config_keys: Specific keys to retrieve. If None, returns entire config.
                        Available keys depend on component type: 'layer_cn_id', 'pkg_name', 
                        'meta', 'default_plugin', 'plugins'
        
        Returns:
            Dictionary containing component configuration or None if failed
        """
        try:
            # Use self.layer_cn_id if no component_name specified
            target_component = component_name or self.layer_cn_id
            
            # Try direct component name first
            config_data_str = self.get_config(f"component_node.{target_component}", config_keys)
            
            # If direct access fails, try accessing through component_node group
            if not config_data_str:
                component_node_config = self.get_config("component_node")
                if component_node_config:
                    all_components = json.loads(component_node_config)
                    if target_component in all_components:
                        component_config = all_components[target_component]
                        
                        # Filter by config_keys if specified
                        if config_keys and isinstance(component_config, dict):
                            filtered_config = {}
                            for key in config_keys:
                                if key in component_config:
                                    filtered_config[key] = component_config[key]
                            component_config = filtered_config
                        
                        config_data_str = json.dumps(component_config)
            
            if config_data_str:
                config_data = json.loads(config_data_str) if isinstance(config_data_str, str) else config_data_str
                self.get_logger().info(f"Loaded component config for {target_component}")
                return config_data
            else:
                self.get_logger().warn(f"Component config not found for {target_component}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Failed to load component config: {e}")
            return None
    
    def load_my_config(self, config_keys: Optional[List[str]] = None) -> Optional[Dict[str, Any]]:
        """
        Convenience method to load this component's own configuration.
        Equivalent to load_component_node_config(self.layer_cn_id, config_keys)
        
        Args:
            config_keys: Specific keys to retrieve. If None, returns entire config.
        
        Returns:
            Dictionary containing this component's configuration or None if failed
        """
        return self.load_component_node_config(self.layer_cn_id, config_keys)