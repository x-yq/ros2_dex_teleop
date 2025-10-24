# src/core_bases/core_bases/component_node.py

from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from typing import Dict, List, Any, Optional
from builtin_interfaces.msg import Time
import json
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from system_interfaces.srv import Logging, GetConfig, ComponentService
# from system_interfaces.msg import ComponentStatus, SystemCommand
from system_interfaces.msg import *
from component_interfaces.msg import *
import time
import importlib
import subprocess
from rclpy.serialization import serialize_message
import asyncio

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

        self.compo_subscribers: Dict[str, Subscription] = {}
        self.compo_publishers: Dict[str, Publisher] = {}

        self.compo_client_callback_group = ReentrantCallbackGroup()
        self.compo_srv_callback_group = ReentrantCallbackGroup()

        # Clients to SystemManager
        self.logging_client = self.create_client(Logging, "logging") #, callback_group=self.compo_client_callback_group)
        self.get_config_client = self.create_client(GetConfig, "get_config") #, callback_group= self.compo_client_callback_group)

        
        # Latency test
        self.pending_ping_ids = {}   # {send_id: send_time_ns}
        self.latency_pubs = {}       # pong publishers
        self.latency_subs = {}       # pong subscribers

        self.simulate_latency = 0


        self.compo_timers = []

        self._setup_component_communication()

        # Ensure services are ready
        self._wait_for_services()
        
    # ------------------------------------------------------------------
    # Component Communication Setup
    # ------------------------------------------------------------------
    
    def _setup_component_communication(self):
        """Setup component communication service"""

        # Publisher for component status to SystemManager
        self.component_channel_pub = self.create_publisher(
            ComponentStatus,
            'component_states',
            10
        )

        # Create service server for handling commands from SystemManager
        if 'monitor' in self.layer_cn_id or 'data_collector' in self.layer_cn_id:
            return

        service_name = f'cn_srv_{self.layer_cn_id}'
        self.component_service = self.create_service(
            ComponentService,
            service_name,
            self._component_service_callback,
            callback_group = self.compo_srv_callback_group
        )
        
        self.get_logger().info(f"Component service '{service_name}' created for {self.layer_cn_id}")


    def _component_service_callback(self, request, response):
        """Handle service requests from SystemManager"""
        try:
            response = ComponentService.Response()
            
            command = request.command.lower()
            
            # self.get_logger().debug(f"Service request received: {command} from {request.sender}")

            # self.logging(f"Service request received: {command} from {request.sender}", "DEBUG")

            if command == "checkup":
                # Return current status
                response.status = self.state  # Assuming you have a state method
                response.message = f"Status requested by {request.sender}"
                response.success = True
                self.get_logger().debug(f"Status requested by {request.sender}: {response.status}")
                
            elif command == "urgent_stop":
                self.get_logger().warn(f"URGENT STOP command received from {request.sender}")
                try:
                    self.stop_component()
                    self.urgent_stop()
                    self.set_state("disconnected")
                    response.status = self.state
                    response.message = "Urgent stop executed successfully"
                    response.success = True
                except Exception as e:
                    response.status = "error"
                    response.message = f"Failed to execute urgent stop: {e}"
                    response.success = False
                    self.get_logger().error(f"Error during urgent stop: {e}")
                
            elif command == "start":
                self.get_logger().warn(f"START command received from {request.sender}")
                try:
                    self.set_state("running")
                    self.start_component()
                    response.status = "running"
                    response.message = "Start command executed successfully"
                    response.success = True
                except Exception as e:
                    response.status = "error"
                    response.message = f"Failed to execute start: {e}"
                    response.success = False
                    self.get_logger().error(f"Error during start: {e}")
                
            elif command == "pause":
                self.get_logger().warn(f"PAUSE command received from {request.sender}")
                try:
                    self.set_state("ready")
                    self.pause_component()
                    response.status = "ready"
                    response.message = "Pause command executed successfully"
                    response.success = True
                except Exception as e:
                    response.status = "error"
                    response.message = f"Failed to execute pause: {e}"
                    response.success = False
                    self.get_logger().error(f"Error during pause: {e}")
                
            else:
                response.status = self.state
                response.message = f"Unknown command received: {command}"
                response.success = False
                self.get_logger().warn(f"Unknown command received: {command}")
            
            # Set response timestamp
            response.timestamp = self.get_clock().now().to_msg()
            
        except Exception as e:
            self.get_logger().error(f"Error handling service request: {e}")
            response.status = "error"
            response.message = f"Service error: {e}"
            response.success = False
            response.timestamp = self.get_clock().now().to_msg()
        
        return response
    
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
    
    # NOTE: API
    def publish_msg(self, topic_id: str, msg: Any):
        """
        Publish data to a specific topic.
        
        Args:
            publisher: ROS2 publisher instance
            topic_id: Topic identifier
            msg: Data to publish
        """
        try:
            publisher = self.compo_publishers[topic_id]
            msg = self.send_ping(topic_id, msg)
            publisher.publish(msg)
            self.logging(f"Published data to topic: {topic_id}", "DEBUG")
        except Exception as e:
            self.logging(f"Failed to publish data to {topic_id}: {str(e)}", "ERROR")
        
    # ------------------------------------------------------------------
    # Componenet lifecycle control
    # ------------------------------------------------------------------
    
    # NOTE: API
    def _initialize_node(self):
        self.get_logger().info(f"Initializing: {self.layer_cn_id}")
        
        # Register publishers and subscribers
        self._load_root_configs() # definded in the child node
        self.register_pub_sub() # definded in the bottom node
        self.register_latency_sub_pub() # definded in this node
        
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

    def start_component(self):
        if self.compo_timers == [] or not self.compo_timers:
            self.register_timer() #defined in the bottom node
        else:
            for timer in self.compo_timers:
                # if timer.is_canceled():
                timer.reset()

    def pause_component(self):
        if self.compo_timers and self.compo_timers!=[]:
            for timer in self.compo_timers:
                # if timer is not None and not timer.is_canceled():
                timer.cancel()

    # NOTE: API for urgent_stop()
    def stop_component(self):
        if self.simulate_latency > 0:
            self._cleanup_latency_simulation()
        if self.compo_timers and self.compo_timers!=[]:
            for timer in self.compo_timers:
                # if timer is not None and not timer.is_canceled():
                timer.cancel()

    # ------------------------------------------------------------------
    # Service handling
    # ------------------------------------------------------------------

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


    async def get_config(self, config_file_name: str, config_keys: Optional[List[str]] = None):
        """Fetch config data from SystemManager."""
        req = GetConfig.Request()
        req.layer_cn_id = self.layer_cn_id
        req.config_file_name = config_file_name
        req.config_keys = config_keys if config_keys else []

        future = self.get_config_client.call_async(req)
        # await future

        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info(f"Config received from {config_file_name}")
            return future.result().config_data
        else:
            self.get_logger().error(f"Failed to get config: {future.result().message if future.result() else 'unknown error'}")
            return None
        # return future

    # NOTE: API
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
    # Error handling methods
    # ------------------------------------------------------------------
    
    # NOTE: API
    def handle_error(self, error_msg: str, exception: Optional[Exception] = None):
        """Handle component errors"""
        self.set_state("disconnected")
        
        error_log = f"Component {self.layer_cn_id} error: {error_msg}"
        if exception:
            error_log += f" - Exception: {str(exception)}"
        
        self.get_logger().error(error_log)
        self.logging(error_log, "ERROR")

    # ------------------------------------------------------------------
    # Abstract methods – enforce subclasses to define in plugin nodes
    # ------------------------------------------------------------------

    @abstractmethod
    def register_pub_sub(self):
        """Register publishers and subscribers (based on config)."""
        pass
    
    @abstractmethod
    def register_timer(self):
        """Register timer for publishers (based on config)."""
        pass

    @abstractmethod
    def urgent_stop(self):
        """Emergency stop triggered by SystemManager. Need to change the state, Need to call stop_component()"""
        pass

    @abstractmethod
    def _load_my_config(self):
        """Load the component plugin configuration"""
        pass

    @abstractmethod
    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        pass

    # ------------------------------------------------------------------
    # Latency Simulation Setup
    # ------------------------------------------------------------------
    
    # NOTE: API
    def _setup_latency_simulation(self):
        """Setup network latency simulation using tc qdisc"""
        try:
            # Get the network interface (typically eth0 or similar)
            interface = self._get_network_interface()
            if not interface:
                self.get_logger().warn("Could not determine network interface for latency simulation")
                return
            
            # Add latency using tc qdisc
            cmd = [
                "sudo", "tc", "qdisc", "add", "dev", interface, "root", "netem",
                "delay", f"{self.simulate_latency}ms"
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Network latency simulation setup: {self.simulate_latency}ms on {interface}")
            else:
                self.get_logger().error(f"Failed to setup latency simulation: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"Error setting up latency simulation: {e}")
    
    def _get_network_interface(self) -> Optional[str]:
        """Get the primary network interface name"""
        try:
            # Try to get default route interface
            result = subprocess.run(
                ["ip", "route", "show", "default"], 
                capture_output=True, text=True
            )
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if 'dev' in line:
                        parts = line.split()
                        dev_index = parts.index('dev')
                        if dev_index + 1 < len(parts):
                            return parts[dev_index + 1]
            
            # Fallback: try common interface names
            common_interfaces = ['eth0', 'enp0s3', 'ens33', 'wlan0']
            for iface in common_interfaces:
                result = subprocess.run(
                    ["ip", "link", "show", iface], 
                    capture_output=True, text=True
                )
                if result.returncode == 0:
                    return iface
                    
        except Exception as e:
            self.get_logger().warn(f"Error determining network interface: {e}")
        
        return None
    
    def _cleanup_latency_simulation(self):
        """Cleanup network latency simulation"""
        try:
            interface = self._get_network_interface()
            if interface:
                cmd = ["sudo", "tc", "qdisc", "del", "dev", interface, "root"]
                subprocess.run(cmd, capture_output=True, text=True)
                self.get_logger().info(f"Cleaned up latency simulation on {interface}")
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up latency simulation: {e}")

    # ------------------------------------------------------------------
    # Latency Measruement Methods (ping pong)
    # ------------------------------------------------------------------
    
    def _resolve_msg_type(self, msg_type_str: str):
        """
        Dynamically import a ROS 2 message type from a string like:
          'sensor_msgs/msg/Image'
          'component_interfaces/msg/RobotState'
        Returns the corresponding Python class.
        """
        try:
            pkg, _, msg_name = msg_type_str.partition('/msg/')
            self.get_logger().info(f"Resolving msg_type: {msg_type_str} -> {pkg}.msg.{msg_name}")

            if not pkg or not msg_name:
                raise ValueError(f"Invalid msg_type format: {msg_type_str}")

            module = importlib.import_module(f"{pkg}.msg")
            return getattr(module, msg_name)
        except Exception as e:
            self.get_logger().error(f"Failed to resolve message type '{msg_type_str}': {e}")
            raise

    def register_latency_sub_pub(self):
        """Register publishers and subscribers based on data_stream configuration for sub and pub"""
        try:
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return

            for topic_config in self.data_stream_config["topics"]:
                topic_id = topic_config["id"]
                msg_type = topic_config["msg_type"]
                latency_test = topic_config.get("latency_test_target", False)

                # ---------------------------
                # Ping node (subscribe the pong topic)
                # ---------------------------
                if latency_test and topic_config.get("publisher") == self.layer_cn_id:
                    # normal publisher already created elsewhere...
                    pong_topic = f"{topic_id}_pong"
                    self.latency_subs[topic_id] = self.create_subscription(
                        self._resolve_msg_type(msg_type),
                        pong_topic,
                        lambda msg, tid=topic_id: self._pong_callback(msg, tid),
                        10
                    )
                    self.get_logger().info(f"[Ping] Subscribed to pong topic: {pong_topic}")

                # ---------------------------
                # Pong node (subscriber side)
                # ---------------------------
                if latency_test and self.layer_cn_id in topic_config.get("subscribers", []):
                    pong_topic = f"{topic_id}_pong"
                    self.latency_pubs[topic_id] = self.create_publisher(
                        self._resolve_msg_type(msg_type),
                        pong_topic,
                        10
                    )
                    self.get_logger().info(f"[Pong] Created pong publisher: {pong_topic}")

        except Exception as e:
            self.handle_error("Failed to register publishers/subscribers for latency test", e)

    def send_ping(self, topic_id, msg):
        """Attach send_id and record timestamp before publishing"""
        if topic_id not in self.latency_subs:
            return msg

        if topic_id not in self.pending_ping_ids:
            self.pending_ping_ids[topic_id] = {}

        send_id = str(time.time_ns())
        if hasattr(msg, "send_id"):
            msg.send_id = send_id
        else:
            # fallback: attach dynamically if message has no header
            try:
                msg.header.frame_id = send_id
            except Exception:
                self.get_logger().warn(f"Message {type(msg)} has no header/send_id field")

        self.pending_ping_ids[topic_id][send_id] = time.time_ns()
        return msg

    def _pong_callback(self, msg, topic_id):
        """Handle pong replies and compute latency"""
        recv_time = time.time_ns()
        send_id = None
        if hasattr(msg, "send_id"):
            send_id = msg.send_id
        elif hasattr(msg, "header"):
            send_id = msg.header.frame_id

        if send_id and send_id in self.pending_ping_ids.get(topic_id, {}):
            send_time = self.pending_ping_ids[topic_id].pop(send_id)
            rtt_ms = (recv_time - send_time) / 1e6
            size_bytes = len(serialize_message(msg))
            self.logging(f"RTT = {rtt_ms/2.0:.3f} ms; Size = {size_bytes} bytes", f"LATENCY_{topic_id}")
            # self.get_logger().info(f"[Latency] Topic {topic_id} RTT = {rtt_ms:.3f} ms")

    # NOTE: API
    def pong_reply(self, topic_id, msg):
        """Republish ping as pong"""
        if topic_id in self.latency_pubs:
            self.latency_pubs[topic_id].publish(msg)


    # ------------------------------------------------------------------
    # Config Loader APIs
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
            config_data_str = asyncio.run(self.get_config("calibration", config_keys))
            if config_data_str:
                config_data = json.loads(config_data_str)
                self.get_logger().info(f"Loaded calibration config for {self.layer_cn_id}")
                return config_data
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration config: {e}")
            return None
    
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
            config_data_str = asyncio.run(self.get_config("calibration", config_keys))
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
            config_data_str = asyncio.run(self.get_config("data_stream", config_keys))
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
            config_data_str = asyncio.run(self.get_config("hardware", config_keys))
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
            config_data_str = asyncio.run(self.get_config("container", config_keys))
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
            config_data_str = asyncio.run(self.get_config("ros2", config_keys))
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
                        Available keys: 'demo_id'
        
        Returns:
            Dictionary containing demo configuration or None if failed
        """
        try:
            config_data_str = asyncio.run(self.get_config("demo", config_keys))
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
            config_data_str = asyncio.run(self.get_config(f"component_node.{target_component}", config_keys))
            
            # If direct access fails, try accessing through component_node group
            if not config_data_str:
                component_node_config = asyncio.run(self.get_config("component_node"))
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