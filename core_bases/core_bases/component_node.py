# core_interfaces/component_node.py

from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from typing import Dict, List, Any, Optional
from builtin_interfaces.msg import Time

from system_manager_interfaces.srv import Logging, GetConfig


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
        self.lifecycle = "unconfigured"

        # Clients to SystemManager
        self.logging_client = self.create_client(Logging, "logging")
        self.get_config_client = self.create_client(GetConfig, "get_config")

        # Plugin management + topics
        self.plugins: Dict[str, Any] = {}
        self.subscribe_topics: List[str] = []
        self.publish_topics: List[str] = []

        self.timer = None

        # Ensure services are ready
        self._wait_for_services()

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
    # Abstract methods — enforce subclasses to define
    # ------------------------------------------------------------------

    @abstractmethod
    def register_pub_sub(self):
        """Register publishers and subscribers (based on config)."""
        pass

    @abstractmethod
    def urgent_stop(self):
        """Emergency stop triggered by SystemManager."""
        pass

    @abstractmethod
    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        pass

    # ------------------------------------------------------------------
    # Optional utilities
    # ------------------------------------------------------------------

    def wait(self):
        """Barrier for async ops (optional override)."""
        self.get_logger().debug("wait() called — not implemented in base.")
