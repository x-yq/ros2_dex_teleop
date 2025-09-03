# monitor_interface/monitor_base.py

from abc import abstractmethod
import rclpy
from builtin_interfaces.msg import Time, Duration

from .component_node import ComponentNode
from system_interfaces.srv import StartMonitor, StopMonitor
import json
from rclpy.callback_groups import ReentrantCallbackGroup


class KPIMonitorBase(ComponentNode):
    """
    Abstract base class for all monitor nodes.
    Extends ComponentNode with monitoring-specific attributes and services.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: int = None):
        super().__init__(layer_cn_id, ros2_domain_id)

        self.callback_group = ReentrantCallbackGroup()


        # Expose monitor services
        self.start_monitor_srv = self.create_service(
            StartMonitor,
            f"{layer_cn_id}/start_monitor",
            self.start_monitor_srv_callback,
            callback_group=self.callback_group
        )
        self.stop_monitor_srv = self.create_service(
            StopMonitor,
            f"{layer_cn_id}/stop_monitor",
            self.stop_monitor_srv_callback,
            callback_group=self.callback_group

        )

    # ------------------------------------------------------------------
    # Abstract methods for subclasses (plugins must implement)
    # ------------------------------------------------------------------

    @abstractmethod
    def collect_kpi(self, object_id: str = None):
        """Collect KPI metrics like CPU, memory, disk, bandwidth."""
        pass

    @abstractmethod
    def start_monitor_srv_callback(self, request: StartMonitor.Request, response: StartMonitor.Response):
        """
        Begin monitoring container resources.
        - request.container_id: which container to monitor
        - request.metrics: which metrics to collect
        """
        pass

    @abstractmethod
    def stop_monitor_srv_callback(self, request: StopMonitor.Request, response: StopMonitor.Response):
        """
        Stop monitoring and return collected results.
        """
        pass

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------

    # def log_kpi(self, msg):
    #     """Use ComponentNode.logging to send KPI data to SystemManager with MONITOR log level."""
    #     to_log = ''
    #     for i in msg:
    #         m = json.dumps(i)
    #         to_log += m
    #     self.get_logger().warn(to_log)
    #     self.logging(to_log, log_level="MONITOR")

    def log_kpi(self, msg):
        """Use ComponentNode.logging to send KPI data to SystemManager with MONITOR log level."""
        
        if isinstance(msg, str):
            # If msg is already a string, log it directly
            to_log = msg
        elif isinstance(msg, (dict, list)):
            # If msg is JSON-like (dict or list), use json.dumps
            to_log = json.dumps(msg)
        else:
            # For other types (like the original list of dicts), process each item
            to_log = ''
            try:
                for i in msg:
                    m = json.dumps(i)
                    to_log += m
            except (TypeError, ValueError):
                # If json.dumps fails, convert to string
                to_log = str(msg)
        
        self.get_logger().warn(to_log)
        self.logging(to_log, log_level=f"MONITOR_{self.layer_cn_id}")
    # ------------------------------------------------------------------
    # Required methods
    # ------------------------------------------------------------------

    def register_pub_sub(self):
        """Register publishers and subscribers based on configuration."""
        # This will be implemented by subclasses based on their specific needs
        self.logging("Base register_pub_sub called - should be overridden", "WARN")

    def register_timer(self):
        """Register timers based on configuration."""
        self.logging("Base register_timer called - should be overridden", "WARN")

    def urgent_stop(self):
        """Emergency stop all projectors."""
        self.logging("Emergency stop triggered - stopping all projectors", "WARN")
        for robot_id in self.robot_config.keys():
            try:
                # Try to send stop command to each robot
                self.logging(f"Emergency stop sent to robot: {robot_id}", "INFO")
            except Exception as e:
                self.logging(f"Error stopping robot {robot_id}: {str(e)}", "ERROR")

    def _load_my_config(self):
        self.logging("Base _load_my_config called - should be overridden", "WARN")
        
    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        self.get_logger().debug(f"Checking plugins for {self.layer_cn_id}")
        
        # Check if concrete implementation provides required projection capabilities
        required_methods = ['register_pub_sub', 'register_timer', 'urgent_stop', 'start_monitor_srv_callback', 'stop_monitor_srv_callback']
        
        for method_name in required_methods:
            if not hasattr(self, method_name) or not callable(getattr(self, method_name)):
                self.get_logger().warn(f"Required method '{method_name}' not properly implemented")
                return False
        return True
