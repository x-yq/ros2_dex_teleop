# monitor_interface/monitor_base.py

from abc import abstractmethod
import rclpy
from builtin_interfaces.msg import Time, Duration

from .component_node import ComponentNode
from system_manager_interfaces.srv import StartMonitor, StopMonitor


class KPIMonitorBase(ComponentNode):
    """
    Abstract base class for all monitor nodes.
    Extends ComponentNode with monitoring-specific attributes and services.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: int = None):
        super().__init__(layer_cn_id, ros2_domain_id)

        self.kpi_metrics = {}
        self.monitor_timer = None
        self.active_session_id = None
        self.monitored_container_id = None

        # Expose monitor services
        self.start_monitor_srv = self.create_service(
            StartMonitor,
            "start_monitor",
            self.start_monitor_srv_callback
        )
        self.stop_monitor_srv = self.create_service(
            StopMonitor,
            "stop_monitor",
            self.stop_monitor_srv_callback
        )
        self.lifecycle = "ready"

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

    def log_kpi(self, log_file: str, msg: dict):
        """Use ComponentNode.logging to send KPI data to SystemManager with MONITOR log level."""
        return self.logging(f"{log_file}: {msg}", log_level="MONITOR")
