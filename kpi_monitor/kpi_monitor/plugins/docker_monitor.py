# location: src/kpi_monitor/kpi_monitor/plugins/docker_monitor.py
import docker
import psutil
from core_bases.kpi_monitor_base import KPIMonitorBase

'''
TODO: develope dummy collect_kpi() 
'''

class DockerMonitor(KPIMonitorBase):
    def __init__(self, layer_cn_id, ros2_domain_id=None):
        super().__init__(layer_cn_id, ros2_domain_id)
        self.client = docker.from_env()
        self.kpi_metrics = []
        self.active_container_id = ''
        self.container_ids = []

    
    def _load_my_config(self):
        """Load monitor-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                # Extract processing frequency from plugin parameters if available
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if 'docker_monitor' in plugin.get('id'):
                        plugin_params = plugin.get('params', {})
                        self.container_ids = plugin_params.get("container_ids", [])
                        self.get_logger().info(f"Loaded docker monitor config: {self.container_ids}")
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load docker monitor config: ", e)

    def start_monitor_srv_callback(self, request, response):
        self.active_container_id = request.container_id
        self.monitor_timer = self.create_timer(
            2.0, lambda: self.collect_kpi(self.active_container_id)
        )
        response.success = True
        response.container_id = self.active_container_id
        response.hardware_id = "host_001"
        response.start_time = self.get_clock().now().to_msg()
        return response

    def stop_monitor_srv_callback(self, request, response):
        self.log_kpi(self.kpi_metrics)
        if self.monitor_timer:
            self.monitor_timer.destroy()
            # self.destroy_timer(self.monitor_timer)
            self.monitor_timer = None
        response.success = True
        response.container_id = self.active_container_id
        response.hardware_id = "host_001"
        response.results_file_path = "log/monitor.log"
        # TODO: compute duration
        return response

    def collect_kpi(self, object_id=None):
        # Example dummy metrics
        self.kpi_metrics.append({'cpu': 0.25, 'mem': 231.0})

    def register_pub_sub(self):
        pass

    def urgent_stop(self):
        self.stop_component()
        self.set_state("disconnected")

    def check_plugins(self):
        self.set_state("ready")

    def register_timer(self):
        pass