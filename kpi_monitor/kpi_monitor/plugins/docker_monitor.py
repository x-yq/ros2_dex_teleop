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

    def start_monitor_srv_callback(self, request, response):
        container_id = request.container_id
        self.monitor_timer = self.create_timer(
            2.0, lambda: self.collect_kpi(container_id)
        )
        response.success = True
        response.container_id = container_id
        response.hardware_id = "host_001"
        response.start_time = self.get_clock().now().to_msg()
        return response

    def stop_monitor_srv_callback(self, request, response):
        self.log_kpi("docker_monitor.log", self.kpi_metrics)
        if self.monitor_timer:
            self.destroy_timer(self.monitor_timer)
            self.monitor_timer = None
        response.success = True
        response.container_id = self.monitored_container_id
        response.hardware_id = "host_001"
        # TODO: compute duration
        return response

    def collect_kpi(self, object_id=None):
        # Example dummy metrics
        self.kpi_metrics.append({'cpu': 0.45, 'mem': 230.0})
        self.kpi_metrics.append({'cpu': 0.25, 'mem': 231.0})
