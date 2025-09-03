# # location: src/kpi_monitor/kpi_monitor/plugins/docker_monitor.py
# import docker
# import psutil
# from core_bases.kpi_monitor_base import KPIMonitorBase

# '''
# TODO: develope dummy collect_kpi() 
# '''

# class DockerMonitor(KPIMonitorBase):
#     def __init__(self, layer_cn_id, ros2_domain_id=None):
#         super().__init__(layer_cn_id, ros2_domain_id)
#         self.client = docker.from_env()
#         self.kpi_metrics = []
#         self.active_container_id = ''
#         self.container_ids = []

    
#     def _load_my_config(self):
#         """Load monitor-specific configuration using component_node config methods"""
#         try:
#             # Load this component's specific configuration
#             self.my_config = self.load_my_config()
#             if self.my_config and "plugins" in self.my_config:
#                 # Extract processing frequency from plugin parameters if available
#                 plugins = self.my_config["plugins"]
#                 for plugin in plugins:
#                     if 'docker_monitor' in plugin.get('id'):
#                         plugin_params = plugin.get('params', {})
#                         self.container_ids = plugin_params.get("container_ids", [])
#                         self.get_logger().info(f"Loaded docker monitor config: {self.container_ids}")
#                         break
                
#         except Exception as e:
#             self.handle_error(f"Failed to load docker monitor config: ", e)

#     def start_monitor_srv_callback(self, request, response):
#         self.active_container_id = request.container_id
#         self.monitor_timer = self.create_timer(
#             2.0, lambda: self.collect_kpi(self.active_container_id)
#         )
#         response.success = True
#         response.container_id = self.active_container_id
#         response.hardware_id = "host_001"
#         response.start_time = self.get_clock().now().to_msg()
#         return response

#     def stop_monitor_srv_callback(self, request, response):
#         self.log_kpi(self.kpi_metrics)
#         if self.monitor_timer:
#             self.monitor_timer.destroy()
#             # self.destroy_timer(self.monitor_timer)
#             self.monitor_timer = None
#         response.success = True
#         response.container_id = self.active_container_id
#         response.hardware_id = "host_001"
#         response.results_file_path = "log/monitor.log"
#         # TODO: compute duration
#         return response

#     def collect_kpi(self, object_id=None):
#         # Example dummy metrics
#         self.kpi_metrics.append({'cpu': 0.25, 'mem': 231.0})

#     def register_pub_sub(self):
#         pass

#     def urgent_stop(self):
#         self.stop_component()
#         self.set_state("disconnected")

#     def check_plugins(self):
#         self.set_state("ready")

#     def register_timer(self):
#         pass

# location: src/kpi_monitor/kpi_monitor/plugins/docker_monitor.py
# location: src/kpi_monitor/kpi_monitor/plugins/docker_monitor.py
import docker
import psutil
import time
from core_bases.kpi_monitor_base import KPIMonitorBase

class DockerMonitor(KPIMonitorBase):
    def __init__(self, layer_cn_id, ros2_domain_id=None):
        super().__init__(layer_cn_id, ros2_domain_id)
        self.client = docker.from_env()
        self.docker_api = docker.APIClient(base_url='unix://var/run/docker.sock')
        
        # Support multiple containers - use dictionaries keyed by container_id
        self.container_kpi_metrics = {}  # {container_id: [metrics]}
        self.active_container_ids = set()  # Set of currently monitored container IDs
        self.containers = {}  # {container_id: container_object}
        self.stats_streams = {}  # {container_id: stats_stream}
        self.monitor_pids = {}  # {container_id: [pids]}
        self.monitor_timers = {}  # {container_id: timer}

    # def _load_my_config(self):
    #     """Load monitor-specific configuration using component_node config methods"""
    #     try:
    #         # Load this component's specific configuration
    #         self.my_config = self.load_my_config()
    #         if self.my_config and "plugins" in self.my_config:
    #             # Extract processing frequency from plugin parameters if available
    #             plugins = self.my_config["plugins"]
    #             for plugin in plugins:
    #                 if 'docker_monitor' in plugin.get('id'):
    #                     plugin_params = plugin.get('params', {})
    #                     self.container_ids = plugin_params.get("container_ids", [])
    #                     self.get_logger().info(f"Loaded docker monitor config: {self.container_ids}")
    #                     break
                
    #     except Exception as e:
    #         self.handle_error(f"Failed to load docker monitor config: ", e)

    def _initialize_container_monitor(self, container_id):
        """Initialize the monitor for a specific Docker container."""
        try:
            self.containers[container_id] = self.client.containers.get(container_id)
            self.stats_streams[container_id] = self.docker_api.stats(container=container_id, stream=True, decode=True)
            
            # Get monitor process PIDs
            exec_log = self.containers[container_id].exec_run("pgrep -f monit")
            if exec_log.exit_code == 0:
                self.monitor_pids[container_id] = exec_log.output.decode().strip().split()
            else:
                self.monitor_pids[container_id] = []
                
            self.get_logger().info(f"Initialized container monitor for {container_id}, found {len(self.monitor_pids[container_id])} monitor processes")
            
        except Exception as e:
            self.handle_error(f"Failed to initialize container monitor for {container_id}: ", e)
            raise

    def get_container_name(self, container_id):
        """Get the name of the container."""
        try:
            container_info = self.docker_api.containers(size=True, filters={'id': container_id})[0]
            return container_info.get('Names')[0].replace('/', '')
        except Exception as e:
            self.handle_error(f"Failed to get container name for {container_id}: ", e)
            return "unknown"

    def get_cpu_percent(self, container_id):
        """Get raw CPU-related usage values for specific container."""
        try:
            stat = next(self.stats_streams[container_id])
            cpu_stats = stat.get('cpu_stats', {})
            precpu_stats = stat.get('precpu_stats', {})

            cpu_usage_total = cpu_stats['cpu_usage'].get('total_usage', 0)
            pre_cpu_usage_total = precpu_stats['cpu_usage'].get('total_usage', 0)
            system_cpu_usage = cpu_stats.get('system_cpu_usage', 0)
            pre_system_cpu_usage = precpu_stats.get('system_cpu_usage', 0)
            online_cpu_count = cpu_stats.get('online_cpus', 1)

            return cpu_usage_total, pre_cpu_usage_total, system_cpu_usage, pre_system_cpu_usage, online_cpu_count
        except Exception as e:
            self.handle_error(f"Failed to get CPU stats for {container_id}: ", e)
            return 0, 0, 0, 0, 1

    def get_memory_usage(self, container_id):
        """Get raw memory usage and cache from memory stats for specific container."""
        try:
            stat = next(self.stats_streams[container_id])
            mem_stats = stat.get("memory_stats", {})

            mem_usage = mem_stats.get("usage", 0)
            
            cache = 0
            if "stats" in mem_stats:
                # cgroup v1
                if "total_inactive_file" in mem_stats["stats"]:
                    cache = mem_stats["stats"]["total_inactive_file"]
                # cgroup v2
                elif "inactive_file" in mem_stats["stats"]:
                    cache = mem_stats["stats"]["inactive_file"]
                # Docker 19.03 -
                elif "cache" in mem_stats["stats"]:
                    cache = mem_stats["stats"]["cache"]
            
            return mem_usage, cache
        except Exception as e:
            self.handle_error(f"Failed to get memory stats for {container_id}: ", e)
            return 0, 0

    def get_disk_usage(self, container_id):
        """Get disk read/write layer and root filesystem sizes in bytes for specific container."""
        try:
            container_info = self.docker_api.containers(size=True, filters={'id': container_id})[0]
            size_rw = container_info.get('SizeRw', 0) or 0
            size_root = container_info.get('SizeRootFs', 0) or 0
            return size_rw, size_root
        except Exception as e:
            self.handle_error(f"Failed to get disk usage for {container_id}: ", e)
            return 0, 0

    def get_monitor_process_stats(self, container_id):
        """Get per-process stats for PIDs related to 'monit' for specific container."""
        if container_id not in self.monitor_pids or not self.monitor_pids[container_id]:
            return []
        
        try:
            cmd = "ps -p {} -o pid=,%cpu=,rss= --no-headers".format(",".join(self.monitor_pids[container_id]))
            result = self.containers[container_id].exec_run(cmd)
            if result.exit_code == 0:
                return [
                    [int(pid), float(cpu), int(rss)*1024] 
                    for line in result.output.decode().splitlines() 
                    for pid, cpu, rss in [line.strip().split(maxsplit=2)]
                    if line.strip()
                ]
        except Exception as e:
            self.handle_error(f"Failed to get monitor process stats for {container_id}: ", e)
        
        return []

    def get_resource_usage(self, container_id):
        """Collects and returns a complete resource usage snapshot for specific container."""
        timestamp = time.time()
        cpu_usage_total, pre_cpu_usage_total, system_cpu_usage, pre_system_cpu_usage, online_cpu_count = self.get_cpu_percent(container_id)
        mem_usage_total, cache = self.get_memory_usage(container_id)
        size_rw, size_root = self.get_disk_usage(container_id)
        monitor_stats = self.get_monitor_process_stats(container_id)

        return {
            "timestamp": timestamp,
            "container_id": container_id,
            "container": {
                "cpu_related": [cpu_usage_total, pre_cpu_usage_total, system_cpu_usage, pre_system_cpu_usage, online_cpu_count],
                "memory_related": [mem_usage_total, cache],
                "disk_related": [size_rw, size_root]
            },
            "monitor_processes": monitor_stats
        }

    def start_monitor_srv_callback(self, request, response):
        """Start monitoring the specified container."""
        container_id = request.container_id
        
        try:
            if container_id in self.active_container_ids:
                self.get_logger().warn(f"Container {container_id} is already being monitored")
                response.success = True
                response.container_id = container_id
                response.start_time = self.get_clock().now().to_msg()
                return response
            
            self.get_logger().info(f"Starting monitor for container: {container_id}")
            
            # Initialize container monitoring
            self._initialize_container_monitor(container_id)
            
            # Initialize metrics storage for this container
            self.container_kpi_metrics[container_id] = []
            
            # Add to active containers
            self.active_container_ids.add(container_id)
            
            # Create timer for collecting KPI metrics (every 1 second)
            self.monitor_timers[container_id] = self.create_timer(
                1.0, lambda cid=container_id: self.collect_kpi(cid)
            )
            
            response.success = True
            response.container_id = container_id
            response.start_time = self.get_clock().now().to_msg()
            
            self.get_logger().info(f"Successfully started monitoring container {container_id}")
            
        except Exception as e:
            self.handle_error(f"Failed to start monitoring for {container_id}: ", e)
            response.success = False
            response.container_id = container_id
            
        return response

    def stop_monitor_srv_callback(self, request, response):
        """Stop monitoring and process/log the collected metrics for the specified container."""
        container_id = request.container_id
        
        try:
            if container_id not in self.active_container_ids:
                self.get_logger().warn(f"Container {container_id} is not being monitored")
                response.success = False
                response.container_id = container_id
                response.results_file_path = f"/log/monitor_{self.layer_cn_id}.log"
                return response
                
            self.get_logger().info(f"Stopping monitor for container: {container_id}")
            
            # Stop the timer for this container
            if container_id in self.monitor_timers:
                self.monitor_timers[container_id].destroy()
                del self.monitor_timers[container_id]
            
            # Process and log the collected metrics for this container
            if container_id in self.container_kpi_metrics and self.container_kpi_metrics[container_id]:
                self._process_and_log_metrics(container_id)
            
            # Clean up resources for this container
            if container_id in self.stats_streams:
                try:
                    self.stats_streams[container_id].close()
                except:
                    pass
                del self.stats_streams[container_id]
            
            # Remove from active containers and clean up
            self.active_container_ids.discard(container_id)
            if container_id in self.containers:
                del self.containers[container_id]
            if container_id in self.monitor_pids:
                del self.monitor_pids[container_id]
            if container_id in self.container_kpi_metrics:
                del self.container_kpi_metrics[container_id]
            
            response.success = True
            response.container_id = container_id
            response.results_file_path = f"log/monitor_{container_id}.log"
            
            self.get_logger().info(f"Successfully stopped monitoring container {container_id}")
            
        except Exception as e:
            self.handle_error(f"Failed to stop monitoring for {container_id}: ", e)
            response.success = False
            
        return response

    def _process_and_log_metrics(self, container_id):
        """Process collected metrics, compute CPU percentages and log the data for specific container."""
        try:
            if container_id not in self.container_kpi_metrics or not self.container_kpi_metrics[container_id]:
                return
                
            # Log header
            header = f"Container: {container_id} ({self.get_container_name(container_id)})"
            self.log_kpi(header)
            
            # Process each metric and compute CPU percentage
            for i, metric in enumerate(self.container_kpi_metrics[container_id]):
                cpu_related = metric["container"]["cpu_related"]
                memory_related = metric["container"]["memory_related"]
                
                # Extract raw CPU values
                cpu_usage_total = cpu_related[0]
                pre_cpu_usage_total = cpu_related[1]
                system_cpu_usage = cpu_related[2]
                pre_system_cpu_usage = cpu_related[3]
                online_cpu_count = cpu_related[4]

                # TODO: Extract Monitor process values
                
                # Compute CPU percentage
                cpu_percent = 0.0
                try:
                    cpu_delta = cpu_usage_total - pre_cpu_usage_total
                    system_delta = system_cpu_usage - pre_system_cpu_usage
                    if system_delta > 0 and cpu_delta > 0:
                        cpu_percent = (cpu_delta / system_delta) * 100 * online_cpu_count
                except Exception as e:
                    self.get_logger().error(f"Error computing CPU % for container {container_id}, sample {i}: {e}")
                
                # Convert memory from bytes to MiB
                memory_usage_mib = memory_related[0] / (1024 * 1024)
                cache_mib = memory_related[1] / (1024 * 1024)
                
                # Create log message with timestamp, CPU percentage, and memory in MiB
                timestamp = metric["timestamp"]
                log_msg = f"Timestamp: {timestamp:.4f}, CPU: {cpu_percent:.2f}%, Memory: {memory_usage_mib:.2f} MiB, Cache: {cache_mib:.2f} MiB"
                
                # Log the processed data
                self.log_kpi(log_msg)
            
            # Log summary
            if len(self.container_kpi_metrics[container_id]) > 0:
                # Compute averages for summary
                total_cpu = 0
                total_memory = 0
                valid_samples = 0
                
                for metric in self.container_kpi_metrics[container_id]:
                    cpu_related = metric["container"]["cpu_related"]
                    memory_related = metric["container"]["memory_related"]
                    
                    cpu_usage_total = cpu_related[0]
                    pre_cpu_usage_total = cpu_related[1]
                    system_cpu_usage = cpu_related[2]
                    pre_system_cpu_usage = cpu_related[3]
                    online_cpu_count = cpu_related[4]
                    
                    try:
                        cpu_delta = cpu_usage_total - pre_cpu_usage_total
                        system_delta = system_cpu_usage - pre_system_cpu_usage
                        if system_delta > 0 and cpu_delta > 0:
                            cpu_percent = (cpu_delta / system_delta) * 100 * online_cpu_count
                            total_cpu += cpu_percent
                            total_memory += memory_related[0] / (1024 * 1024)  # Convert to MiB
                            total_cache += memory_related[1] / (1024 * 1024)
                            valid_samples += 1
                    except:
                        continue
                
                if valid_samples > 0:
                    avg_cpu = total_cpu / valid_samples
                    avg_memory = total_memory / valid_samples
                    avg_cache = total_cache / valid_samples
                    summary_msg = f"Summary - Samples: {valid_samples}, Avg CPU: {avg_cpu:.2f}%, Avg Memory: {avg_memory:.2f} MiB, Avg Cache: {avg_cache:.2f} MiB"
                    self.log_kpi(summary_msg)
            
        except Exception as e:
            self.handle_error(f"Failed to process metrics for container {container_id}: ", e)

    def collect_kpi(self, container_id):
        """Collect KPI metrics for the specified container."""
        try:
            if container_id not in self.active_container_ids or container_id not in self.stats_streams:
                return
                
            # Get resource usage data
            data = self.get_resource_usage(container_id)
            
            # Store the raw data for later processing
            if container_id not in self.container_kpi_metrics:
                self.container_kpi_metrics[container_id] = []
            self.container_kpi_metrics[container_id].append(data)
            
        except Exception as e:
            self.handle_error(f"Failed to collect KPI metrics for container {container_id}: ", e)

    def stop_all_monitoring(self):
        """Stop monitoring for all active containers."""
        container_ids_to_stop = list(self.active_container_ids)
        for container_id in container_ids_to_stop:
            try:
                self.get_logger().info(f"Stopping monitoring for container {container_id}")
                
                # Stop the timer
                if container_id in self.monitor_timers:
                    self.monitor_timers[container_id].destroy()
                    del self.monitor_timers[container_id]
                
                # Process and log metrics
                if container_id in self.container_kpi_metrics and self.container_kpi_metrics[container_id]:
                    self._process_and_log_metrics(container_id)
                
                # Clean up resources
                if container_id in self.stats_streams:
                    try:
                        self.stats_streams[container_id].close()
                    except:
                        pass
                    del self.stats_streams[container_id]
                
            except Exception as e:
                self.handle_error(f"Error stopping monitoring for {container_id}: ", e)
        
        # Clear all tracking data
        self.active_container_ids.clear()
        self.containers.clear()
        self.monitor_pids.clear()
        self.container_kpi_metrics.clear()
        self.monitor_timers.clear()

    def register_pub_sub(self):
        pass

    def urgent_stop(self):
        """Emergency stop of the component."""
        self.stop_all_monitoring()
        self.stop_component()
        self.set_state("disconnected")

    def check_plugins(self):
        self.set_state("ready")

    def register_timer(self):
        pass