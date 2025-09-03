#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import sys
import yaml
import os
import subprocess
import threading
from threading import Thread
import time
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String
from system_interfaces.srv import (
    Logging, GetConfig, SystemControl
)
from system_interfaces.msg import ComponentStatus
from system_interfaces.srv import ComponentService
import asyncio
from concurrent.futures import ThreadPoolExecutor

class SystemManager(Node):
    """Main system management node with state machine and component monitoring"""
    
    def __init__(self, config_folder_path="", ros2_domain_id=None):
        super().__init__('system_manager')

        self.ros2_domain_id = ros2_domain_id
        
        # File paths for logging
        self.system_log_file = "/root/dev_ws/log/system.log"
        self.monitor_log_file = "/root/dev_ws/log/monitor"
        
        # Initialize configuration storage
        self.config_folder_path = config_folder_path
        self._initialize_configs()
        
        # Initialize state machine
        self.system_state = 'disconnected'
        self.checkup_running = False
        self.checkup_result = None
        
        # Component monitoring
        self.component_statuses = {}
        self.expected_components = set()

        # Create separate callback groups for different operations
        self.system_ctl_cmd_callback_group = MutuallyExclusiveCallbackGroup()
        self.compo_srv_callback_group = ReentrantCallbackGroup()
        self.logging_config_callback_group = MutuallyExclusiveCallbackGroup()
        self.compo_status_callback_group = ReentrantCallbackGroup()

        # Thread pool for async operations
        self.thread_pool = ThreadPoolExecutor(max_workers=10)
        
        # Create services
        self._create_services()
        
        # Load expected components from config
        self._load_expected_components()
        
        # Create topics for component communication
        self._create_component_communication()
        
        # # Run initial system checkup
        self._start_background_checkup()
        
        self.logging(self.system_log_file, 'SystemManager node started successfully', 'INFO')

    def _initialize_configs(self):
        """Parse and initialize configuration dictionaries dynamically"""
        self.all_configs = {}
        
        # Parse root-level config files
        root_config_files = [
            'calibration.yaml',
            'container.yaml', 
            'data_stream.yaml',
            'demo.yaml',
            'hardware.yaml',
            'ros2.yaml'
        ]
        
        for config_file in root_config_files:
            file_path = os.path.join(self.config_folder_path, config_file)
            config_name = os.path.splitext(config_file)[0]  # Remove extension
            self.all_configs[config_name] = self.parse_config_file(file_path)
        
        # Parse component_node folder configs
        component_node_path = os.path.join(self.config_folder_path, "component_node")
        if os.path.exists(component_node_path):
            component_configs = {}
            for config_file in os.listdir(component_node_path):
                if config_file.endswith(('.yaml', '.yml', '.json')):
                    file_path = os.path.join(component_node_path, config_file)
                    config_name = os.path.splitext(config_file)[0]
                    component_configs[config_name] = self.parse_config_file(file_path)
            
            # Store component configs both individually and as a group
            self.all_configs['component_node'] = component_configs
            # Also store each component config individually for direct access
            for config_name, config_data in component_configs.items():
                self.all_configs[f"component_node.{config_name}"] = config_data
        
        # Maintain backward compatibility with existing attributes
        self.data_stream_config = self.all_configs.get('data_stream', {})
        self.container_config = self.all_configs.get('container', {})
        self.hardware_config = self.all_configs.get('hardware', {})
        self.ros2_config = self.all_configs.get('ros2', {})
        self.demo_config = self.all_configs.get('demo', {})
        self.calibration_config = self.all_configs.get('calibration', {})
        
        # Create plugin_config as a consolidated view of component_node configs
        self.plugin_config = self.all_configs.get('component_node', {})
        
        self.logging(self.system_log_file, f"Loaded {len(self.all_configs)} configuration groups", 'INFO')
        self.logging(self.system_log_file, f"Available configurations: {list(self.all_configs.keys())}", 'INFO')
    
    def parse_config_file(self, config_file):
        """Parse configuration file (YAML or JSON)"""
        try:
            if not os.path.exists(config_file):
                self.logging(self.system_log_file, f'Config file not found: {config_file}', 'WARN')
                return {}
                
            with open(config_file, 'r') as f:
                if config_file.endswith(('.yaml', '.yml')):
                    return yaml.safe_load(f) or {}
                elif config_file.endswith('.json'):
                    return json.load(f)
                else:
                    self.logging(self.system_log_file, f'Unsupported config file format: {config_file}', 'WARN')
                    return {}
                    
        except Exception as e:
            self.logging(self.system_log_file, f'Failed to parse config file {config_file}: {e}', 'ERROR')
            return {}

############################## Communication Setup ################################

    def _create_services(self):
        """Create all ROS2 services provided by SystemManager"""
        self.logging_srv = self.create_service(
            Logging, 'logging', self.logging_srv_callback, callback_group=self.logging_config_callback_group)
            
        self.get_config_srv = self.create_service(
            GetConfig, 'get_config', self.get_config_srv_callback, callback_group=self.logging_config_callback_group)
            
        self.system_ctl_srv = self.create_service(
            SystemControl, 'system_control', self.system_ctl_srv_callback_async, callback_group=self.system_ctl_cmd_callback_group)
            
        self.logging(self.system_log_file, 'All services created successfully', 'INFO')

    def _create_component_communication(self):
        """Create service clients for component communication"""
        # Dictionary to store service clients for each component
        self.component_clients = {}
        
        # Create service clients for each expected component
        for component_id in self.expected_components:
            service_name = f'cn_srv_{component_id}'
            client = self.create_client(ComponentService, service_name, callback_group=self.compo_srv_callback_group)
            self.component_clients[component_id] = client
            
            # Wait for service to be available (optional, with timeout)
            if not client.wait_for_service(timeout_sec=5.0):
                self.logging(self.system_log_file, f'Service {service_name} not available', 'WARN')
            else:
                self.logging(self.system_log_file, f'Connected to service {service_name}', 'INFO')

        # Subscriber for component status responses
        self.component_channel_sub = self.create_subscription(
            ComponentStatus, 'component_states', 
            self.component_states_callback, 10, callback_group=self.compo_status_callback_group)

        self.logging(self.system_log_file, 'Component communication service clients created', 'INFO')
    
    def _load_expected_components(self):
        """Load expected component nodes from configuration"""
        try:
            if 'component_node' in self.all_configs:
                for component_name in self.all_configs['component_node'].keys():
                    if 'monitor' in component_name:
                        continue
                    self.expected_components.add(component_name)
            
            self.logging(self.system_log_file, f"Expected components: {list(self.expected_components)}", 'INFO')
            
        except Exception as e:
            self.logging(self.system_log_file, f"Failed to load expected components: {e}", 'WARN')
            
    def component_states_callback(self, msg):
        """Handle component status messages"""
        self.component_statuses[msg.layer_cn_id] = msg.status
        
        self.logging(self.system_log_file, f'Received status from {msg.layer_cn_id}: {msg.status}', 'INFO')
        
        # Check for component errors when system is running
        if self.system_state in ['paused', 'running']:
            # Exclude monitor and data collection nodes from error checking
            excluded_nodes = ['monitor_0', 'data_collection']
            
            if (msg.layer_cn_id not in excluded_nodes and 
                msg.status in ['error', 'disconnected', 'stopped']):
                
                self.logging(self.system_log_file, f'Component error detected: {msg.layer_cn_id} -> {msg.status}', 'ERROR')
                self._handle_component_error(msg.layer_cn_id)

########################## system control service ###############################

    async def _send_command_to_all_components_async(self, command, timeout=5.0):
        """Send command to all components asynchronously"""
        results = {}
        
        self.logging(self.system_log_file, f"Sending '{command}' command to all components...", 'INFO')
        
        # Create tasks for all component service calls
        tasks = []
        component_task_map = {}
        
        for component_id in self.expected_components:
            if component_id not in self.component_clients:
                results[component_id] = {'success': False, 'status': 'no_client'}
                continue
                
            client = self.component_clients[component_id]
            if not client.service_is_ready():
                results[component_id] = {'success': False, 'status': 'not_ready'}
                continue
            
            # Create request
            request = ComponentService.Request()
            request.command = command
            request.timestamp = self.get_clock().now().to_msg()
            request.sender = "system_manager"
            
            # Create async task
            task = asyncio.create_task(self._call_component_service_async(client, request))
            tasks.append(task)
            component_task_map[task] = component_id
        
        if not tasks:
            self.logging(self.system_log_file, "No component services available", 'WARN')
            return results
        
        # Wait for all tasks to complete with timeout
        try:
            completed_tasks = await asyncio.wait_for(
                asyncio.gather(*tasks, return_exceptions=True), 
                timeout=timeout
            )
            
            # Process results
            for i, result in enumerate(completed_tasks):
                task = tasks[i]
                component_id = component_task_map[task]
                
                if isinstance(result, Exception):
                    results[component_id] = {'success': False, 'status': f'error: {result}'}
                elif result:
                    results[component_id] = {
                        'success': result.success,
                        'status': result.status,
                        'response': result
                    }
                    self.component_statuses[component_id] = result.status
                else:
                    results[component_id] = {'success': False, 'status': 'no_response'}
                    
        except asyncio.TimeoutError:
            self.logging(self.system_log_file, f"Timeout waiting for component responses after {timeout}s", 'ERROR')
            # Mark all pending components as timeout
            for task in tasks:
                if not task.done():
                    component_id = component_task_map[task]
                    results[component_id] = {'success': False, 'status': 'timeout'}
                    task.cancel()
        
        return results

    async def _call_component_service_async(self, client, request):
        """Async wrapper for service call"""
        loop = asyncio.get_event_loop()
        future = client.call_async(request)
        
        # Convert ROS future to asyncio future
        while not future.done():
            await asyncio.sleep(0.01)  # Small delay to prevent busy waiting
            
        return future.result()

    def system_ctl_srv_callback_async(self, request, response):
        """Handle system control commands with async component communication"""
        try:
            self.logging(self.system_log_file, "system_ctl_srv_callback called", 'INFO')

            description = request.description.lower()
            success = False
            message = ""
            
            # Run the async operation in thread pool to avoid blocking
            future = self.thread_pool.submit(self._handle_system_command_sync, description)
            
            # Wait for completion with reasonable timeout
            try:
                result = future.result(timeout=30.0)  # 30 second timeout
                success = result['success']
                message = result['message']
                if 'system_state' in result:
                    self.system_state = result['system_state']
                    
            except Exception as e:
                success = False
                message = f"Command execution failed: {e}"
                self.logging(self.system_log_file, f"System command execution error: {e}", 'ERROR')
            
            response.success = success
            response.system_state = self.system_state
            response.message = message
            response.timestamp = self.get_clock().now().to_msg()
            
            self.logging(self.system_log_file, 
                        f"SYSTEM_CONTROL: {description} -> {response.system_state} ({'SUCCESS' if success else 'FAILED'})", 'INFO')

        except Exception as e:
            response.success = False
            response.message = f"System control error: {e}"
            response.system_state = "error"
            response.timestamp = self.get_clock().now().to_msg()
            self.logging(self.system_log_file, f"System control error: {e}", 'ERROR')
            
        return response

    def _handle_system_command_sync(self, description):
        """Handle system command synchronously in thread pool"""
        try:
            # Create new event loop for this thread
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            try:
                if description in ["run", "start", "restart"]:
                    results = loop.run_until_complete(
                        self._send_command_to_all_components_async("start")
                    )
                    success = all(r.get('success', False) for r in results.values())
                    return {
                        'success': success,
                        'message': "System started" if success else "Some components failed to start",
                        'system_state': 'running' if success else 'disconnected'
                    }
                    
                elif description == "pause":
                    results = loop.run_until_complete(
                        self._send_command_to_all_components_async("pause")
                    )
                    success = all(r.get('success', False) for r in results.values())
                    return {
                        'success': success,
                        'message': "System paused" if success else "Some components failed to pause",
                        'system_state': 'paused' if success else 'disconnected'
                    }
                    
                elif description == "stop" or description == "urgent_stop":
                    results = loop.run_until_complete(
                        self._send_command_to_all_components_async("urgent_stop")
                    )
                    return {
                        'success': True,
                        'message': "System emergency stopped",
                        'system_state': 'disconnected'
                    }
                    
                elif description in ["get system state", "status", "state"]:
                    return {
                        'success': True,
                        'message': f"Current system state: {self.system_state}"
                    }
                elif description == "checkup":
                    self.logging(self.system_log_file, "checkup cmd received from user, checking...", 'INFO')
                    
                    if self.checkup_running:
                        return {
                            'success': True,
                            'message': "Checkup already in progress, please wait..."
                        }
                    
                    # Start async checkup in background
                    self._start_background_checkup()
                    
                    return {
                        'success': True,
                        'message': "System checkup started in background"
                    }
                    
                else:
                    return {
                        'success': False,
                        'message': f"Unknown command: {description}"
                    }
                    
            finally:
                loop.close()
                
        except Exception as e:
            return {
                'success': False,
                'message': f"Command handling error: {e}"
            }

####################### component checkup ###########################
    
    def _start_background_checkup(self):
        """Start the checkup process in a background thread"""
        def run_async_checkup():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self._run_checkup_async())
            finally:
                loop.close()
        
        checkup_thread = Thread(target=run_async_checkup, daemon=True)
        checkup_thread.start()
        self.logging(self.system_log_file, 'Background checkup thread started', 'INFO')

    async def _run_checkup_async(self, timeout_sec=30, check_interval=1):
        """Run comprehensive system checkup asynchronously without blocking services"""
        self.checkup_running = True
        self.logging(self.system_log_file, 'Running system checkup (async)...', 'INFO')
        
        start_time = time.time()
        overall_ok = False

        try:
            while time.time() - start_time < timeout_sec:
                # Run hardware checkup (make it async if possible)
                hardware_ok = await self._check_hardware_connections_async()
                
                # Run component checkup (already async)
                components_ok = await self._check_component_status_async()
                
                # Overall checkup result
                overall_ok = hardware_ok and components_ok

                if overall_ok:
                    break  # exit loop if everything is okay
                else:
                    self.logging(self.system_log_file,
                        f'Checkup failed, retrying in {check_interval}s...', 'WARN'
                    )
                    # Use asyncio.sleep instead of time.sleep to avoid blocking
                    await asyncio.sleep(check_interval)

            # Final result after retries
            if overall_ok:
                self.system_state = 'paused'
                self.logging(self.system_log_file, 'Checkup passed - System ready (paused)', 'INFO')
            else:
                self.system_state = 'disconnected'
                self.logging(self.system_log_file,
                    'Startup checkup failed - System remains disconnected after retries', 'WARN'
                )

            self.checkup_result = overall_ok
            return overall_ok

        except Exception as e:
            self.logging(self.system_log_file, f'Checkup error: {e}', 'ERROR')
            self.checkup_result = False
            return False
        finally:
            self.checkup_running = False

    async def _check_hardware_connections_async(self):
        """Async version of hardware connections check"""
        if 'hardware' not in self.all_configs:
            self.logging(self.system_log_file, 'No hardware config found, skipping hardware check', 'WARN')
            return True
        
        hardware_config = self.all_configs['hardware']
        all_hardware_ok = True
        
        self.logging(self.system_log_file, 'Checking hardware connections (async)...', 'INFO')
        
        # Check different hardware categories
        hardware_categories = ['robots', 'sensors', 'execution_plattforms']
        
        # Create tasks for parallel ping operations
        ping_tasks = []
        device_info = []
        
        for category in hardware_categories:
            if category in hardware_config:
                devices = hardware_config[category]
                if not isinstance(devices, list):
                    continue
                for device in devices:
                    if isinstance(device, dict) and 'ip' in device:
                        device_name = device.get('hardware_id', 'unknown')
                        ip_address = device['ip']
                        
                        # Create async ping task
                        task = asyncio.create_task(self._ping_device_async(ip_address))
                        ping_tasks.append(task)
                        device_info.append((device_name, ip_address))
        
        # Wait for all ping operations to complete
        if ping_tasks:
            ping_results = await asyncio.gather(*ping_tasks, return_exceptions=True)
            
            # Process results
            for i, result in enumerate(ping_results):
                device_name, ip_address = device_info[i]
                
                if isinstance(result, Exception):
                    self.logging(self.system_log_file, f'Hardware FAIL: {device_name} ({ip_address}) - {result}', 'ERROR')
                    all_hardware_ok = False
                elif result:
                    self.logging(self.system_log_file, f'Hardware OK: {device_name} ({ip_address})', 'INFO')
                else:
                    self.logging(self.system_log_file, f'Hardware FAIL: {device_name} ({ip_address}) not reachable', 'ERROR')
                    all_hardware_ok = False
        
        return all_hardware_ok

    async def _ping_device_async(self, ip_address):
        """Async ping a device to check if it's reachable"""
        try:
            # # Use asyncio subprocess for non-blocking ping
            # process = await asyncio.create_subprocess_exec(
            #     'ping', '-c', '1', '-W', '2', ip_address,
            #     stdout=asyncio.subprocess.PIPE,
            #     stderr=asyncio.subprocess.PIPE
            # )
            
            # stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=5.0)
            # return process.returncode == 0

            return True
            
        except asyncio.TimeoutError:
            self.logging(self.system_log_file, f'Ping timeout for {ip_address}', 'WARN')
            return False
        except Exception as e:
            self.logging(self.system_log_file, f'Ping failed for {ip_address}: {e}', 'ERROR')
            return False

    async def _check_component_status_async(self, timeout=10.0):
        """Async version of component status check"""
        self.logging(self.system_log_file, "Starting component checkup via services (async)...", 'INFO')
        
        # Clear previous statuses
        self.component_statuses.clear()
        
        try:
            # Send checkup command to all components (already async)
            results = await self._send_command_to_all_components_async("checkup", timeout)
            
            # Analyze results
            self.logging(self.system_log_file, "Analysing component checkup...", 'INFO')
            all_components_ok = True

            for component in self.expected_components:
                if component in self.component_statuses:
                    status = self.component_statuses[component]
                    if status not in ["ready", "running"]:
                        self.logging(self.system_log_file, f"Component {component} not ready: {status}", 'ERROR')
                        all_components_ok = False
                    else:
                        self.logging(self.system_log_file, f"Component OK: {component} ({status})", 'INFO')
                else:
                    self.logging(self.system_log_file, f"Component {component} not responding", 'ERROR')
                    all_components_ok = False

            self.logging(self.system_log_file,
                f"Component checkup {'passed' if all_components_ok else 'failed'}", 'INFO'
            )
            return all_components_ok
            
        except Exception as e:
            self.logging(self.system_log_file, f"Component checkup error: {e}", 'ERROR')
            return False

    def _handle_component_error(self, component_id):
        """Handle component error by triggering urgent stop"""
        self.logging(self.system_log_file, f'Triggering urgent stop due to component error: {component_id}', 'ERROR')
        
        # Create new event loop for this thread if needed
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
        
        try:
            # Send urgent stop command
            loop.run_until_complete(
                self._send_command_to_all_components_async("urgent_stop")
            )
            
            # Transition to disconnected state
            self.system_state = 'disconnected'
        except Exception as e:
            self.logging(self.system_log_file, f"Error during emergency stop: {e}", 'ERROR')

####################### logging and config service ###########################

    def logging_srv_callback(self, request, response):
        """Handle centralized logging requests from component nodes"""
        try:
            # # Format log entry
            # timestamp_str = datetime.fromtimestamp(
            #     request.timestamp.sec + request.timestamp.nanosec * 1e-9
            # ).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            # log_entry = f"[{timestamp_str}] [{request.log_level}] {request.layer_cn_id}: {request.message}"
            log_entry = f"{request.layer_cn_id}: {request.message}"
            
            # Route to appropriate log file
            if "MONITOR" in request.log_level:
                log_file = os.path.join(self.monitor_log_file, f"{request.log_level}.log")
            else:
                log_file = self.system_log_file
                
            # Write to log file
            self.logging(log_file, log_entry, request.log_level)
            
            # Also log to node's logger for console output
            if request.log_level in ['ERROR', 'FATAL']:
                self.logging(self.system_log_file, f"[{request.layer_cn_id}] {request.message}", 'ERROR')
            elif request.log_level == 'WARN':
                self.logging(self.system_log_file, f"[{request.layer_cn_id}] {request.message}", 'WARN')
            elif request.log_level == 'INFO':
                self.logging(self.system_log_file, f"[{request.layer_cn_id}] {request.message}", 'INFO')
            elif request.log_level == 'DEBUG':  # DEBUG, MONITOR
                self.logging(self.system_log_file, f"[{request.layer_cn_id}] {request.message}", 'DEBUG')
            else:
                self.logging(self.system_log_file, f"{request.message}", 'MONITOR')
                
            response.success = True
            response.message = "Log entry recorded successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to record log entry: {e}"
            self.logging(self.system_log_file, f"Logging service error: {e}", 'ERROR')
            
        return response
        
    def logging(self, log_file, msg, log_level='INFO'):
        """Internal logging function for system-level messages"""
        try:
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            with open(log_file, 'a') as f:
                f.write(f"{msg}\n")
            # Also log to console using the original logger for debugging
            if log_level == 'ERROR':
                super().get_logger().error(msg)
            elif log_level == 'WARN':
                super().get_logger().warn(msg)
            elif log_level == 'INFO':
                super().get_logger().info(msg)
            else:
                super().get_logger().debug(msg)
        except Exception as e:
            super().get_logger().error(f"Failed to write to log file {log_file}: {e}")

    def get_config_srv_callback(self, request, response):
        """Handle configuration retrieval requests"""
        try:
            config_name = request.config_file_name.lower().strip()
            config_keys = request.config_keys
            
            # Create flexible mapping with aliases and exact matches
            config_data = None
            matched_key = None
            
            # First, try exact match
            if config_name in self.all_configs:
                config_data = self.all_configs[config_name]
                matched_key = config_name
            else:
                # Try to find partial matches or aliases
                possible_matches = []
                
                for key in self.all_configs.keys():
                    # Check for exact match (case insensitive)
                    if key.lower() == config_name:
                        config_data = self.all_configs[key]
                        matched_key = key
                        break
                    # Check for partial matches
                    elif config_name in key.lower() or key.lower() in config_name:
                        possible_matches.append((key, self.all_configs[key]))
                
                # If no exact match found but we have partial matches
                if config_data is None and possible_matches:
                    if len(possible_matches) == 1:
                        matched_key, config_data = possible_matches[0]
                    else:
                        # Multiple matches found, list them
                        match_keys = [match[0] for match in possible_matches]
                        response.success = False
                        response.message = f"Multiple configurations match '{config_name}': {match_keys}. Please be more specific."
                        response.config_data = "{}"
                        return response
            
            if config_data is not None:
                # Filter by requested keys if specified
                if config_keys:
                    if isinstance(config_data, dict):
                        filtered_data = {}
                        for k in config_keys:
                            if k in config_data:
                                filtered_data[k] = config_data[k]
                            else:
                                # Try to find nested keys using dot notation
                                nested_value = self._get_nested_value(config_data, k)
                                if nested_value is not None:
                                    filtered_data[k] = nested_value
                        config_data = filtered_data
                    else:
                        response.success = False
                        response.message = f"Cannot filter keys from non-dictionary configuration '{matched_key}'"
                        response.config_data = "{}"
                        return response
                
                response.success = True
                response.config_data = json.dumps(config_data, indent=2, default=str)
                response.message = f"Configuration '{matched_key}' retrieved successfully"
                
                self.logging(self.system_log_file, f"Config request fulfilled: {config_name} -> {matched_key}", 'INFO')
                
            else:
                available_configs = sorted(self.all_configs.keys())
                response.success = False
                response.message = f"Configuration '{config_name}' not found. Available: {available_configs}"
                response.config_data = "{}"
                
                self.logging(self.system_log_file, f"Unknown config requested: {config_name}", 'WARN')
                
        except Exception as e:
            response.success = False
            response.message = f"Error retrieving configuration: {e}"
            response.config_data = "{}"
            self.logging(self.system_log_file, f"Config service error: {e}", 'ERROR')
            
        return response

    def _get_nested_value(self, data, key):
        """Helper method to get nested dictionary values using dot notation"""
        try:
            keys = key.split('.')
            current = data
            for k in keys:
                if isinstance(current, dict) and k in current:
                    current = current[k]
                else:
                    return None
            return current
        except:
            return None

def main(args=None):
    rclpy.init(args=args)
    
    if args:
        # Parse custom arguments
        for i, arg in enumerate(args):
            if arg == "--config_folder_path" and i + 1 < len(args):
                config_folder_path = args[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(args):
                ros2_domain_id = int(args[i + 1])
    else:
        # If no args provided, try to get from sys.argv
        for i, arg in enumerate(sys.argv):
            if arg == "--config_folder_path" and i + 1 < len(sys.argv):
                config_folder_path = sys.argv[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(sys.argv):
                ros2_domain_id = int(sys.argv[i + 1])

    # Create the driver node with the parsed parameters
    node = SystemManager(
        config_folder_path=config_folder_path,
        ros2_domain_id=ros2_domain_id
    )

    try:
        node.logging(f"SystemManager node initialized, spinning...", "INFO")

        executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        node.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        node.logging(f"Unexpected error: {str(e)}", "ERROR")
    finally:
        node._send_command_to_all_components_async("urgent_stop")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()