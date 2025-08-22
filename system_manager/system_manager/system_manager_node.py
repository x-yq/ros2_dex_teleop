#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import yaml
import os
import subprocess
import threading
import time
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String
from system_interfaces.srv import (
    Logging, GetConfig, SystemControl
)
from system_interfaces.msg import ComponentStatus, SystemCommand

class SystemStateMachine:
    """System state machine managing system states"""
    
    def __init__(self, logger):
        self.logger = logger
        self.current_state = "disconnected"
        self.valid_states = ["disconnected", "paused", "running"]
        self.valid_transitions = {
            "disconnected": ["paused"],
            "paused": ["running", "paused","disconnected"],
            "running": ["paused"]
        }
    
    def get_current_state(self):
        return self.current_state
    
    def can_transition_to(self, new_state):
        return new_state in self.valid_transitions.get(self.current_state, [])
    
    def transition_to(self, new_state, reason=""):
        if new_state not in self.valid_states:
            self.logger.error(f"Invalid state: {new_state}")
            return False
        
        if not self.can_transition_to(new_state):
            self.logger.warn(f"Cannot transition from {self.current_state} to {new_state}")
            return False
        
        old_state = self.current_state
        self.current_state = new_state
        self.logger.info(f"State transition: {old_state} -> {new_state}. Reason: {reason}")
        return True
    
    def connect(self):
        """Transition to paused state after successful checkup"""
        return self.transition_to("paused", "System checkup passed")
    
    def run(self):
        """Start the system (paused -> running)"""
        return self.transition_to("running", "User requested system start")
    
    def pause(self):
        """Pause the system (running -> paused)"""
        return self.transition_to("paused", "User requested system pause")
    
    def urgent_stop(self):
        """Emergency stop - force to disconnected state"""
        old_state = self.current_state
        self.current_state = "disconnected"
        self.logger.warn(f"URGENT STOP: {old_state} -> disconnected")
        return True
    
    def error_occurred(self):
        """Handle component error - force to disconnected state"""
        return self.urgent_stop()

class SystemManager(Node):
    """Main system management node with state machine and component monitoring"""
    
    def __init__(self, config_folder_path="/root/dev_ws/src/config_files"):
        super().__init__('system_manager')
        
        # File paths for logging
        self.system_log_file = "/root/dev_ws/log/system.log"
        self.monitor_log_file = "/root/dev_ws/log/monitor.log"
        
        # Initialize configuration storage
        self.config_folder_path = config_folder_path
        self._initialize_configs()
        
        # Initialize state machine
        self.state_machine = SystemStateMachine(self.get_logger())
        
        # Component monitoring
        self.component_statuses = {}
        self.expected_components = set()
        self.status_check_timeout = 10.0  # seconds
        self.status_check_timer = None
        self.awaiting_status_responses = False
        
        # Create services
        self._create_services()
        
        # Create topics for component communication
        self._create_component_communication()
        
        # Load expected components from config
        self._load_expected_components()
        
        # Run initial system checkup
        self._run_startup_checkup()
        
        # Start periodic monitoring
        # self._start_monitoring()
        
        self.get_logger().info('SystemManager node started successfully')

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
        
        self.get_logger().info(f"Loaded {len(self.all_configs)} configuration groups")
        self.get_logger().info(f"Available configurations: {list(self.all_configs.keys())}")
    
    def parse_config_file(self, config_file):
        """Parse configuration file (YAML or JSON)"""
        try:
            if not os.path.exists(config_file):
                self.get_logger().warn(f'Config file not found: {config_file}')
                return {}
                
            with open(config_file, 'r') as f:
                if config_file.endswith(('.yaml', '.yml')):
                    return yaml.safe_load(f) or {}
                elif config_file.endswith('.json'):
                    return json.load(f)
                else:
                    self.get_logger().warn(f'Unsupported config file format: {config_file}')
                    return {}
                    
        except Exception as e:
            self.get_logger().error(f'Failed to parse config file {config_file}: {e}')
            return {}


    def _create_services(self):
        """Create all ROS2 services provided by SystemManager"""
        self.logging_srv = self.create_service(
            Logging, 'logging', self.logging_srv_callback)
            
        self.get_config_srv = self.create_service(
            GetConfig, 'get_config', self.get_config_srv_callback)
            
        self.system_ctl_srv = self.create_service(
            SystemControl, 'system_control', self.system_ctl_srv_callback)
            
        self.get_logger().info('All services created successfully')
    
    def _create_component_communication(self):
        """Create topics for component communication"""
        # Publisher for system commands (to all components)
        self.system_channel_pub = self.create_publisher(
            SystemCommand, 'system_channel', 10)
        
        # Subscriber for component status responses
        self.component_channel_sub = self.create_subscription(
            ComponentStatus, 'component_channel', 
            self.component_status_callback, 10)
        
        self.get_logger().info('Component communication topics created')
    
    def _load_expected_components(self):
        """Load expected component nodes from configuration"""
        try:

            if 'component_node' in self.all_configs:
                for component_name in self.all_configs['component_node'].keys():
                    if 'monitor' in component_name:
                        continue
                    self.expected_components.add(component_name)
            
            self.get_logger().info(f"Expected components: {list(self.expected_components)}")
            self.logging(self.system_log_file,f"Expected components: {list(self.expected_components)}")

            
        except Exception as e:
            self.get_logger().warn(f"Failed to load expected components: {e}")
    
    def _run_startup_checkup(self, timeout_sec=30, check_interval=1):
        """Run comprehensive system checkup during startup with retry until timeout"""
        self.get_logger().info('Running startup system checkup...')
        
        start_time = time.time()
        overall_ok = False

        while time.time() - start_time < timeout_sec:
            # Run hardware checkup
            hardware_ok = self._check_hardware_connections()
            
            # Run component checkup
            components_ok = self._check_component_status()
            
            # Overall checkup result
            overall_ok = hardware_ok and components_ok

            if overall_ok:
                break  # exit loop if everything is okay
            else:
                self.get_logger().warn(
                    f'Startup checkup failed, retrying in {check_interval}s...'
                )
                time.sleep(check_interval)

        # Final result after retries
        if overall_ok:
            success = self.state_machine.connect()
            if success:
                self.get_logger().info('Startup checkup passed - System ready (paused)')
            else:
                self.get_logger().error('Failed to transition to paused state')
        else:
            self.state_machine.error_occurred()
            self.get_logger().warn(
                'Startup checkup failed - System remains disconnected after retries'
            )
    
    def _check_hardware_connections(self):
        """Check hardware connections using ping"""
        if 'hardware' not in self.all_configs:
            self.get_logger().warn('No hardware config found, skipping hardware check')
            return True
        
        hardware_config = self.all_configs['hardware']
        all_hardware_ok = True
        
        self.get_logger().info('Checking hardware connections...')
        
        # Check different hardware categories
        hardware_categories = ['robots', 'sensors', 'execution_plattforms']
        
        for category in hardware_categories:
            if category in hardware_config:
                devices = hardware_config[category]
                if not isinstance(devices, list):
                    continue
                for device in devices:
                    if isinstance(device, dict) and 'ip' in device:
                        device_name = device.get('hardware_id', 'unknown')
                        ip_address = device['ip']
                        
                        if self._ping_device(ip_address):
                            self.get_logger().info(f'Hardware OK: {device_name} ({ip_address})')
                        else:
                            self.get_logger().error(f'Hardware FAIL: {device_name} ({ip_address}) not reachable')
                            all_hardware_ok = False
        
        return all_hardware_ok  
    
    def _ping_device(self, ip_address):
        """Ping a device to check if it's reachable"""
        try:
            # # Use ping command (works on both Linux and Windows)
            # result = subprocess.run(
            #     ['ping', '-c', '3', '-W', '3', ip_address], 
            #     capture_output=True, 
            #     text=True, 
            #     timeout=10
            # )
            # return result.returncode == 0
            ## NOTE: Simulate the result
            return True
        except Exception as e:
            self.get_logger().error(f'Ping failed for {ip_address}: {e}')
            return False

    # def _check_component_status(self):
    #     """Check status of all component nodes"""
    #     if not self.expected_components:
    #         self.get_logger().warn('No expected components defined')
    #         return True
        
    #     self.get_logger().info('Checking component node status...')
        
    #     # Clear previous status
    #     self.awaiting_status_responses = True
    #     self.component_statuses.clear()

        
    #     # Send get status command
    #     cmd_msg = SystemCommand()
    #     cmd_msg.command = "get_status"
    #     cmd_msg.timestamp = self.get_clock().now().to_msg()
    #     cmd_msg.sender = "system_manager"
        
    #     self.system_channel_pub.publish(cmd_msg)
        
    #     # Wait for responses
    #     timeout = 15.0
    #     start_time = time.time()
        
    #     while (time.time() - start_time) < timeout:
    #         rclpy.spin_once(self, timeout_sec=0.1)
            
    #         # Check if we got responses from all expected components
    #         if len(self.component_statuses) >= len(self.expected_components):
    #             break
        
    #     self.awaiting_status_responses = False
        
    #     # Analyze results
    #     all_components_ok = True
    #     for component in self.expected_components:
    #         if component in self.component_statuses:
    #             status = self.component_statuses[component]
    #             if status not in ['ready', 'running', 'paused']:
    #                 self.get_logger().error(f'Component {component} not ready: {status}')
    #                 all_components_ok = False
    #             else:
    #                 self.get_logger().info(f'Component OK: {component} ({status})')
    #         else:
    #             self.get_logger().error(f'Component {component} not responding')
    #             all_components_ok = False

    #     # all_components_ok = True
    #     return all_components_ok


    def _check_component_status(self, timeout=10.0):
        self.awaiting_status_responses = True
        self.component_statuses.clear()

        cmd_msg = SystemCommand()
        cmd_msg.command = "get_status"
        cmd_msg.timestamp = self.get_clock().now().to_msg()
        cmd_msg.sender = "system_manager"
        self.system_channel_pub.publish(cmd_msg)

        # set a timer that will stop waiting
        self.checkup_done = False
        self.create_timer(timeout, self._finish_checkup)
        
        # return immediately (don’t block here)
        return True
    
    def _finish_checkup(self):
        """Finalize checkup after timeout and analyze results"""
        if not self.awaiting_status_responses:
            return  # nothing to do

        self.awaiting_status_responses = False
        self.get_logger().info("Finishing component checkup...")

        all_components_ok = True

        for component in self.expected_components:
            if component in self.component_statuses:
                status = self.component_statuses[component]
                if status not in ["ready", "running", "paused"]:
                    self.get_logger().error(f"Component {component} not ready: {status}")
                    all_components_ok = False
                else:
                    self.get_logger().info(f"Component OK: {component} ({status})")
            else:
                self.get_logger().error(f"Component {component} not responding")
                all_components_ok = False

        # Save result for later retrieval
        self.last_checkup_result = all_components_ok
        self.get_logger().info(
            f"System checkup {'passed' if all_components_ok else 'failed'}"
        )

    def component_status_callback(self, msg):
        """Handle component status messages"""
        self.component_statuses[msg.layer_cn_id] = msg.status
        
        self.get_logger().debug(f'Received status from {msg.layer_cn_id}: {msg.status}')
        self.logging(self.system_log_file, f'Received status from {msg.layer_cn_id}: {msg.status}')
        
        # Check for component errors when system is running
        if self.state_machine.get_current_state() in ['paused', 'running']:
            # Exclude monitor and data collection nodes from error checking
            excluded_nodes = ['monitor_0', 'data_collection']
            
            if (msg.layer_cn_id not in excluded_nodes and 
                msg.status in ['error', 'disconnected', 'stopped']):
                
                self.logging(self.system_log_file, f'Component error detected: {msg.layer_cn_id} -> {msg.status}')
                self._handle_component_error(msg.layer_cn_id)
    
    def _handle_component_error(self, component_id):
        """Handle component error by triggering urgent stop"""
        self.get_logger().error(f'Triggering urgent stop due to component error: {component_id}')
        
        # Send urgent stop command
        self._send_urgent_stop()
        
        # Transition to disconnected state
        self.state_machine.error_occurred()
    
    def _send_start(self):
        """Send pause to all components"""
        cmd_msg = SystemCommand()
        cmd_msg.command = "start"
        cmd_msg.timestamp = self.get_clock().now().to_msg()
        cmd_msg.sender = "system_manager"
        
        self.system_channel_pub.publish(cmd_msg)
        self.get_logger().warn('START command sent to all components')

    def _send_pause(self):
        """Send pause to all components"""
        cmd_msg = SystemCommand()
        cmd_msg.command = "pause"
        cmd_msg.timestamp = self.get_clock().now().to_msg()
        cmd_msg.sender = "system_manager"
        
        self.system_channel_pub.publish(cmd_msg)
        self.get_logger().warn('PAUSE command sent to all components')
        self.logging(self.system_log_file, 'PAUSE command sent to all components')

    def _send_urgent_stop(self):
        """Send urgent stop command to all components"""
        cmd_msg = SystemCommand()
        cmd_msg.command = "urgent_stop"
        cmd_msg.timestamp = self.get_clock().now().to_msg()
        cmd_msg.sender = "system_manager"
        
        self.system_channel_pub.publish(cmd_msg)
        self.get_logger().warn('URGENT STOP command sent to all components')
        self.logging(self.system_log_file, 'URGENT STOP command sent to all components')
        
        # Wait for stop confirmations
        self._wait_for_stop_confirmations()
    
    def _wait_for_stop_confirmations(self):
        """Wait for components to confirm they have stopped"""
        self.component_statuses.clear()
        timeout = 10.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check if all expected components have stopped
            stopped_components = sum(1 for status in self.component_statuses.values() 
                                   if status == 'stopped')
            
            if stopped_components >= len(self.expected_components):
                self.get_logger().info('All components confirmed stop')
                break
        
        # Log any components that didn't respond
        for component in self.expected_components:
            if component not in self.component_statuses:
                self.get_logger().warn(f'Component {component} did not confirm stop')
    
    def _start_monitoring(self):
        """Start periodic monitoring of system and components"""
        # Create timer for periodic status checks (every 60 seconds when running)
        self.monitoring_timer = self.create_timer(60.0, self._periodic_monitoring)
    
    def _periodic_monitoring(self):
        """Periodic monitoring callback"""
        current_state = self.state_machine.get_current_state()
        
        if current_state in ['paused', 'running']:
            # Only monitor when system is active
            self.get_logger().debug('Running periodic component status check')
            self._check_component_status()
    
    def logging_srv_callback(self, request, response):
        """Handle centralized logging requests from component nodes"""
        try:
            # Format log entry
            timestamp_str = datetime.fromtimestamp(
                request.timestamp.sec + request.timestamp.nanosec * 1e-9
            ).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            
            log_entry = f"[{timestamp_str}] [{request.log_level}] {request.layer_cn_id}: {request.message}"
            
            # Route to appropriate log file
            if request.log_level == "MONITOR":
                log_file = self.monitor_log_file
            else:
                log_file = self.system_log_file
                
            # Write to log file
            self.logging(log_file, log_entry)
            
            # Also log to node's logger for console output
            if request.log_level in ['ERROR', 'FATAL']:
                self.get_logger().error(f"[{request.layer_cn_id}] {request.message}")
            elif request.log_level == 'WARN':
                self.get_logger().warn(f"[{request.layer_cn_id}] {request.message}")
            elif request.log_level == 'INFO':
                self.get_logger().info(f"[{request.layer_cn_id}] {request.message}")
            else:  # DEBUG, MONITOR
                self.get_logger().debug(f"[{request.layer_cn_id}] {request.message}")
                
            response.success = True
            response.message = "Log entry recorded successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to record log entry: {e}"
            self.get_logger().error(f"Logging service error: {e}")
            
        return response
        
    def logging(self, log_file, msg):
        """Internal logging function for system-level messages"""
        try:
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            with open(log_file, 'a') as f:
                f.write(f"{msg}\n")
                self.get_logger().info(f"{msg}\n")
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file {log_file}: {e}")

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
                
                self.get_logger().info(f"Config request fulfilled: {config_name} -> {matched_key}")
                
            else:
                available_configs = sorted(self.all_configs.keys())
                response.success = False
                response.message = f"Configuration '{config_name}' not found. Available: {available_configs}"
                response.config_data = "{}"
                
                self.get_logger().warn(f"Unknown config requested: {config_name}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error retrieving configuration: {e}"
            response.config_data = "{}"
            self.get_logger().error(f"Config service error: {e}")
            
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
            
    def system_ctl_srv_callback(self, request, response):
        """Handle system control commands"""
        try:
            description = request.description.lower()
            
            success = False
            message = ""
            
            if description in ["run", "start", "restart"]:
                # if self.state_machine.get_current_state() == "paused":
                self._send_start()
                success = self.state_machine.run()
                message = "System started" if success else "Cannot start system from current state"
                # else:
                #     message = f"Cannot start from {self.state_machine.get_current_state()} state. System must be in paused state."
                
            elif description == "pause":
                # if self.state_machine.get_current_state() == "running":
                self._send_pause()
                success = self.state_machine.pause()
                message = "System paused" if success else "Cannot pause system from current state"
                # else:
                #     message = f"Cannot pause from {self.state_machine.get_current_state()} state. System must be running."
                
            elif description == "stop" or description == "urgent_stop":
                self._send_urgent_stop()
                success = self.state_machine.urgent_stop()
                message = "System emergency stopped"
                
            elif description in ["get system state", "status", "state"]:
                success = True
                message = f"Current system state: {self.state_machine.get_current_state()}"
                
            elif description == "checkup":
                self.logging("checkup cmd received from user, checking...", "WARN")
                hardware_ok = self._check_hardware_connections()
                components_ok = self._check_component_status()
                success = hardware_ok and components_ok
                message = f"System checkup {'passed' if success else 'failed'}"

            else:
                success = False
                message = f"Unknown command: {description}. Available commands: run, pause, urgent_stop, status, checkup"
                
            response.success = success
            response.system_state = self.state_machine.get_current_state()
            response.message = message
            response.timestamp = self.get_clock().now().to_msg()
            
            # Log system control action
            self.logging(self.system_log_file, 
                        f"SYSTEM_CONTROL: {description} -> {response.system_state} ({'SUCCESS' if success else 'FAILED'})")

        except Exception as e:
            response.success = False
            response.message = f"System control error: {e}"
            response.system_state = "error"
            response.timestamp = self.get_clock().now().to_msg()
            self.get_logger().error(f"System control error: {e}")
            
        return response

def main(args=None):
    """Main entry point for SystemManager node"""
    rclpy.init(args=args)
    
    try:
        node = SystemManager()
        node.get_logger().info("SystemManager node initialized, spinning...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("SystemManager node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"SystemManager node error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("SystemManager node shutdown complete")

if __name__ == '__main__':
    main()