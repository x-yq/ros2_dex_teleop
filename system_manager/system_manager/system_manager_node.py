#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import yaml
import os
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from system_interfaces.srv import (
    Logging, GetConfig, SystemControl
)
from system_checkup import SystemCheckup
from system_state_machine import SystemStateMachine

'''
TODO: 
1. system_ctl srv req expand: start and stop monitor
2. how to call the node's urgent stop and checkup?
'''

class SystemManager(Node):
    """Main system management node"""
    
    def __init__(self, config_folder_path="/root/dev_ws/src/config_files"):
        super().__init__('system_manager')
        
        # File paths for logging
        self.system_log_file = "/tmp/system.log"
        self.monitor_log_file = "/tmp/monitor.log"
        
        # Initialize configuration storage
        self.config_folder_path = config_folder_path
        self._initialize_configs()
        
        # Initialize core components
        self.state_machine = SystemStateMachine()
        self.system_checkup = SystemCheckup(logger=self.get_logger())
        
        # Create services
        self._create_services()
        
        # Run initial system checkup
        self._run_startup_checkup()
        
        self.get_logger().info('SystemManager node started successfully')
        
    def _initialize_configs(self):
        """Parse and initialize configuration dictionaries with default values"""
        self.data_stream_config = self.parse_config_file(os.path.join(self.config_folder_path,"data_stream.yaml"))
            
        self.container_config = self.parse_config_file(os.path.join(self.config_folder_path,"container.yaml"))
        
        self.hardware_config = self.parse_config_file(os.path.join(self.config_folder_path,"hardware.yaml"))
        
        self.ros2_config = self.parse_config_file(os.path.join(self.config_folder_path,"ros2.yaml"))
        
        self.demo_config = self.parse_config_file(os.path.join(self.config_folder_path,"demo.yaml"))

        ## TODO: merge plugin configs into one dict
        self.plugin_config = {}
        # self.plugin_config = self.parse_config_file(os.path.join(self.config_folder_path,".yaml"))

        
    def _create_services(self):
        """Create all ROS2 services provided by SystemManager"""
        self.logging_srv = self.create_service(
            Logging, 'logging', self.logging_srv_callback)
            
        self.get_config_srv = self.create_service(
            GetConfig, 'get_config', self.get_config_srv_callback)
            
        self.system_ctl_srv = self.create_service(
            SystemControl, 'system_control', self.system_ctl_srv_callback)
            
        self.get_logger().info('All services created successfully')
        
    def _run_startup_checkup(self):
        """Run system checkup during startup"""
        self.get_logger().info('Running startup system checkup...')
        checkup_results = self.system_checkup.run_full_checkup()
        
        if checkup_results['overall_status']:
            self.state_machine.connect()
            self.get_logger().info('Startup checkup passed - System ready')
        else:
            self.state_machine.error()
            self.get_logger().warn('Startup checkup failed - System in error state')
            self.get_logger().warn(f"Errors: {checkup_results['errors']}")
            self.get_logger().warn(f"Warnings: {checkup_results['warnings']}")
        
    def parse_config_file(self, config_file):
        """Parse configuration file (YAML or JSON)"""
        try:
            if not os.path.exists(config_file):
                self.get_logger().warn(f'Config file not found: {config_file}')
                return {}
                
            with open(config_file, 'r') as f:
                if config_file.endswith(('.yaml', '.yml')):
                    return yaml.safe_load(f)
                elif config_file.endswith('.json'):
                    return json.load(f)
                else:
                    self.get_logger().warn(f'Unsupported config file format: {config_file}')
                    return {}
                    
        except Exception as e:
            self.get_logger().error(f'Failed to parse config file {config_file}: {e}')
            return {}
            
    def get_config_srv_callback(self, request, response):
        """Handle configuration retrieval requests"""
        try:
            config_name = request.config_file_name.lower()
            config_keys = request.config_keys
            
            # Map config names to internal configurations
            config_mapping = {
                'data_stream': self.data_stream_config,
                'plugin': self.plugin_config,
                'container': self.container_config,
                'hardware': self.hardware_config,
                'ros2': self.ros2_config,
                'demo': self.demo_config
            }
            
            if config_name in config_mapping:
                config_data = config_mapping[config_name]
                
                # Filter by requested keys if specified
                if config_keys:
                    filtered_data = {k: config_data.get(k) for k in config_keys if k in config_data}
                    config_data = filtered_data
                    
                response.success = True
                response.config_data = json.dumps(config_data, indent=2)
                response.message = f"Configuration '{config_name}' retrieved successfully"
                
                self.get_logger().info(f"Config request fulfilled: {config_name}")
                
            else:
                response.success = False
                response.message = f"Unknown configuration: {config_name}. Available: {list(config_mapping.keys())}"
                response.config_data = "{}"
                
                self.get_logger().warn(f"Unknown config requested: {config_name}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error retrieving configuration: {e}"
            response.config_data = "{}"
            self.get_logger().error(f"Config service error: {e}")
            
        return response
        
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
        except Exception as e:
            self.get_logger().error(f"Failed to write to log file {log_file}: {e}")
            
    def system_ctl_srv_callback(self, request, response):
        """Handle system control commands"""
        try:
            description = request.description.lower()
            parameters = request.parameters
            
            success = False
            message = ""
            
            if description == "run" or description == "start":
                success = self.state_machine.run()
                message = "System started" if success else "Cannot start system from current state"
                
            elif description == "pause":
                success = self.state_machine.pause()
                message = "System paused" if success else "Cannot pause system from current state"
                
            elif description == "stop":
                success = self.state_machine.urgent_stop()
                message = "System stopped"
                
            elif description in ["get system state", "status", "state"]:
                success = True
                message = f"Current system state: {self.state_machine.get_current_system_state()}"
                
            elif description == "checkup":
                checkup_results = self.system_checkup.run_full_checkup()
                success = checkup_results['overall_status']
                message = f"System checkup {'passed' if success else 'failed'}"
                
            elif description == "connect":
                success = self.state_machine.connect()
                message = "System connected" if success else "Cannot connect from current state"
                
            else:
                success = False
                message = f"Unknown command: {description}. Available commands: run, pause, stop, status, checkup, connect"
                
            response.success = success
            response.system_state = self.state_machine.get_current_system_state()
            response.message = message
            response.timestamp = self.get_clock().now().to_msg()
            
            # Log system control action
            self.logging(self.system_log_file, 
                        f"SYSTEM_CONTROL: {description} -> {response.system_state} ({'SUCCESS' if success else 'FAILED'})")
            
            self.get_logger().info(f"System control: {description} -> {response.system_state}")
            
        except Exception as e:
            response.success = False
            response.message = f"System control error: {e}"
            response.system_state = "error"
            response.timestamp = self.get_clock().now().to_msg()
            self.get_logger().error(f"System control error: {e}")
            
        return response
        
    # def get_connected_components(self):
    #     """Return list of currently connected component nodes"""
    #     return self.connected_components.copy()
        
    # def add_connected_component(self, component_id):
    #     """Add a component to the connected list"""
    #     if component_id not in self.connected_components:
    #         self.connected_components.append(component_id)
    #         self.get_logger().info(f"Component connected: {component_id}")
            
    # def remove_connected_component(self, component_id):
    #     """Remove a component from the connected list"""
    #     if component_id in self.connected_components:
    #         self.connected_components.remove(component_id)
    #         self.get_logger().info(f"Component disconnected: {component_id}")
            

def main(args=None):
    """Main entry point for SystemManager node"""
    rclpy.init(args=args)
    
    try:
        node = SystemManager()
        self.get_logger().info("SystemManager node initialized, spinning...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("SystemManager node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"SystemManager node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("SystemManager node shutdown complete")

if __name__ == '__main__':
    main()