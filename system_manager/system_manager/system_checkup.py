import json
import yaml
import os
from datetime import datetime

## TODO: add config files checkup, if required files are prepared


class SystemCheckup:
    """System health and connectivity checker"""
    
    def __init__(self, logger=None):
        self.result = {}
        self.logger = logger


    def _load_component_node_configs(self, layer_cn_config_folder):
        """Load all component node configs and extract layer_cn_ids"""
        expected_nodes = []
        
        if not os.path.exists(layer_cn_config_folder):
            self.logger.error(f"Component node config folder not found: {layer_cn_config_folder}")
            return expected_nodes
            
        # Get all yaml/yml files in the config folder
        config_files = [f for f in os.listdir(layer_cn_config_folder) 
                       if f.endswith(('.yaml', '.yml'))]
        
        for config_file in config_files:
            config_path = os.path.join(layer_cn_config_folder, config_file)
            try:
                with open(config_path, 'r') as file:
                    config_data = yaml.safe_load(file)
                    
                    # Extract layer_cn_id from the config
                    if 'layer_cn_id' in config_data:
                        layer_cn_id = config_data['layer_cn_id']
                        expected_nodes.append(layer_cn_id)
                        
                        self.logger.debug(f"Found component node: {layer_cn_id} in {config_file}")
                    else:
                        self.logger.error(f"No layer_cn_id found in {config_file}")
                        
            except Exception as e:
                self.logger.error(f"Failed to load config {config_file}: {e}")
                self.logger.error(f"Error loading {config_file}: {e}")
                    
        return expected_nodes
        
    def _load_hardware_config(self, hardware_config_path):
        """Load hardware configuration from YAML file"""
        hardware_list = []
        
        try:
            with open(hardware_config_path, 'r') as file:
                config_data = yaml.safe_load(file)
                
                if 'hardware' in config_data:
                    hardware_list = config_data['hardware']
                else:
                    self.logger.error("No 'hardware' section found in hardware config")
                    
        except Exception as e:
            self.logger.error(f"Failed to load hardware config {hardware_config_path}: {e}")
            self.logger.error(f"Error loading hardware config: {e}")
                
        return hardware_list
        
    def check_hardware_connection(self, hardware_config_path=None):
        ## NOTE: dummy impl
        """Check hardware connections by pinging IPs and checking ports"""
        
        # Load hardware configuration
        if hardware_config_path:
            hardware_list = self._load_hardware_config(hardware_config_path)
        else:
            self.logger.error(f"Hardware list not found")
            
        hardware_status = {}
        for hw in hardware_list:
            hardware_id = hw.get('hardware_id', 'unknown')
            
            try:
                # Dummy ping implementation - in real scenario use subprocess ping
                # subprocess.run(['ping', '-c', '1', hw['ip']], check=True)
                
                # Check main access port (SSH)
                access_port_open = True  # Dummy check for access_port
                
                # Check additional ports if specified
                additional_ports_status = {}
                if 'ports' in hw:
                    for port_name, port_num in hw['ports'].items():
                        # Dummy port check - in real scenario use socket connection
                        additional_ports_status[port_name] = {
                            'port': port_num,
                            'open': True  # Dummy status
                        }
                
                hardware_status[hardware_id] = {
                    'ip_reachable': True,
                    'access_port_open': access_port_open,
                    'additional_ports': additional_ports_status,
                    'status': 'online'
                }
                
            except Exception as e:
                hardware_status[hardware_id] = {
                    'ip_reachable': False, 
                    'access_port_open': False,
                    'additional_ports': {},
                    'status': 'offline',
                    'error': str(e),
                }
                self.logger.error(f"Hardware {hardware_id} unreachable: {e}")
                
        self.result['hardware'] = hardware_status
        overall_hw_status = all(hw['status'] == 'online' for hw in hardware_status.values())
        
        online_count = sum(1 for hw in hardware_status.values() if hw['status'] == 'online')
        total_count = len(hardware_status)
        self.logger.info(f"Hardware check completed. {online_count}/{total_count} devices online. Overall status: {'PASS' if overall_hw_status else 'FAIL'}")
        
        return overall_hw_status
        
    def check_cn_connection(self, layer_cn_config_folder):
        ## NOTE: dummy impl
        """Check component node connections"""
        
        # Load expected nodes from component node config files
        expected_nodes = self._load_component_node_configs(layer_cn_config_folder)
        
        if not expected_nodes:
            self.logger.error("No component nodes found in config folder")
            return True  # Return True if no nodes expected
            
        # In real implementation, use ros2 node discovery
        # discovered_nodes = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        
        # Dummy implementation - simulate partial discovery
        discovered_nodes = expected_nodes[:max(1, len(expected_nodes)//2)]  # Simulate partial discovery
        node_status = {}
        
        for node in expected_nodes:
            if node in discovered_nodes:
                node_status[node] = {
                    'discovered': True,
                    'responsive': True,  # Could ping node services
                    'status': 'online'
                }
            else:
                node_status[node] = {
                    'discovered': False,
                    'responsive': False,
                    'status': 'missing'
                }
                self.logger.error(f"Expected node {node} not discovered")
                
        self.result['component_nodes'] = node_status
        overall_cn_status = all(node['status'] == 'online' for node in node_status.values())
        
        self.logger.info(f"Component node check completed. Found {len(discovered_nodes)}/{len(expected_nodes)} nodes")
            
        return overall_cn_status
        
    def check_plugin_format(self, plugin_configs=None):
        """Check plugin format validation"""
            
        plugin_status = {}
        for config_file in plugin_configs:
            try:
                # In real scenario, validate against schema
                if os.path.exists(config_file):
                    with open(config_file, 'r') as file:
                        config_data = yaml.safe_load(file)
                        
                        # Basic validation checks
                        required_fields = ['layer_cn_id', 'pkg_name', 'plugins']
                        missing_fields = [field for field in required_fields if field not in config_data]
                        
                        if missing_fields:
                            raise ValueError(f"Missing required fields: {missing_fields}")
                            
                        # Validate plugins structure
                        if 'plugins' in config_data:
                            for plugin in config_data['plugins']:
                                if 'id' not in plugin or 'class_path' not in plugin:
                                    raise ValueError(f"Plugin missing required fields (id, class_path): {plugin}")
                    
                    plugin_status[config_file] = {
                        'format_valid': True,
                        'schema_valid': True,
                        'status': 'valid'
                    }
                else:
                    plugin_status[config_file] = {
                        'format_valid': False,
                        'schema_valid': False,
                        'status': 'file_not_found'
                    }
                    self.logger.error(f"Plugin config file not found: {config_file}")
                    
            except Exception as e:
                plugin_status[config_file] = {
                    'format_valid': False,
                    'schema_valid': False, 
                    'status': 'invalid',
                    'error': str(e)
                }
                self.logger.error(f"Plugin config {config_file} invalid: {e}")
                
        self.result['plugins'] = plugin_status
        overall_plugin_status = all(plugin['status'] in ['valid', 'file_not_found'] for plugin in plugin_status.values())
        
        self.logger.info(f"Plugin format check completed. Status: {'PASS' if overall_plugin_status else 'FAIL'}")
            
        return overall_plugin_status
        
    def run_full_checkup(self, layer_cn_config_folder=None, hardware_config_path=None, plugin_configs=None):
        """Run complete system checkup"""
        
        if layer_cn_config_folder is None:
            layer_cn_config_folder = "config_files/component_node/"
            
        checkup_results = {
            'hardware': self.check_hardware_connection(hardware_config_path),
            'component_nodes': self.check_cn_connection(layer_cn_config_folder),
            'plugins': self.check_plugin_format(plugin_configs),
            'timestamp': datetime.now().isoformat()
        }
        
        checkup_results['overall_status'] = all([
            checkup_results['hardware'],
            checkup_results['component_nodes'], 
            checkup_results['plugins']
        ])
        
        status_msg = "PASS" if checkup_results['overall_status'] else "FAIL"
        self.logger.info(f"Full system checkup completed. Overall status: {status_msg}")
            
        return checkup_results
