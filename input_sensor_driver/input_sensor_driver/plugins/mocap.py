import rclpy
from rclpy.timer import Timer
import json
import yaml
import numpy as np
from typing import Dict, List, Any, Optional
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from component_interfaces.msg import RawHandKeypoint
from core_bases.sensor_driver_base import SensorDriverBase


class RococoDriver(SensorDriverBase):
    """
    ROS2 Driver node for Rococo glove sensor input.
    Captures and processes hand keypoint data from rococo gloves.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Driver specific attributes
        self.freq = 60  # Default frequency
        self.timer: Optional[Timer] = None
        self.sensor_active = False
        self.rococo_connection_status = False
        
        # Load plugin parameters
        self._load_plugin_params()

        self.register_pub_sub()
        
        # Start streaming timer
        self._start_streaming()
        
    def _load_plugin_params(self):
        """Load plugin-specific parameters from configuration."""
        try:
            config_data = self.get_config(f"{self.layer_cn_id}.yaml")
            if config_data:
                if config_data.strip().startswith('{'):
                    config = json.loads(config_data)
                else:
                    config = yaml.safe_load(config_data)
                
                # Extract frequency from plugin params
                plugins = config.get('plugins', [])
                for plugin in plugins:
                    if plugin.get('id') == 'rococo_driver':
                        params = plugin.get('params', {})
                        self.freq = params.get('freq', 60)
                        break
                        
                self.logging(f"Loaded plugin params: freq={self.freq}Hz", "INFO")
            else:
                self.logging("Using default plugin parameters", "WARN")
        except Exception as e:
            self.logging(f"Error loading plugin params: {str(e)}", "ERROR")
            
    def register_pub_sub(self):
        """Register publishers and subscribers based on data stream configuration."""
        try:
            # Get data stream configuration
            config_data = self.get_config("data_stream.yaml")
            if not config_data:
                self.logging("Failed to load data stream configuration", "ERROR")
                return
                
            # Parse configuration
            if config_data.strip().startswith('{'):
                stream_config = json.loads(config_data)
            else:
                stream_config = yaml.safe_load(config_data)
            
            topics = stream_config.get('topics', [])
            
            # Find topics where this node is the publisher
            for topic in topics:
                if topic.get('publisher') == self.layer_cn_id:
                    topic_id = topic.get('id')
                    msg_type = topic.get('msg_type')
                    publish_freq = topic.get('publish_freq', 30)
                    
                    if msg_type == 'component_interfaces/msg/RawHandKeypoint':
                        # Create publisher for raw hand keypoint data
                        publisher = self.create_publisher(
                            RawHandKeypoint,
                            topic_id,
                            10
                        )
                        self.data_publishers[topic_id] = publisher
                        self.logging(f"Created publisher for {topic_id} with type {msg_type}", "INFO")
                    else:
                        self.logging(f"Unsupported message type: {msg_type}", "WARN")
            
            self.logging(f"Registered {len(self.data_publishers)} publishers", "INFO")
            
        except Exception as e:
            self.logging(f"Error in register_pub_sub: {str(e)}", "ERROR")

    def start_sensor(self, sensor_id: str) -> bool:
        """Start the rococo glove sensor."""
        try:
            self.logging(f"Starting rococo sensor: {sensor_id}", "INFO")
            
            # Get sensor connection info
            connection_info = self.get_sensor_ip_port(sensor_id)
            if connection_info:
                ip, port = connection_info
                self.logging(f"Connecting to rococo gloves at {ip}:{port}", "INFO")
                
                # Simulate connection to rococo gloves
                # In real implementation, this would establish UDP/TCP connection
                self.rococo_connection_status = True
                self.sensor_active = True
                
                self.logging(f"Rococo sensor {sensor_id} started successfully", "INFO")
                return True
            else:
                self.logging(f"No connection info found for sensor: {sensor_id}", "ERROR")
                return False
                
        except Exception as e:
            self.logging(f"Error starting sensor {sensor_id}: {str(e)}", "ERROR")
            return False

    def stop_sensor(self, sensor_id: str) -> bool:
        """Stop the rococo glove sensor."""
        try:
            self.logging(f"Stopping rococo sensor: {sensor_id}", "INFO")
            
            self.sensor_active = False
            self.rococo_connection_status = False
            
            self.logging(f"Rococo sensor {sensor_id} stopped successfully", "INFO")
            return True
            
        except Exception as e:
            self.logging(f"Error stopping sensor {sensor_id}: {str(e)}", "ERROR")
            return False

    def get_data_from_sensor(self, sensor_id: str) -> Any:
        """Get data from rococo glove sensor."""
        if not self.sensor_active or not self.rococo_connection_status:
            return None
            
        try:
            # NOTE: Generate dummy hand keypoint data (5 fingertips)
            # In real implementation, this would read from rococo gloves UDP stream
            fingertips = []
            finger_names = ["thumb", "index", "middle", "ring", "pinky"]
            
            # Generate realistic hand pose data with some variation
            base_time = self.get_clock().now().nanoseconds / 1e9
            variation = 0.02 * np.sin(base_time * 2.0)  # Small sinusoidal movement
            
            # Dummy fingertip positions (relative to hand frame)
            base_positions = [
                [0.05 + variation, 0.02, 0.01],      # thumb
                [0.08, 0.04 + variation, 0.02],      # index  
                [0.09, 0.01 + variation, 0.025],     # middle
                [0.085, -0.02 + variation, 0.02],    # ring
                [0.07, -0.04 + variation, 0.015]     # pinky
            ]
            
            for i, pos in enumerate(base_positions):
                point = Point()
                point.x = float(pos[0])
                point.y = float(pos[1]) 
                point.z = float(pos[2])
                fingertips.append(point)
            
            # Create hand keypoint data structure
            hand_data = {
                'fingertips': fingertips,
                'finger_names': finger_names,
                'confidence': 0.95,
                'timestamp': self.get_clock().now().to_msg()
            }
            
            return hand_data
            
        except Exception as e:
            self.logging(f"Error getting data from sensor {sensor_id}: {str(e)}", "ERROR")
            return None

    def _create_raw_hand_keypoint_msg(self, hand_data: Dict[str, Any]) -> RawHandKeypoint:
        """Create RawHandKeypoint message from hand data."""
        msg = RawHandKeypoint()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = hand_data['timestamp']
        msg.header.frame_id = "rococo"  # From data_stream.yaml frame_id
        
        # Set fingertip data
        msg.fingertips = hand_data['fingertips']
        msg.finger_names = hand_data['finger_names']
        msg.confidence = hand_data['confidence']
        
        return msg

    def stream(self):
        """Main streaming function - called by timer."""
        if not self.sensor_active:
            return
            
        try:
            # Get data from rococo sensor
            hand_data = self.get_data_from_sensor('rococo')
            
            if hand_data is not None:
                # Create and publish RawHandKeypoint message
                if 'right_hand_keypoint_raw' in self.data_publishers:
                    msg = self._create_raw_hand_keypoint_msg(hand_data)
                    self.publish_data('right_hand_keypoint_raw', msg)
                    
                    self.logging("Published right hand keypoint data", "DEBUG")
                else:
                    self.logging("No publisher found for right_hand_keypoint_raw", "WARN")
            else:
                self.logging("No data received from rococo sensor", "DEBUG")
                
        except Exception as e:
            self.logging(f"Error in stream: {str(e)}", "ERROR")

    def _start_streaming(self):
        """Start the streaming timer."""
        if self.timer is not None:
            self.timer.cancel()
            
        # Calculate timer period from frequency
        timer_period = 1.0 / self.freq
        self.timer = self.create_timer(timer_period, self.stream)
        
        # Start the rococo sensor
        self.start_sensor('rococo')
        
        self.logging(f"Started streaming timer at {self.freq}Hz", "INFO")

    def check_plugins(self) -> bool:
        """Verify that this driver implements required interfaces."""
        required_methods = ['start_sensor', 'stop_sensor', 'get_data_from_sensor', 'stream']
        
        for method_name in required_methods:
            if not hasattr(self, method_name):
                self.logging(f"Missing required method: {method_name}", "ERROR")
                return False
                
        self.logging("All required plugin methods implemented", "INFO")
        return True

    def urgent_stop(self):
        """Emergency stop - stop streaming and sensor."""
        self.logging("Emergency stop triggered", "WARN")
        
        # Cancel timer
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            
        # Stop sensor
        self.stop_sensor('rococo')
        
        self.logging("Emergency stop completed", "WARN")

    def wait(self):
        """Wait for any pending operations to complete."""
        if self.timer is not None and self.sensor_active:
            self.logging("Waiting for streaming operations to complete", "DEBUG")
        
