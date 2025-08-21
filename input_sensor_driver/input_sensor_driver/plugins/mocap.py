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
from geometry_msgs.msg import Pose

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
        self.data_publishers = {}
        
        self._initialize_node()

        self._wait_for_start_cmd()

        # Start streaming timer
        self._start_streaming()

    def _initialize_node(self):
        """Initialize Rococo driver node."""
        self._load_config()


        # Register publishers and subscribers
        self.register_pub_sub()


        # Verify plugins
        if self.check_plugins():
            self.set_state("ready")


    def _load_config(self):
        """Load driver-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config.get("plugins", [])
                for plugin in plugins:
                    if plugin.get('id') == 'rococo_driver' or plugin.get('_target_', '').endswith('RococoDriver'):
                        plugin_params = plugin.get("params", {})
                        self.freq = plugin_params.get("freq", self.freq)
                        self.get_logger().info(f"Loaded RococoDriver config: {self.freq}Hz")
                        break

            # Load data stream configuration
            self.data_stream_config = self.load_data_stream_config()
            if not self.data_stream_config:
                self.get_logger().warn("Failed to load data stream configuration")
        except Exception as e:
            self.handle_error("Failed to load RococoDriver config: ", e)


    def register_pub_sub(self):
        """Register publishers and subscribers based on data stream configuration."""
        try:
            if not self.data_stream_config:
                self.handle_error("No data stream configuration available")
                return

            # Iterate through configured topics
            for stream in self.data_stream_config.get("topics", []):
                topic_name = stream.get("id")
                msg_type = stream.get("msg_type")
                publisher = stream.get("publisher")

                if publisher != self.layer_cn_id:
                    continue

                if not topic_name or not msg_type:
                    continue

                if msg_type == "component_interfaces/msg/RawHandKeypoint":
                    self.data_publishers[topic_name] = self.create_publisher(
                            RawHandKeypoint, topic_name, 10
                        )
                    # self.get_logger().info(f"Publisher registered: {topic_name} [{msg_type}]")
                    self.logging(f"Publisher registered: {topic_name} [{msg_type}]", "INFO")

        except Exception as e:
            self.handle_error("Error during pub/sub registration: ", e)

    def start_sensor(self, sensor_id: str) -> bool:
        """Start the rococo glove sensor."""
        try:
            self.logging(f"Starting rococo sensor: {sensor_id}", "INFO")
            
            # # Get sensor connection info
            # connection_info = self.get_sensor_ip_port(sensor_id)
            # if connection_info:
            #     ip, port = connection_info
            #     self.logging(f"Connecting to rococo gloves at {ip}:{port}", "INFO")
                
                # Simulate connection to rococo gloves
                # In real implementation, this would establish UDP/TCP connection
            self.rococo_connection_status = True
            self.sensor_active = True
            
            self.logging(f"Rococo sensor {sensor_id} started successfully", "INFO")
            return True
            # else:
            #     self.handle_error(f"No connection info found for sensor: {sensor_id}")
            #     return False
                
        except Exception as e:
            self.handle_error(f"Error starting sensor {sensor_id}: ", e)
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
            self.handle_error(f"Error stopping sensor {sensor_id}: ", e)
            return False


    def get_data_from_sensor(self, sensor_id: str) -> Any:
        """Get data from rococo glove sensor."""
        if not self.sensor_active or not self.rococo_connection_status:
            return None
            
        try:
            # Generate full hand keypoint data (21 keypoints for MediaPipe-style hand model)
            # MediaPipe standard hand landmark order (indices 0-20)
            keypoint_names = [
                "wrist",                    # 0
                # Thumb (indices 1-4)
                "thumb_cmc", "thumb_mcp", "thumb_ip", "thumb_tip",      # 1,2,3,4
                # Index finger (indices 5-8)
                "index_mcp", "index_pip", "index_dip", "index_tip",    # 5,6,7,8
                # Middle finger (indices 9-12) 
                "middle_mcp", "middle_pip", "middle_dip", "middle_tip", # 9,10,11,12
                # Ring finger (indices 13-16)
                "ring_mcp", "ring_pip", "ring_dip", "ring_tip",        # 13,14,15,16
                # Pinky finger (indices 17-20)
                "pinky_mcp", "pinky_pip", "pinky_dip", "pinky_tip"     # 17,18,19,20
            ]
            
            # Generate realistic hand pose data with some variation
            base_time = self.get_clock().now().nanoseconds / 1e9
            position_variation = 0.02 * np.sin(base_time * 2.0)
            
            # Base positions for all 21 keypoints (MediaPipe format - relative to hand frame)
            base_positions = [
                # Wrist (index 0)
                [0.0, 0.0, 0.0],
                
                # Thumb (indices 1-4: CMC, MCP, IP, TIP)
                [0.015, 0.008, -0.005],     # thumb_cmc (1)
                [0.025, 0.012, -0.003],     # thumb_mcp (2) 
                [0.035, 0.016, 0.0],        # thumb_ip (3)
                [0.045 + position_variation, 0.020, 0.005],  # thumb_tip (4)
                
                # Index finger (indices 5-8: MCP, PIP, DIP, TIP)
                [0.015, 0.025, -0.002],     # index_mcp (5)
                [0.035, 0.030, 0.005],      # index_pip (6)
                [0.055, 0.032, 0.010],      # index_dip (7) 
                [0.075, 0.035 + position_variation, 0.015],  # index_tip (8)
                
                # Middle finger (indices 9-12: MCP, PIP, DIP, TIP)
                [0.005, 0.030, -0.002],     # middle_mcp (9)
                [0.025, 0.038, 0.008],      # middle_pip (10)
                [0.050, 0.042, 0.015],      # middle_dip (11)
                [0.080, 0.045 + position_variation, 0.022], # middle_tip (12)
                
                # Ring finger (indices 13-16: MCP, PIP, DIP, TIP) 
                [-0.005, 0.025, -0.002],    # ring_mcp (13)
                [0.015, 0.030, 0.005],      # ring_pip (14)
                [0.040, 0.032, 0.012],      # ring_dip (15)
                [0.070, 0.028 + position_variation, 0.018], # ring_tip (16)
                
                # Pinky finger (indices 17-20: MCP, PIP, DIP, TIP)
                [-0.015, 0.020, -0.003],    # pinky_mcp (17)
                [-0.005, 0.022, 0.002],     # pinky_pip (18)
                [0.018, 0.020, 0.008],      # pinky_dip (19)
                [0.045, 0.015 + position_variation, 0.012]  # pinky_tip (20)
            ]
            
            # Create normalized quaternions for each keypoint
            fingertip_poses = []
            for i, position in enumerate(base_positions):
                pose = Pose()
                
                # Set position
                pose.position.x = float(position[0])
                pose.position.y = float(position[1]) 
                pose.position.z = float(position[2])
                
                # Create normalized quaternion with slight variation
                angle_variation = 0.1 * np.sin(base_time * 1.5 + i * 0.5)
                
                # Simple rotation around z-axis for variety
                cos_half = np.cos(angle_variation / 2.0)
                sin_half = np.sin(angle_variation / 2.0)
                
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = float(sin_half)
                pose.orientation.w = float(cos_half)
                
                fingertip_poses.append(pose)
            
            # Create hand keypoint data structure
            hand_data = {
                'fingertip_poses': fingertip_poses,  # Now contains all 21 keypoints
                'finger_names': keypoint_names,      # Updated to match
                'confidence': 0.95,
                'timestamp': self.get_clock().now().to_msg()
            }
            
            return hand_data
            
        except Exception as e:
            self.handle_error(f"Error getting data from sensor {sensor_id}: ", e)
            return None
            
    # def get_data_from_sensor(self, sensor_id: str) -> Any:
    #     """Get data from rococo glove sensor."""
    #     if not self.sensor_active or not self.rococo_connection_status:
    #         return None
            
    #     try:
    #         # NOTE: Generate dummy hand keypoint pose data (5 fingertips)
    #         # In real implementation, this would read from rococo gloves UDP stream
    #         fingertip_poses = []
    #         finger_names = ["thumb", "index", "middle", "ring", "pinky"]
            
    #         # Generate realistic hand pose data with some variation
    #         base_time = self.get_clock().now().nanoseconds / 1e9
    #         position_variation = 0.02 * np.sin(base_time * 2.0)  # Small sinusoidal movement
    #         orientation_variation = 0.1 * np.sin(base_time * 1.5)  # Small orientation variation
            
    #         # Dummy fingertip positions and orientations (relative to hand frame)
    #         base_positions = [
    #             [0.05 + position_variation, 0.02, 0.01],      # thumb
    #             [0.08, 0.04 + position_variation, 0.02],      # index  
    #             [0.09, 0.01 + position_variation, 0.025],     # middle
    #             [0.085, -0.02 + position_variation, 0.02],    # ring
    #             [0.07, -0.04 + position_variation, 0.015]     # pinky
    #         ]
            
    #         # Base orientations (quaternions) for each finger
    #         base_orientations = [
    #             [0.0, 0.0, 0.0, 1.0],                        # thumb - identity
    #             [0.0, orientation_variation, 0.0, 1.0],      # index - slight y rotation
    #             [0.0, 0.0, orientation_variation, 1.0],      # middle - slight z rotation  
    #             [orientation_variation, 0.0, 0.0, 1.0],      # ring - slight x rotation
    #             [0.0, -orientation_variation, 0.0, 1.0]      # pinky - slight negative y rotation
    #         ]
            
    #         for i in range(len(base_positions)):
    #             # Create Pose message
    #             pose = Pose()
                
    #             # Set position
    #             pose.position.x = float(base_positions[i][0])
    #             pose.position.y = float(base_positions[i][1])
    #             pose.position.z = float(base_positions[i][2])
                
    #             # Set orientation (quaternion)
    #             pose.orientation.x = float(base_orientations[i][0])
    #             pose.orientation.y = float(base_orientations[i][1])
    #             pose.orientation.z = float(base_orientations[i][2])
    #             pose.orientation.w = float(base_orientations[i][3])
                
    #             fingertip_poses.append(pose)
            
    #         # Create hand keypoint data structure
    #         hand_data = {
    #             'fingertip_poses': fingertip_poses,
    #             'finger_names': finger_names,
    #             'confidence': 0.95,
    #             'timestamp': self.get_clock().now().to_msg()
    #         }
            
    #         return hand_data
            
    #     except Exception as e:
    #         self.handle_error(f"Error getting data from sensor {sensor_id}: ", e)
    #         return None

    def _create_raw_hand_keypoint_msg(self, hand_data: Dict[str, Any]) -> RawHandKeypoint:
        """Create RawHandKeypoint message from hand data."""
        msg = RawHandKeypoint()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = hand_data['timestamp']
        msg.header.frame_id = "rococo"  # From data_stream.yaml frame_id
        
        # Set fingertip data
        msg.fingertips = hand_data['fingertip_poses']
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
            self.handle_error(f"Error in stream: ", e)

    def _start_streaming(self):
        if self.timer is not None:
            self.timer.cancel()

        # Only start timer if sensor starts successfully
        if self.start_sensor('rococo'):
            timer_period = 1.0 / self.freq
            self.timer = self.create_timer(timer_period, self.stream)
            self.logging(f"Started streaming timer at {self.freq}Hz", "INFO")
        else:
            self.logging("Failed to start sensor - streaming not started", "ERROR")

    def check_plugins(self) -> bool:
        """Verify that this driver implements required interfaces."""
        required_methods = ['start_sensor', 'stop_sensor', 'get_data_from_sensor', 'stream']
        
        for method_name in required_methods:
            if not hasattr(self, method_name):
                self.handle_error(f"Missing required method: {method_name}")
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
        self.set_state("disconnected")
        
        self.logging("Emergency stop completed", "WARN")

    def wait(self):
        """Wait for any pending operations to complete."""
        if self.timer is not None and self.sensor_active:
            self.logging("Waiting for streaming operations to complete", "DEBUG")
        
