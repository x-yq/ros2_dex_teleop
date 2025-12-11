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

class VRDetectorDriver(SensorDriverBase):
    """
    ROS2 Driver node for VR glove sensor input.
    Captures and processes hand keypoint data from VR gloves.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Driver specific attributes
        self.freq = 1  # Default frequency
        self.sensor_active = False
        self.vr_connection_status = False

        self.raw_keypoint_left_socket = None
        self.raw_keypoint_right_socket = None
        self.button_feedback = None
        self.left_hand_port = 8110
        self.right_hand_port = 8087
        self.button_port = 8095

        self._load_my_config()
        self.get_logger().warn(f"simulated latency: {self.simulate_latency}")
        if self.simulate_latency > 0:
            self._setup_latency_simulation()

        self.start_sensor('vr_detector')
        
        self._register_zmq_sub()


    def _load_my_config(self):
        """Load driver-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config.get("plugins", [])
                for plugin in plugins:
                    if plugin.get('id') == 'vr_detector_driver':
                        plugin_params = plugin.get("params", {})
                        self.execution_platform_ip = plugin_params.get("execution_platform_ip", self.execution_platform_ip)
                        self.left_hand_port = plugin_params.get("left_hand_port", self.left_hand_port)
                        self.right_hand_port = plugin_params.get("right_hand_port", self.right_hand_port)
                        self.button_port = plugin_params.get("button_port", self.button_port)

                        self.freq = plugin_params.get("freq", self.freq)
                        self.simulate_latency = plugin_params.get("simulate_latency", self.simulate_latency)
                        self.get_logger().info(f"Loaded VRDriver config: {self.freq}Hz")
                        break

        except Exception as e:
            self.handle_error("Failed to load VRDriver config: ", e)


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

                # if publisher != self.layer_cn_id:
                #     continue

                # if not topic_name or not msg_type:
                #     continue

                if msg_type == "component_interfaces/msg/RawHandKeypoint":
                    self.compo_publishers[topic_name] = self.create_publisher(
                            RawHandKeypoint, topic_name, 10
                        )
                    # self.get_logger().info(f"Publisher registered: {topic_name} [{msg_type}]")
                    self.get_logger().info(f"Publisher registered: {topic_name} [{msg_type}]")

        except Exception as e:
            self.handle_error("Error during pub/sub registration: ", e)

    def _register_zmq_sub(self):

        self.raw_keypoint_left_socket = self.create_pull_socket(self.execution_platform_ip, self.left_hand_port)
        self.raw_keypoint_right_socket = self.create_pull_socket(self.execution_platform_ip, self.right_hand_port)
        self.button_keypoint_socket = self.create_pull_socket(self.execution_platform_ip, self.button_port)

    def start_sensor(self, sensor_id: str) -> bool:
        """Start the VR glove sensor."""
        try:
            self.logging(f"Starting VR sensor: {sensor_id}", "INFO")
            
            # # Get sensor connection info
            # connection_info = self.get_sensor_ip_port(sensor_id)
            # if connection_info:
            #     ip, port = connection_info
            #     self.logging(f"Connecting to VR gloves at {ip}:{port}", "INFO")
                
                # Simulate connection to VR gloves
                # In real implementation, this would establish UDP/TCP connection
            self.vr_connection_status = True
            self.sensor_active = True
            
            self.logging(f"VR sensor {sensor_id} started successfully", "INFO")
            return True
            # else:
            #     self.handle_error(f"No connection info found for sensor: {sensor_id}")
            #     return False
                
        except Exception as e:
            self.handle_error(f"Error starting sensor {sensor_id}: ", e)
            return False

    def stop_sensor(self, sensor_id: str) -> bool:
        """Stop the VR glove sensor."""
        try:
            self.logging(f"Stopping VR sensor: {sensor_id}", "INFO")
            
            self.sensor_active = False
            self.vr_connection_status = False
            
            self.logging(f"VR sensor {sensor_id} stopped successfully", "INFO")
            return True
            
        except Exception as e:
            self.handle_error(f"Error stopping sensor {sensor_id}: ", e)
            return False

    def _extract_data_from_token(self, token):        
        data = token.decode().strip()
        information = dict()
        keypoint_vals = [0] if data.startswith('absolute') else [1]
        # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
        vector_strings = data.split(':')[1].strip().split('|')
        for vector_str in vector_strings:
            vector_vals = vector_str.split(',')
            for float_str in vector_vals[:3]:
                keypoint_vals.append(float(float_str))
            
        information['keypoints'] = keypoint_vals
        return information
    
    def _create_raw_hand_keypoint_msg(self, hand_data: Dict[str, Any]) -> RawHandKeypoint:
        """Create RawHandKeypoint message from parsed hand token data."""
        msg = RawHandKeypoint()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "VR"

        # Send ID (optional, if available)
        msg.send_id = hand_data.get("send_id", "")

        # First value = mode (0=absolute, 1=relative)
        keypoints = hand_data["keypoints"]
        msg.mode = int(keypoints[0])

        # Remaining values = flattened xyz keypoints
        msg.keypoints = keypoints[1:]

        return msg

    def stream(self):
        """Main streaming function - called by timer."""
        if not self.sensor_active:
            return

        try:
            ##NOTE
            # if self.state!='running':
            #     return  

            
            # Get data from VR sensor
            # hand_data = self.get_data_from_sensor('VR')
            raw_right_keypoints = self.raw_keypoint_right_socket.recv()
            raw_left_keypoints = self.raw_keypoint_left_socket.recv()
            button_feedback = self.button_keypoint_socket.recv()
            
            if raw_right_keypoints is not None:
                # Create and publish RawHandKeypoint message
                # if 'right_hand_keypoint_raw' in self.compo_publishers:
                raw_right_keypoints = self._extract_data_from_token(raw_right_keypoints)
                msg = self._create_raw_hand_keypoint_msg(raw_right_keypoints)
                topic_id = 'right_hand_keypoint_raw'
                self.publish_msg(topic_id, msg)
                
                self.logging("Published right hand keypoint data", "DEBUG")
                # else:
                #     self.logging("No publisher found for right_hand_keypoint_raw", "WARN")
            else:
                self.logging("No data received from VR sensor", "DEBUG")
                
        except Exception as e:
            self.handle_error(f"Error in stream: ", e)

    def register_timer(self):
        if self.compo_timers != []:
            for t in self.compo_timers:
                t.destroy()
            self.compo_timers.clear()

        # Calculate timer period from frequency
        timer_period = 1.0 / self.freq
        timer = self.create_timer(timer_period, self.stream)
        self.logging(f"Registered timer at {self.freq}Hz", "INFO")
        self.compo_timers.append(timer)

    
    def start_component(self):
        if self.compo_timers == [] or not self.compo_timers:
            self.register_timer() #defined in the bottom node
        else:
            for timer in self.compo_timers:
                # if timer.is_canceled():
                timer.reset()
        self.start_sensor('vr_detector')

    def urgent_stop(self):
        """Emergency stop - stop streaming and sensor."""
        self.logging("Emergency stop triggered", "WARN")
        
        # # stop the timer
        # self.stop_component()

        self.raw_keypoint_left_socket.close()
        self.raw_keypoint_right_socket.close()
        self.button_keypoint_socket.close()

        # Stop sensor
        self.stop_sensor('vr_detector')
        self.set_state("disconnected")
        
        self.logging("Emergency stop completed", "WARN")
        
