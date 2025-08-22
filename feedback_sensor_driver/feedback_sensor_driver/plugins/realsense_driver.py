import rclpy
from rclpy.timer import Timer
import json
import numpy as np
import cv2
from typing import Dict, List, Any, Optional
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from core_bases.sensor_driver_base import SensorDriverBase
import hydra
from omegaconf import OmegaConf, DictConfig
from hydra.utils import instantiate


class RealsenseDriver(SensorDriverBase):
    """
    ROS2 Driver node for RealSense camera sensor input.
    Captures and processes camera sensor data for feedback.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Driver specific attributes
        self.cam_ids = [0]
        self.width = 1280
        self.height = 720
        self.fps = 30
        self.fps_sim = 60
        self.rotation_angle = 0
        self.processing_preset = 1
        
        # Constants
        self.VISUAL_RESCALE_FACTOR = 2
        self.VIZ_PORT_OFFSET = 500
        self.DEPTH_PORT_OFFSET = 1000
        
        # Runtime attributes
        self.timer: Optional[Timer] = None
        self.sensor_active = False
        self.camera_connections = {}
        self.cv_bridge = CvBridge()
        
        # Load plugin parameters
        self._initialize_node()
        # self._wait_for_start_cmd()
        
        # Start streaming timer
        self._start_streaming()

    def _initialize_node(self):
        """Override the base initialization for RealsenseDriver specific setup"""

        # Load configurations using ComponentNode methods
        self._load_configs()

        # Load sensor configuration from hardware config
        self._load_sensor_config_from_hardware()
        
        self.register_pub_sub()

        if self.check_plugins():
            self.set_state("ready")

    def _load_configs(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if not self.my_config:
                self.get_logger().warn(f"No component config found for {self.layer_cn_id}")
                self.my_config = {}
            
            # Load data stream configuration
            self.data_stream_config = self.load_data_stream_config()
            if not self.data_stream_config:
                self.get_logger().error("Failed to load data stream configuration")
                self.data_stream_config = {}
            
            # Load hardware configuration for sensor settings
            self.hardware_config = self.load_hardware_config()
            if not self.hardware_config:
                self.get_logger().warn("No hardware config found")
                self.hardware_config = {}
            
            # Load plugin parameters from component config
            self._load_plugin_params_from_config()
            
            self.get_logger().info("All configurations loaded successfully")
            
        except Exception as e:
            self.handle_error("Failed to load configurations", e)

    def _load_plugin_params_from_config(self):
        """Load plugin-specific parameters from loaded configuration."""
        try:
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'realsense_driver' or plugin.get('_target_', '').endswith('RealsenseDriver'):
                        params = plugin.get('params', {})
                        self.cam_ids = params.get('cam_ids', self.cam_ids)
                        self.width = params.get('width', self.width)
                        self.height = params.get('height', self.height)
                        self.fps = params.get('fps', self.fps)
                        self.fps_sim = params.get('fps_sim', self.fps_sim)
                        self.rotation_angle = params.get('rotation_angle', self.rotation_angle)
                        self.processing_preset = params.get('processing_preset', self.processing_preset)
                        
                        # Load constants
                        constants = params.get('constants', {})
                        self.VISUAL_RESCALE_FACTOR = constants.get('VISUAL_RESCALE_FACTOR', self.VISUAL_RESCALE_FACTOR)
                        self.VIZ_PORT_OFFSET = constants.get('VIZ_PORT_OFFSET', self.VIZ_PORT_OFFSET)
                        self.DEPTH_PORT_OFFSET = constants.get('DEPTH_PORT_OFFSET', self.DEPTH_PORT_OFFSET)
                        break
                        
                self.get_logger().info(f"Loaded plugin params: cams={self.cam_ids}, res={self.width}x{self.height}, fps={self.fps}")
            else:
                self.get_logger().warn("Using default plugin parameters - no plugin config found")
                
        except Exception as e:
            self.handle_error("Error loading plugin params from config", e)

    def _load_sensor_config_from_hardware(self):
        """Load sensor configuration from hardware config."""
        try:
            if self.hardware_config and 'sensors' in self.hardware_config:
                # Convert list format to dictionary for easier access
                hardware_list = self.hardware_config.get('sensors', [])
                self.sensor_config = {}
                
                for hw_item in hardware_list:
                    if isinstance(hw_item, dict) and 'hardware_id' in hw_item:
                        hardware_id = hw_item['hardware_id']
                        if hardware_id != 'realsense':
                            continue
                        # Create sensor config entry with IP and access info
                        sensor_info = {
                            'ip': hw_item.get('ip', ''),
                            'access_port': hw_item.get('access_port', 22),
                            'description': f"Sensor on {hardware_id}",
                            'execution_platform': hw_item.get('execution_plattform', False)
                        }
                        
                        # Add SSH info if available
                        if 'ssh' in hw_item:
                            sensor_info.update(hw_item['ssh'])
                            
                        self.sensor_config[hardware_id] = sensor_info
                
                self.get_logger().info(f"Loaded sensor config for hardware: {list(self.sensor_config.keys())}")
            else:
                self.get_logger().warn("No hardware configuration found for sensors")
                self.sensor_config = {}
                
        except Exception as e:
            self.handle_error("Error loading sensor config from hardware", e)
            self.sensor_config = {}

    def get_sensor_ip_port(self, sensor_id: str) -> Optional[tuple]:
        """Get sensor IP and port by sensor ID."""
        if sensor_id in self.sensor_config:
            sensor_info = self.sensor_config[sensor_id]
            ip = sensor_info.get('ip')
            port = sensor_info.get('access_port')
            if ip and port:
                return (ip, port)
        return None
            
    def register_pub_sub(self):
        """Register publishers and subscribers based on data stream configuration."""
        try:
            # Use the loaded data_stream_config instead of manually parsing YAML/JSON
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return
            
            topics = self.data_stream_config.get('topics', [])
            
            # Find topics where this node is the publisher
            for topic in topics:
                if topic.get('publisher') == 'feedback_sensor_driver':  # From data_stream.yaml
                    topic_id = topic.get('id')
                    msg_type = topic.get('msg_type')
                    publish_freq = topic.get('publish_freq', 30)
                    
                    if msg_type == 'sensor_msgs/msg/Image':
                        # Create publisher for raw image data
                        publisher = self.create_publisher(
                            Image,
                            topic_id,
                            10
                        )
                        self.data_publishers[topic_id] = publisher
                        self.get_logger().info(f"Created publisher for {topic_id} with type {msg_type}")
                    else:
                        self.get_logger().warn(f"Unsupported message type: {msg_type} for topic {topic_id}")
            
            self.get_logger().info(f"Registered {len(self.data_publishers)} publishers")
            
        except Exception as e:
            self.handle_error("Error in register_pub_sub", e)

    def start_sensor(self, sensor_id: str) -> bool:
        """Start the RealSense camera sensor."""
        try:
            self.get_logger().info(f"Starting RealSense sensor: {sensor_id}")
            
            # Get sensor connection info
            connection_info = self.get_sensor_ip_port(sensor_id)
            if connection_info:
                ip, port = connection_info
                self.get_logger().info(f"Connecting to RealSense camera at {ip}:{port}")
            
            # Simulate camera initialization for each cam_id
            for cam_id in self.cam_ids:
                # In real implementation, this would initialize pyrealsense2 pipeline
                camera_config = {
                    'cam_id': cam_id,
                    'width': self.width,
                    'height': self.height,
                    'fps': self.fps,
                    'active': True,
                    'frame_count': 0
                }
                self.camera_connections[cam_id] = camera_config
                self.get_logger().info(f"Initialized camera {cam_id} with config: {self.width}x{self.height}@{self.fps}fps")
            
            self.sensor_active = True
            self.get_logger().info(f"RealSense sensor {sensor_id} started successfully with {len(self.cam_ids)} cameras")
            return True
                
        except Exception as e:
            self.handle_error(f"Error starting sensor {sensor_id}", e)
            return False

    def stop_sensor(self, sensor_id: str) -> bool:
        """Stop the RealSense camera sensor."""
        try:
            self.get_logger().info(f"Stopping RealSense sensor: {sensor_id}")
            
            # Stop all cameras
            for cam_id in list(self.camera_connections.keys()):
                self.camera_connections[cam_id]['active'] = False
                del self.camera_connections[cam_id]
                self.get_logger().info(f"Stopped camera {cam_id}")
            
            self.sensor_active = False
            self.get_logger().info(f"RealSense sensor {sensor_id} stopped successfully")
            return True
            
        except Exception as e:
            self.handle_error(f"Error stopping sensor {sensor_id}", e)
            return False

    def get_data_from_sensor(self, sensor_id: str) -> Any:
        """Get data from RealSense camera sensor."""
        if not self.sensor_active or not self.camera_connections:
            return None
            
        try:
            sensor_data = {}
            
            # Get data from each active camera
            for cam_id, cam_config in self.camera_connections.items():
                if not cam_config['active']:
                    continue
                    
                # Generate dummy RGB image data
                # In real implementation, this would capture from RealSense pipeline
                rgb_image = self._generate_dummy_rgb_image(cam_id, cam_config)
                
                # Apply rotation if specified
                if self.rotation_angle != 0:
                    rgb_image = self._rotate_image(rgb_image, self.rotation_angle)
                
                # Update frame count
                cam_config['frame_count'] += 1
                
                sensor_data[f'camera_{cam_id}'] = {
                    'rgb_image': rgb_image,
                    'timestamp': self.get_clock().now().to_msg(),
                    'cam_id': cam_id,
                    'frame_count': cam_config['frame_count']
                }
            
            return sensor_data if sensor_data else None
            
        except Exception as e:
            self.handle_error(f"Error getting data from sensor {sensor_id}", e)
            return None

    def _generate_dummy_rgb_image(self, cam_id: int, cam_config: Dict) -> np.ndarray:
        """Generate dummy RGB image with realistic content."""
        # Create base image with gradient background
        image = np.zeros((cam_config['height'], cam_config['width'], 3), dtype=np.uint8)
        
        # Add gradient background
        for y in range(cam_config['height']):
            for x in range(cam_config['width']):
                image[y, x] = [
                    int(128 + 127 * np.sin(x * 0.01 + cam_config['frame_count'] * 0.1)),
                    int(128 + 127 * np.sin(y * 0.01 + cam_config['frame_count'] * 0.1)),
                    100 + cam_id * 50  # Different base color per camera
                ]
        
        # Add some moving objects (circles)
        center_x = int(cam_config['width']/2 + 100 * np.sin(cam_config['frame_count'] * 0.05))
        center_y = int(cam_config['height']/2 + 100 * np.cos(cam_config['frame_count'] * 0.05))
        
        cv2.circle(image, (center_x, center_y), 50, (0, 255, 255), -1)  # Yellow circle
        
        # Add text overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = f"Camera {cam_id} - Frame {cam_config['frame_count']}"
        cv2.putText(image, text, (10, 30), font, 0.7, (255, 255, 255), 2)
        
        # Add timestamp
        timestamp_text = f"FPS: {self.fps}"
        cv2.putText(image, timestamp_text, (10, cam_config['height'] - 20), font, 0.5, (255, 255, 255), 1)
        
        return image

    def _rotate_image(self, image: np.ndarray, angle: float) -> np.ndarray:
        """Rotate image by specified angle."""
        if angle == 0:
            return image
            
        height, width = image.shape[:2]
        center = (width // 2, height // 2)
        
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_image = cv2.warpAffine(image, rotation_matrix, (width, height))
        
        return rotated_image

    def _create_image_msg(self, rgb_image: np.ndarray, timestamp: Time, cam_id: int) -> Image:
        """Create ROS2 Image message from RGB image."""
        try:
            # Convert OpenCV image to ROS2 Image message
            img_msg = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            
            # Set header
            img_msg.header = Header()
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = "xarm5_robot"  # From data_stream.yaml frame_id
            
            return img_msg
            
        except Exception as e:
            self.handle_error(f"Error creating image message for cam {cam_id}", e)
            return None

    def stream(self):
        """Main streaming function - called by timer."""
        if not self.sensor_active:
            return
            
        try:

            if self.state!='running':
                return  

            # Get data from RealSense cameras
            sensor_data = self.get_data_from_sensor('realsense')
            
            if sensor_data is not None:
                # Publish data for each camera
                for cam_key, cam_data in sensor_data.items():
                    rgb_image = cam_data['rgb_image']
                    timestamp = cam_data['timestamp']
                    cam_id = cam_data['cam_id']
                    
                    # Create and publish Image message
                    if 'scene_image_raw' in self.data_publishers:
                        img_msg = self._create_image_msg(rgb_image, timestamp, cam_id)
                        if img_msg is not None:
                            self.publish_data('scene_image_raw', img_msg)
                            self.get_logger().debug(f"Published scene image from camera {cam_id}")
                    else:
                        self.get_logger().warn("No publisher found for scene_image_raw")
            else:
                self.get_logger().debug("No data received from RealSense cameras")
                
        except Exception as e:
            self.handle_error("Error in stream", e)

    def _start_streaming(self):
        """Start the streaming timer."""
        if self.timer is not None:
            self.timer.cancel()
            
        # Calculate timer period from frequency
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.stream)
        
        # Start the RealSense sensor
        self.start_sensor('realsense')
        
        self.get_logger().info(f"Started streaming timer at {self.fps}Hz")

    def check_plugins(self) -> bool:
        """Verify that this driver implements required interfaces."""
        required_methods = ['start_sensor', 'stop_sensor', 'get_data_from_sensor', 'stream']
        
        for method_name in required_methods:
            if not hasattr(self, method_name):
                self.handle_error(f"Missing required method: {method_name}")
                return False
                
        self.get_logger().info("All required plugin methods implemented")
        return True

    def urgent_stop(self):
        """Emergency stop - stop streaming and cameras."""
        self.get_logger().warn("Emergency stop triggered")
        
        # Cancel timer
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            
        # Stop all cameras
        self.stop_sensor('realsense')
        self.set_state("disconnected")
        
        self.get_logger().warn("Emergency stop completed")

    def wait(self):
        """Wait for any pending operations to complete."""
        if self.timer is not None and self.sensor_active:
            self.get_logger().debug("Waiting for camera streaming operations to complete")