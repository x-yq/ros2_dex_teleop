import rclpy
from rclpy.timer import Timer
import json
import yaml
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
        self._load_plugin_params()
        
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
                
                # Extract parameters from plugin params
                plugins = config.get('plugins', [])
                for plugin in plugins:
                    if plugin.get('id') == 'realsense_driver' or plugin.get('class_path', '').endswith('RealsenseDriver'):
                        params = plugin.get('params', {})
                        self.cam_ids = params.get('cam_ids', [0])
                        self.width = params.get('width', 1280)
                        self.height = params.get('height', 720)
                        self.fps = params.get('fps', 30)
                        self.fps_sim = params.get('fps_sim', 60)
                        self.rotation_angle = params.get('rotation_angle', 0)
                        self.processing_preset = params.get('processing_preset', 1)
                        
                        # Load constants
                        constants = params.get('constants', {})
                        self.VISUAL_RESCALE_FACTOR = constants.get('VISUAL_RESCALE_FACTOR', 2)
                        self.VIZ_PORT_OFFSET = constants.get('VIZ_PORT_OFFSET', 500)
                        self.DEPTH_PORT_OFFSET = constants.get('DEPTH_PORT_OFFSET', 1000)
                        break
                        
                self.logging(f"Loaded plugin params: cams={self.cam_ids}, res={self.width}x{self.height}, fps={self.fps}", "INFO")
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
                        self.logging(f"Created publisher for {topic_id} with type {msg_type}", "INFO")
                    else:
                        self.logging(f"Unsupported message type: {msg_type} for topic {topic_id}", "WARN")
            
            self.logging(f"Registered {len(self.data_publishers)} publishers", "INFO")
            
        except Exception as e:
            self.logging(f"Error in register_pub_sub: {str(e)}", "ERROR")

    def start_sensor(self, sensor_id: str) -> bool:
        """Start the RealSense camera sensor."""
        try:
            self.logging(f"Starting RealSense sensor: {sensor_id}", "INFO")
            
            # Get sensor connection info
            connection_info = self.get_sensor_ip_port(sensor_id)
            if connection_info:
                ip, port = connection_info
                self.logging(f"Connecting to RealSense camera at {ip}:{port}", "INFO")
            
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
                self.logging(f"Initialized camera {cam_id} with config: {self.width}x{self.height}@{self.fps}fps", "INFO")
            
            self.sensor_active = True
            self.logging(f"RealSense sensor {sensor_id} started successfully with {len(self.cam_ids)} cameras", "INFO")
            return True
                
        except Exception as e:
            self.logging(f"Error starting sensor {sensor_id}: {str(e)}", "ERROR")
            return False

    def stop_sensor(self, sensor_id: str) -> bool:
        """Stop the RealSense camera sensor."""
        try:
            self.logging(f"Stopping RealSense sensor: {sensor_id}", "INFO")
            
            # Stop all cameras
            for cam_id in list(self.camera_connections.keys()):
                self.camera_connections[cam_id]['active'] = False
                del self.camera_connections[cam_id]
                self.logging(f"Stopped camera {cam_id}", "INFO")
            
            self.sensor_active = False
            self.logging(f"RealSense sensor {sensor_id} stopped successfully", "INFO")
            return True
            
        except Exception as e:
            self.logging(f"Error stopping sensor {sensor_id}: {str(e)}", "ERROR")
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
            self.logging(f"Error getting data from sensor {sensor_id}: {str(e)}", "ERROR")
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
            self.logging(f"Error creating image message for cam {cam_id}: {str(e)}", "ERROR")
            return None

    def stream(self):
        """Main streaming function - called by timer."""
        if not self.sensor_active:
            return
            
        try:
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
                            self.logging(f"Published scene image from camera {cam_id}", "DEBUG")
                    else:
                        self.logging("No publisher found for scene_image_raw", "WARN")
            else:
                self.logging("No data received from RealSense cameras", "DEBUG")
                
        except Exception as e:
            self.logging(f"Error in stream: {str(e)}", "ERROR")

    def _start_streaming(self):
        """Start the streaming timer."""
        if self.timer is not None:
            self.timer.cancel()
            
        # Calculate timer period from frequency
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.stream)
        
        # Start the RealSense sensor
        self.start_sensor('realsense')
        
        self.logging(f"Started streaming timer at {self.fps}Hz", "INFO")

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
        """Emergency stop - stop streaming and cameras."""
        self.logging("Emergency stop triggered", "WARN")
        
        # Cancel timer
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            
        # Stop all cameras
        self.stop_sensor('realsense')
        
        self.logging("Emergency stop completed", "WARN")

    def wait(self):
        """Wait for any pending operations to complete."""
        if self.timer is not None and self.sensor_active:
            self.logging("Waiting for camera streaming operations to complete", "DEBUG")
