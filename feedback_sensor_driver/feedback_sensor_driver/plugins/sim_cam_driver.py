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

class SimCamDriver(SensorDriverBase):
    """
    ROS2 Driver node for simulated camera sensor in Gazebo input.
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
        self.sensor_active = False
        self.camera_connections = {}
        self.cv_bridge = CvBridge()
        
        self._load_my_config()
        self.start_sensor('sim_cam')

        
        # # Start streaming timer
        # self._start_streaming()


    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            
            # Load plugin parameters from component config
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'sim_cam_driver':
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
            self.handle_error("Failed to load configurations", e)
            
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
                        self.compo_publishers[topic_id] = self.create_publisher(
                            Image,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for {topic_id} with type {msg_type}")
                    else:
                        self.get_logger().warn(f"Unsupported message type: {msg_type} for topic {topic_id}")
            
            self.get_logger().info(f"Registered {len(self.compo_publishers)} publishers")
            
        except Exception as e:
            self.handle_error("Error in publish_msg_pub_sub", e)

    def get_data_from_sensor(self, sensor_id: str) -> Any:
        """Get data from sim camera sensor."""
            
        try:
            sensor_data = {}
            
            # Get data from each active camera
            for cam_id, cam_config in self.camera_connections.items():
                    
                # Generate dummy RGB image data
                # In real implementation, this would capture from RealSense pipeline
                rgb_image = self._generate_dummy_rgb_image(cam_id, cam_config)
                
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

            # Get data from RealSense cameras
            sensor_data = self.get_data_from_sensor('realsense')
            
            if sensor_data is not None:
                # Publish data for each camera
                for cam_key, cam_data in sensor_data.items():
                    rgb_image = cam_data['rgb_image']
                    timestamp = cam_data['timestamp']
                    cam_id = cam_data['cam_id']
                    
                    # Create and publish Image message
                    if 'scene_image_raw' in self.compo_publishers:
                        img_msg = self._create_image_msg(rgb_image, timestamp, cam_id)
                        if img_msg is not None:
                            self.publish_msg('scene_image_raw', img_msg)
                            self.get_logger().debug(f"Published scene image from camera {cam_id}")
                    else:
                        self.get_logger().warn("No publisher found for scene_image_raw")
            else:
                self.get_logger().debug("No data received from RealSense cameras")
                
        except Exception as e:
            self.handle_error("Error in stream", e)

    def register_timer(self):
        if self.compo_timers != []:
            for t in self.compo_timers:
                t.cancel()

        # Calculate timer period from frequency
        timer_period = 1.0 / self.fps
        timer = self.create_timer(timer_period, self.stream)
        self.compo_timers.append(timer)

    def start_component(self):
        if self.compo_timers == [] or not self.compo_timers:
            self.register_timer() #defined in the bottom node
        else:
            for timer in self.compo_timers:
                # if timer.is_canceled():
                timer.reset()
        self.start_sensor('realsense')

    def urgent_stop(self):
        """Emergency stop - stop streaming and cameras."""
        self.get_logger().warn("Emergency stop triggered")
            
        # Stop all cameras
        # self.stop_component()
        ## TODO: consider how to restart sensor when start component
        self.stop_sensor('realsense')
        self.set_state("disconnected")
        
        self.get_logger().warn("Emergency stop completed")
