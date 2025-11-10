import zmq
import cv2
import numpy as np
import threading
from typing import Dict, List, Any, Optional, Union
import json
import rclpy
from rclpy.subscription import Subscription
from core_bases.feedback_projector_driver_base import FeedbackProjectorDriverBase
from cv_bridge import CvBridge

# Import message types based on the interface definitions
from component_interfaces.msg import (
    SceneImage,
    Hand2DPlotData
)
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

class ZMQCompressedImageTransmitter(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        # self._init_push_socket()
        self._init_publisher()

    def _init_publisher(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def _init_push_socket(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.bind('tcp://{}:{}'.format(self._host, self._port))

    def send_image(self, rgb_image):
        _, buffer = cv2.imencode('.jpg', rgb_image, [int(cv2.IMWRITE_WEBP_QUALITY), 10])
        self.socket.send(np.array(buffer).tobytes())

    def stop(self):
        print('Closing the publisher in {}:{}.'.format(self._host, self._port))
        self.socket.close()
        self.context.term()

class VRDriver(FeedbackProjectorDriverBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific driver
        self.width = 1280
        self.height = 720
        self.execution_platform_ip = '192.168.178.54'
        self.vr_cam_port = 10505
        self.vr_graph_port = 15001

        self.freq = 30
        
        # Store latest received data
        self.latest_scene_image = None
        self.latest_hand_plot_data = None
        self.bridge = CvBridge()
        self.rgb_viz_zmq_publisher = ZMQCompressedImageTransmitter(
            host = self.execution_platform_ip,
            port = self.vr_cam_port
        )

        self.hand_2d_zmq_publisher = ZMQCompressedImageTransmitter(
            host = self.execution_platform_ip,
            port = self.vr_graph_port
        )        
        # Dummy projection parameters
        self.last_projection_time = self.get_clock().now()

        self._load_my_config()

    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if not self.my_config:
                self.get_logger().warn(f"No component config found for {self.layer_cn_id}")
                self.my_config = {}
            
            # Override display parameters from component config if available
            if self.my_config and "plugins" in self.my_config:
                for plugin in self.my_config["plugins"]:
                    if plugin.get("id") == "vr_driver":
                        plugin_params = plugin.get("params", {})
                        self.width = plugin_params.get("width", self.width)
                        self.height = plugin_params.get("height", self.height)
                        self.freq = plugin_params.get("freq", self.freq)
                        # self.execution_platform_ip = plugin_params.get("execution_platform_ip", self.execution_platform_ip)
                        # self.vr_port = plugin_params.get("vr_port", self.vr_port)
                        self.get_logger().info(f"Loaded display config: {self.width}x{self.height}")
                        break
            self.get_logger().info("All configurations loaded successfully")
            
        except Exception as e:
            self.handle_error("Failed to load configurations", e)

    def register_pub_sub(self):
        """Register subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of manually parsing YAML
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return
                
            # Find topics where this node is a subscriber
            for topic_config in self.data_stream_config["topics"]:
                if self.layer_cn_id in topic_config.get("subscribers", []):
                    topic_id = topic_config["id"]
                    msg_type = topic_config["msg_type"]
                    
                    # Create subscription based on message type
                    if msg_type == "component_interfaces/msg/SceneImage":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            SceneImage,
                            topic_id,
                            self.scene_image_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to {topic_id}")
                        
                    elif msg_type == "component_interfaces/msg/Hand2DPlotData":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            Hand2DPlotData,
                            topic_id,
                            self.hand_plot_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to {topic_id}")
            
            # self.rgb_viz_zmq_publisher = ZMQCompressedImageTransmitter(
			# 	host = self.execution_platform_ip,
			# 	port = self.vr_port + 500
			# )
            self.get_logger().info("All subscriptions registered successfully")
            
        except Exception as e:
            self.handle_error("Failed to register subscriptions", e)

    def scene_image_callback(self, msg: SceneImage):
        """Callback for processed scene image data"""       
        self.latest_scene_image = self.bridge.imgmsg_to_cv2(msg.feedback_image, desired_encoding='bgr8')
        self.get_logger().debug(f"Received scene image from {msg.image_source}")

    def hand_plot_callback(self, msg: Hand2DPlotData):
        """Callback for hand 2D plot data"""      
        self.latest_hand_plot_data = msg
        self.get_logger().debug(f"Received hand plot data for {msg.hand_id}")

    # def _rescale_image(self, image, rescale_factor):
    #     width, height = int(image.shape[1] / rescale_factor), int(image.shape[0] / rescale_factor)
    #     img = cv2.resize(image, (width, height), interpolation = cv2.INTER_AREA)
    #     cv2.imshow('dummy rgb image', img)
    #     return img

    def _rescale_image(self, image, rescale_factor):
        # Ensure RGB format
        if len(image.shape) == 2:  # grayscale
            img_rgb = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        else:  # assume BGR (OpenCV default)
            img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Rescale
        width, height = int(img_rgb.shape[1] / rescale_factor), int(img_rgb.shape[0] / rescale_factor)
        img_rescaled = cv2.resize(img_rgb, (width, height), interpolation=cv2.INTER_AREA)

        # # Display (OpenCV expects BGR)
        # cv2.imshow('dummy rgb image', cv2.cvtColor(img_rescaled, cv2.COLOR_RGB2BGR))
        # cv2.waitKey(1)
        return img_rescaled

    def project_feedback(self, projector_id: str, data: Dict[str, Any]):
        """
        Dummy implementation of projection method
        TODO: need to use zmq publisher to pub the scene image, see the image.py
        """
        self.get_logger().info(f"Projecting data on {projector_id}")
        self.get_logger().info(f"Data keys: {list(data.keys())}")
        
        # Simulate some projection work
        if "image" in data:
            self.rgb_viz_zmq_publisher.send_image(self._rescale_image(data['image'], 2))
            self.get_logger().info(f"Projecting image with shape {data['image'].shape if hasattr(data['image'], 'shape') else 'unknown'}")
        
        if "hand_plot" in data:
            self.hand_2d_zmq_publisher.send_image(self._rescale_image(data['hand_plot'], 1))
            self.get_logger().info(f"Projecting hand plot")
        
        # Simulate projection success
        return True

    def periodic_projection(self):
        """Periodically project dummy data for testing"""
            
        current_time = self.get_clock().now()            
        # Create dummy projection data
        projection_data = {
            "timestamp": current_time.to_msg(),
            "projector_id": "vr_display",
            "content": self.generate_dummy_projection_content()
        }
        
        # Project the data
        success = self.project_feedback("vr", projection_data["content"])
        
        if success:
            self.get_logger().debug("Dummy projection completed successfully")
        
        self.last_projection_time = current_time

    def generate_dummy_projection_content(self) -> Dict[str, Any]:
        """Generate dummy projection content for testing"""
        # # Create a dummy image
        dummy_image = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
        
        # Create dummy hand plot data
        dummy_hand_plot = np.random.randint(0,255,(128,128,3), dtype=np.uint8)
        
        return {
            "image": dummy_image, #self.latest_scene_image, #TODO: change to self.latest_scene_image
            "hand_plot": dummy_hand_plot,
            "timestamp": self.get_clock().now().to_msg().sec
        }

    
    def register_timer(self):
        if self.compo_timers != []:
            for t in self.compo_timers:
                t.destroy()
            self.compo_timers.clear()

        # Calculate timer period from frequency
        timer_period = 1.0 / self.freq
        timer = self.create_timer(timer_period, self.periodic_projection)
        self.logging(f"Registered timer at {self.freq}Hz", "INFO")
        self.compo_timers.append(timer)

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RenderPCDriver")
        
        # Clear all data
        self.latest_scene_image = None
        self.latest_robot_state = None
        self.latest_hand_plot_data = None
        
        # self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("RenderPCDriver emergency stopped successfully")