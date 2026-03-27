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
    # SceneImage,
    Hand2DPlotData,
    RawHandKeypoint
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import threading
import time

class ZMQCompressedImageTransmitter(object):
    """
        .. note::
        Adapted from the Open-Teach teleoperation system:

        | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
        | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
        | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
        |
        | Source: https://github.com/aadhithya14/Open-Teach
    """
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
        self.execution_platform_ip = '192.168.178.54' #'192.168.64.3' #'192.168.178.54'
        self.vr_cam_port = 10505
        self.vr_graph_port = 15001
        self.left_hand_port = 8110
        self.right_hand_port = 8087
        self.button_port = 8095

        self.freq = 90
        
        # Store latest received data
        self.latest_scene_image = None
        self.latest_hand_plot_data = None
        self.bridge = CvBridge()   
        self.rgb_viz_zmq_publisher = None
        self.hand_2d_zmq_publisher = None
        self.raw_keypoint_left_socket = None
        self.raw_keypoint_right_socket = None
        self.button_feedback = None
        self.raw_right_keypoints = None
        self.raw_left_keypoints = None
        self.button_feedback = None

        self._load_my_config()
        # self._register_zmq_sub()
        self._register_zmq_pub()   

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
                        # self.freq = plugin_params.get("freq", self.freq)
                        self.execution_platform_ip = plugin_params.get("execution_platform_ip", self.execution_platform_ip)
                        self.vr_cam_port = plugin_params.get("vr_cam_port", self.vr_cam_port)
                        self.vr_graph_port = plugin_params.get("vr_graph_port", self.vr_graph_port)

                        self.get_logger().info(f"Loaded display config: {self.width}x{self.height}")
                        break
            self.get_logger().info("All configurations loaded successfully")
            
        except Exception as e:
            self.handle_error("Failed to load configurations", e)

    def _register_zmq_pub(self):
        """
            .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        
        if self.rgb_viz_zmq_publisher:
            self.rgb_viz_zmq_publisher.stop()
        self.rgb_viz_zmq_publisher = ZMQCompressedImageTransmitter(
            host = self.execution_platform_ip,
            port = self.vr_cam_port
        )

        self.hand_2d_zmq_publisher = ZMQCompressedImageTransmitter(
            host = self.execution_platform_ip,
            port = self.vr_graph_port
        )
    
    def create_pull_socket(self, host, port):
        """
            .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        context = zmq.Context()
        socket = context.socket(zmq.PULL)
        socket.setsockopt(zmq.CONFLATE, 1)
        socket.setsockopt(zmq.RCVTIMEO, 50)  # 50ms timeout
        socket.bind('tcp://{}:{}'.format(host, port))
        return socket

    # def _register_zmq_sub(self):
    #     self.get_logger().warn('Registering zmq sub for vr detector!!!')
    #     self.raw_keypoint_left_socket = self.create_pull_socket(self.execution_platform_ip, self.left_hand_port)
    #     self.raw_keypoint_right_socket = self.create_pull_socket(self.execution_platform_ip, self.right_hand_port)
    #     self.button_keypoint_socket = self.create_pull_socket(self.execution_platform_ip, self.button_port)

    def register_pub_sub(self):
        """Register subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of manually parsing YAML
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return
                
            # Find topics where this node is a subscriber
            for topic_config in self.data_stream_config["topics"]:
                topic_id = topic_config["id"]
                msg_type = topic_config["msg_type"]

                # # if self.layer_cn_id == topic_config.get("publisher", ''):
                # if msg_type == "component_interfaces/msg/RawHandKeypoint":
                #     self.compo_publishers[topic_id] = self.create_publisher(
                #             RawHandKeypoint, topic_id, 10
                #         )
                #     self.get_logger().info(f"Publisher registered: {topic_id}")
                    

                if self.layer_cn_id in topic_config.get("subscribers", []):
                    # Create subscription based on message type
                    if msg_type == "sensor_msgs/msg/Image":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            Image,
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

            self.get_logger().info("All subscriptions registered successfully")
            
        except Exception as e:
            self.handle_error("Failed to register subscriptions", e)

    # def _extract_data_from_token(self, token):        
    #     data = token.decode().strip()
    #     information = dict()
    #     keypoint_vals = [0] if data.startswith('absolute') else [1]
    #     # Data is in the format <hand>:x,y,z|x,y,z|x,y,z
    #     vector_strings = data.split(':')[1].strip().split('|')
    #     for vector_str in vector_strings:
    #         vector_vals = vector_str.split(',')
    #         for float_str in vector_vals[:3]:
    #             keypoint_vals.append(float(float_str))
            
    #     information['keypoints'] = keypoint_vals
    #     return information
    
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

    def scene_image_callback(self, msg: Image):
        """Callback for processed scene image data"""
        
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_scene_image = self._rescale_image(img, 1)
        self.rgb_viz_zmq_publisher.send_image(self.latest_scene_image)

        # self.raw_right_keypoints = self.raw_keypoint_right_socket.recv()
        # self.raw_left_keypoints = self.raw_keypoint_left_socket.recv()
        # self.button_feedback = self.button_keypoint_socket.recv()

    def hand_plot_callback(self, msg: Hand2DPlotData):
        """Callback for hand 2D plot data"""
        self.pong_reply('handkeypoint_plot_2d',msg)      
        img = self.bridge.imgmsg_to_cv2(msg.plotter, desired_encoding='bgr8')
        self.latest_hand_plot_data = self._rescale_image(img, 4)
        self.hand_2d_zmq_publisher.send_image(self.latest_hand_plot_data) #TODO: change to self.latest_hand_plot_data
        
        # self.raw_right_keypoints = self.raw_keypoint_right_socket.recv()
        # self.raw_left_keypoints = self.raw_keypoint_left_socket.recv()
        # self.button_feedback = self.button_keypoint_socket.recv()
        

    def _rescale_image(self, image, rescale_factor):
        """
            .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        # Ensure RGB format
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Rescale
        width, height = int(img_rgb.shape[1] / rescale_factor), int(img_rgb.shape[0] / rescale_factor)
        img_rescaled = cv2.resize(img_rgb, (width, height), interpolation=cv2.INTER_AREA)

        return img_rescaled

    def periodic_projection(self):
        """Periodically project dummy data for testing"""
                    
        # Project the data

        dummy_image = np.random.randint(0, 255, (self.height, self.width, 3), dtype=np.uint8)
        
        # Create dummy hand plot data
        dummy_hand_plot = np.random.randint(0,255,(128,128,3), dtype=np.uint8)

        # raw_right_keypoints = self.raw_keypoint_right_socket.recv()
        # raw_left_keypoints = self.raw_keypoint_left_socket.recv()
        # button_feedback = self.button_keypoint_socket.recv()
        # if raw_right_keypoints is not None:
        #     # Create and publish RawHandKeypoint message
        #     raw_right_keypoints = self._extract_data_from_token(raw_right_keypoints)
        #     handkeypoint_msg = self._create_raw_hand_keypoint_msg(raw_right_keypoints)
        #     topic_id = 'right_hand_keypoint_raw'
        #     self.publish_msg(topic_id, handkeypoint_msg)
        
        # # Simulate some projection work
        # if self.latest_scene_image is not None:
        #     self.rgb_viz_zmq_publisher.send_image(self.latest_scene_image)
        # else:
        #     self.rgb_viz_zmq_publisher.send_image(dummy_image)

        # if self.latest_hand_plot_data is not None:
        #     self.hand_2d_zmq_publisher.send_image(self.latest_hand_plot_data) #TODO: change to self.latest_hand_plot_data
        # else:
        #     self.hand_2d_zmq_publisher.send_image(dummy_hand_plot)
        # # self.get_logger().info(f"Projecting hand plot")

    
    def register_timer(self):
        if self.compo_timers != []:
            for t in self.compo_timers:
                t.destroy()
            self.compo_timers.clear()

        # # Calculate timer period from frequency
        # timer_period = 1.0 / self.freq
        # timer = self.create_timer(timer_period, self.periodic_projection)
        # self.logging(f"Registered timer at {self.freq}Hz", "INFO")
        # self.compo_timers.append(timer)

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for RenderPCDriver")
        
        # Clear all data
        self.latest_scene_image = None
        self.latest_hand_plot_data = None

        self.rgb_viz_zmq_publisher.stop()
        self.hand_2d_zmq_publisher.stop()

        # self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("RenderPCDriver emergency stopped successfully")
