from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional
import rclpy
import json
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

# Import message types based on the interface definitions
from component_interfaces.msg import (
    # SceneImage,
    # RobotState,
    Hand2DPlotData,
    ProcessedHandKeypoint
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

from cv_bridge import CvBridge

import cv2
import matplotlib
matplotlib.use("Agg")   # No GUI backend
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas

class PlotHand2D():
    def __init__(self):
        # Create figure and canvas once
        self.fig = plt.figure(figsize=(6, 6), dpi=60)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)

        self.bound_update_counter = 0

        self.OCULUS_JOINTS = {
            'metacarpals': [2, 6, 9, 12, 15],
            'knuckles': [6, 9, 12, 16],
            'thumb':   [2, 3, 4, 5, 19],
            'index':   [6, 7, 8, 20],
            'middle':  [9, 10, 11, 21],
            'ring':    [12, 13, 14, 22],
            'pinky':   [15, 16, 17, 18, 23]
        }

        self.thumb_bounds = np.array([
            [ 0.07147029,  0.10102608],
            [ 0.06931292,  0.06902901],
            [ 0.02568949,  0.09142401],
            [ 0.00698855,  0.08615090],
            [-0.00888068,  0.08024863],
            [-0.00749210,  0.09412790],
            [ 0.00949221,  0.10919102],
            [ 0.03664591,  0.11207025]
        ])

    def plot_line(self, X1, X2, Y1, Y2):
        self.ax.plot([X1, X2], [Y1, Y2], color="black")

    def _set_limits(self):
        self.ax.set_xlim(-0.12, 0.12)
        self.ax.set_ylim(-0.02, 0.2)

    def _draw_thumb_bounds(self):
        # Draw polygon
        n = len(self.thumb_bounds)
        for i in range(n):
            x1, y1 = self.thumb_bounds[i]
            x2, y2 = self.thumb_bounds[(i + 1) % n]
            self.plot_line(x1, x2, y1, y2)

    def draw_hand(self, X, Y):
        # plot points
        self.ax.plot(X, Y, 'ro')

        # thumb constraint
        if self.thumb_bounds is not None:
            self._draw_thumb_bounds()

        # Wrist to knuckles
        for idx in self.OCULUS_JOINTS['metacarpals']:
            self.plot_line(X[0], X[idx], Y[0], Y[idx])

        # Joints inside each finger
        for key in ['knuckles', 'thumb', 'index', 'middle', 'ring', 'pinky']:
            joints = self.OCULUS_JOINTS[key]
            for i in range(len(joints) - 1):
                a, b = joints[i], joints[i+1]
                self.plot_line(X[a], X[b], Y[a], Y[b])

    def draw(self, X, Y):
        # Clear previous frame
        self.ax.clear()

        # Plot settings
        self._set_limits()

        # Thumb bounds update
        if self.bound_update_counter % 10 == 0:
            self.bound_update_counter = 0
            # (your update logic) self._check_thumb_bounds()
        self.bound_update_counter += 1

        # Draw hand
        self.draw_hand(X, Y)

        # Render figure to numpy array
        self.canvas.draw()
        
        # Use the modern method instead of deprecated tostring_rgb()
        buf = np.frombuffer(self.canvas.buffer_rgba(), dtype=np.uint8)
        w, h = self.fig.canvas.get_width_height()
        img = buf.reshape(h, w, 4)   # RGBA image
        
        # Convert RGBA to RGB by removing alpha channel
        img = img[:, :, :3]

        return img
    
class MultiFeedbackDataProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific processor
        self.processing_types = ["hand_2d_plot"]
        self.processing_counter = 0
        
        self.bridge = CvBridge() 
        
        # Store latest received data
        self.latest_hand_keypoints = None
        # self.latest_scene_image = None
        # self.latest_robot_state = None
        
        # # Initialize processing parameters
        self._load_my_config()

        self.plotter2D = PlotHand2D()


    def _load_my_config(self):
            """Load all required configurations using ComponentNode methods"""
            try:
                # Load component-specific configuration
                self.my_config = self.load_my_config()
                if not self.my_config:
                    self.get_logger().warn(f"No component config found for {self.layer_cn_id}")
                    self.my_config = {}
                
                # Override processing types from component config if available
                if self.my_config and "plugins" in self.my_config:
                    for plugin in self.my_config["plugins"]:
                        if plugin.get("id") == "multi_feedback":
                            plugin_params = plugin.get("params", {})
                            if "types" in plugin_params:
                                self.processing_types = plugin_params["types"]
                                self.get_logger().info(f"Loaded processing types from config: {self.processing_types}")
                            break
                            
                self.get_logger().info("All configurations loaded successfully")
                
            except Exception as e:
                self.handle_error("Failed to load configurations", e)

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of manually parsing YAML
            if not self.data_stream_config or "topics" not in self.data_stream_config:
                self.handle_error("Data stream configuration not available or invalid")
                return
                
            # Find topics where this node is a subscriber or publisher
            for topic_config in self.data_stream_config["topics"]:
                topic_id = topic_config["id"]
                
                # Subscribe to topics where this node is a subscriber
                if self.layer_cn_id in topic_config.get("subscribers", []):
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            ProcessedHandKeypoint,
                            topic_id,
                            self.hand_keypoint_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to hand keypoints: {topic_id}")
                    
                    # elif msg_type == "sensor_msgs/msg/Image":
                    #     self.compo_subscribers[topic_id] = self.create_subscription(
                    #         Image,
                    #         topic_id,
                    #         self.scene_image_callback,
                    #         10
                    #     )
                    #     self.get_logger().info(f"Subscribed to scene image: {topic_id}")
                    
                    # elif msg_type == "component_interfaces/msg/RobotState":
                    #     self.compo_subscribers[topic_id] = self.create_subscription(
                    #         RobotState,
                    #         topic_id,
                    #         self.robot_state_callback,
                    #         10
                    #     )
                    #     self.get_logger().info(f"Subscribed to robot state: {topic_id}")
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    # if msg_type == "component_interfaces/msg/SceneImage":
                    #     self.compo_publishers[topic_id] = self.create_publisher(
                    #         SceneImage,
                    #         topic_id,
                    #         10
                    #     )
                    #     self.get_logger().info(f"Created publisher for processed scene image: {topic_id}")
                    
                    if msg_type == "component_interfaces/msg/Hand2DPlotData":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            Hand2DPlotData,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for hand 2D plot: {topic_id}")
                    
                    # elif msg_type == "component_interfaces/msg/RobotState":
                    #     self.compo_publishers[topic_id] = self.create_publisher(
                    #         RobotState,
                    #         topic_id,
                    #         10
                    #     )
                    #     self.get_logger().info(f"Created publisher for processed robot state: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error("Failed to register publishers/subscribers", e)

    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""

        ##NOTE
        # if self.state!='running':
        #     return
        self.latest_hand_keypoints = msg
        self.get_logger().debug(f"Received processed hand keypoints with joints")
        
        # Process hand data for 2D plotting
        if "hand_2d_plot" in self.processing_types:
            self.process_hand_data(msg)

    # def scene_image_callback(self, msg: Image):
    #     """Callback for raw scene image data"""
    #     ##NOTE
    #     # if self.state!='running':
    #     #     return
    #     self.latest_scene_image = msg
    #     self.get_logger().debug("Received raw scene image")
        
    #     # Process scene image
    #     if "scene_img" in self.processing_types:
    #         self.process_scene_image(msg)

    # def robot_state_callback(self, msg: RobotState):
    #     """Callback for robot state data"""

    #     ##NOTE
    #     # if self.state!='running':
    #     #     return
            
    #     self.latest_robot_state = msg
    #     self.get_logger().debug(f"Received robot state for {msg.robot_id}")
        
    #     # Process robot state for plotting
    #     if "robot_state_plot" in self.processing_types:
    #         self.process_robot_state(msg)

    def process_hand_data(self, hand_data: ProcessedHandKeypoint):
        """Process hand keypoints into 2D plot data"""
        try:
            self.get_logger().debug(f"Processing hand data")
            
            plot_msg = Hand2DPlotData()
            plot_msg.header = hand_data.header
            plot_msg.header.stamp = self.get_clock().now().to_msg()
            plot_msg.hand_id = "right_hand"

            keypoints = np.array(hand_data.hand_coords).reshape(24, 3)
            plot_img = self.plotter2D.draw(keypoints[:, 0], keypoints[:, 1])
            plot_img_bgr = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)
            plot_msg.plotter = self.bridge.cv2_to_imgmsg(plot_img_bgr, encoding='bgr8') 
            
            # Publish processed data
            self.publish_msg("handkeypoint_plot_2d", plot_msg)
            
            self.get_logger().debug("Hand data processing completed successfully")
            
        except Exception as e:
            self.handle_error(f"Error processing hand data: ", e)

    # def process_scene_image(self, image_data: Image):
    #     """Process raw scene image"""
    #     try:
    #         self.processing_counter += 1
    #         self.get_logger().debug(f"Processing scene image #{self.processing_counter}")
            
    #         # Create processed scene image
    #         processed_image = self._generate_processed_image(image_data)
            
    #         # Publish processed data
    #         self.publish_msg("scene_image_processed", processed_image)
            
    #         self.get_logger().debug("Scene image processing completed successfully")
            
    #     except Exception as e:
    #         self.handle_error(f"Error processing scene image: ", e)

    # def process_robot_state(self, robot_data: RobotState):
    #     """Process robot state for visualization"""
    #     try:
    #         self.processing_counter += 1
    #         self.get_logger().debug(f"Processing robot state #{self.processing_counter}")
            
    #         # Create processed robot state
    #         processed_state = self._generate_processed_robot_state(robot_data)
            
    #         # Publish processed data
    #         self.publish_msg("robot_state_processed", processed_state)
            
    #         self.get_logger().debug("Robot state processing completed successfully")
            
    #     except Exception as e:
    #         self.handle_error("Error processing robot state: ", e)


    # def _generate_processed_image(self, image_data: Image) -> SceneImage:
    #     """Generate processed scene image"""
    #     processed_msg = SceneImage()
    #     processed_msg.header = image_data.header
    #     processed_msg.header.stamp = self.get_clock().now().to_msg()
    #     processed_msg.image_source = "realsense_camera"
    #     processed_msg.feedback_image = image_data  # In real implementation, would process the image
        
    #     return processed_msg

    # def _generate_processed_robot_state(self, robot_data: RobotState) -> RobotState:
    #     """Generate processed robot state"""
    #     processed_msg = RobotState()
    #     processed_msg.header = robot_data.header
    #     processed_msg.header.stamp = self.get_clock().now().to_msg()
    #     processed_msg.robot_id = robot_data.robot_id
    #     processed_msg.joint_names = robot_data.joint_names
    #     processed_msg.joint_states = robot_data.joint_states
    #     processed_msg.end_effector_pose = robot_data.end_effector_pose
        
    #     # Add some dummy processing (e.g., calculate velocity, etc.)
    #     return processed_msg

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for MultiFeedbackDataProcessor")
        
        # Clear data
        self.latest_hand_keypoints = None
        # self.latest_scene_image = None
        # self.latest_robot_state = None
        
        # self.stop_component()

        self.set_state("disconnected")
        
        self.get_logger().info("MultiFeedbackDataProcessor emergency stopped successfully")