# from core_bases.motion_retarget_base import MotionRetargetBase
# from typing import Dict, List, Any, Optional
# import rclpy
# import yaml
# import numpy as np
# from rclpy.publisher import Publisher
# from rclpy.subscription import Subscription
# import tf2_ros
# import tf2_geometry_msgs
# import math
# import time
# from scipy.spatial.transform import Rotation, Slerp
# from scipy.spatial.transform import Rotation as R
# from numpy.linalg import pinv

# # Import message types based on the interface definitions
# from component_interfaces.msg import (
#     ProcessedHandKeypoint,
#     ControlCmd,
#     RobotState
# )
# from std_msgs.msg import Header
# from geometry_msgs.msg import Pose, Point, Transform, TransformStamped, Quaternion, PoseStamped
# from builtin_interfaces.msg import Time, Duration
# from copy import copy, deepcopy

# np.set_printoptions(precision=3, suppress=True)

# # Oculus joint indices (adjust based on your system)
# OCULUS_JOINTS = {
#     'thumb': [0, 1, 2, 3, 4],
#     'index': [5, 6, 7, 8, 9],
#     'middle': [10, 11, 12, 13, 14],
#     'ring': [15, 16, 17, 18, 19],
#     'pinky': [20, 21, 22, 23, 24]
# }

# # Filter for removing noise in the teleoperation
# class Filter:
#     def __init__(self, state, comp_ratio=0.6):
#         self.pos_state = state[:3]
#         self.ori_state = state[3:6]
#         self.comp_ratio = comp_ratio

#     def __call__(self, next_state):
#         self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
#         ori_interp = Slerp([0, 1], Rotation.from_rotvec(
#             np.stack([self.ori_state, next_state[3:6]], axis=0)),)
#         self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
#         return np.concatenate([self.pos_state, self.ori_state])

# class MotionRetargetOT(MotionRetargetBase):
#     def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
#         super().__init__(layer_cn_id, ros2_domain_id)
        
#         # Additional attributes for this specific retargeter
#         self.processing_frequency = 90  # Hz
#         self.retargeting_counter = 0
        
#         self.latest_robot_state = None
#         self.robot_init_H = None
#         self.hand_init_H = None
#         self.hand_init_t = None
#         self.hand_moving_H = None
#         self.robot_moving_H = None
        
#         # Use filter
#         self.use_filter = True
#         self.comp_filter = None
        
#         # Class variables for gripper and pause control
#         self.gripper_flag = 1
#         self.pause_flag = 1
#         self.prev_pause_flag = 0
#         self.is_first_frame = True
#         self.gripper_cnt = 0
#         self.prev_gripper_flag = 0
#         self.pause_cnt = 0
#         self.gripper_correct_state = 1
#         self.resolution_scale = 1.0
        
#         # Latest hand data
#         self.latest_hand_coords = None
#         self.latest_hand_frame = None

#         # Initialize retargeting parameters
#         self._load_my_config()

#     def _load_my_config(self):
#         """Load all required configurations using ComponentNode methods"""
#         try:
#             # Load component-specific configuration
#             self.my_config = self.load_my_config()
#             if self.my_config and "plugins" in self.my_config:
#                 plugins = self.my_config["plugins"]
#                 for plugin in plugins:
#                     if plugin.get('id') == 'motion_retarget_ot':
#                         plugin_params = plugin.get('params', {})
#                         self.processing_frequency = plugin_params.get("freq", self.processing_frequency)
#                         self.use_filter = plugin_params.get("use_filter", self.use_filter)
#                         self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
#                         break
                
#         except Exception as e:
#             self.handle_error(f"Failed to load configurations: {str(e)}")

#     def register_pub_sub(self):
#         """Register publishers and subscribers based on data_stream configuration"""
#         try:
#             # Use the loaded data_stream_config instead of loading again
#             if not self.data_stream_config or 'topics' not in self.data_stream_config:
#                 self.get_logger().warn("No data stream configuration available")
#                 return
            
#             # Find topics where this node is a subscriber
#             for topic_config in self.data_stream_config['topics']:
#                 topic_id = topic_config["id"]
                
#                 # Subscribe to topics where this node is a subscriber
#                 if self.layer_cn_id in topic_config.get("subscribers", []):
#                     msg_type = topic_config["msg_type"]
                    
#                     if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
#                         self.compo_subscribers[topic_id] = self.create_subscription(
#                             ProcessedHandKeypoint,
#                             topic_id,
#                             self.hand_keypoint_callback,
#                             10
#                         )
#                         self.get_logger().info(f"Subscribed to hand keypoints: {topic_id}")
#                     elif msg_type == "component_interfaces/msg/RobotState":
#                         self.compo_subscribers[topic_id] = self.create_subscription(
#                             RobotState,
#                             topic_id,
#                             self.robot_state_callback,
#                             10
#                         )
                
#                 # Create publishers for topics where this node is the publisher
#                 if topic_config.get("publisher") == self.layer_cn_id:
#                     msg_type = topic_config["msg_type"]
                    
#                     if msg_type == "component_interfaces/msg/ControlCmd":
#                         self.compo_publishers[topic_id] = self.create_publisher(
#                             ControlCmd,
#                             topic_id,
#                             10
#                         )
#                         self.get_logger().info(f"Created publisher for control cmd: {topic_id}")
        
#             self.get_logger().info("All publishers and subscribers registered successfully")
            
#         except Exception as e:
#             self.handle_error(f"Failed to register publishers/subscribers: {str(e)}")
            
#     def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
#         """Callback for processed hand keypoint data"""
#         self.get_logger().debug(f"Received processed hand keypoints")

#         self.pong_reply("right_hand_keypoint_processed", msg)
        
#         # Store latest hand data
#         self.latest_hand_coords = np.array(msg.hand_coords).reshape(-1, 3)  # Reshape to (N, 3)
#         self.latest_hand_frame = np.array(msg.hand_frame).reshape(4, 3)  # (4, 3) format
        
#         # Retarget the hand data to robot commands
#         self.retarget_hand_data(msg)

#     def robot_state_callback(self, msg: RobotState):
#         """Callback for robot state updates"""
#         pose = msg.end_effector_pose
#         x = pose.position.x
#         y = pose.position.y
#         z = pose.position.z

#         qx = pose.orientation.x
#         qy = pose.orientation.y
#         qz = pose.orientation.z
#         qw = pose.orientation.w

#         # Rotation matrix from quaternion
#         R_mat = np.array([
#             [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
#             [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
#             [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
#         ], dtype=np.float64)

#         # Homogeneous transform
#         T = np.eye(4, dtype=np.float64)
#         T[:3, :3] = R_mat
#         T[:3, 3] = [x, y, z]

#         self.latest_robot_state = copy(T)

#     def get_robot_init(self):
#         """Non-blocking method to get robot initial state"""
#         if self.latest_robot_state is None:
#             self.get_logger().warn("No robot state available yet")
#             return False
        
#         self.robot_init_H = copy(self.latest_robot_state)
        
#         # Initialize filter if needed
#         if self.use_filter and self.comp_filter is None:
#             robot_init_cart = self._homo2cart(self.robot_init_H)
#             self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)
#             self.get_logger().info("Filter initialized")
        
#         return True

#     def retarget_hand_data(self, hand_data: ProcessedHandKeypoint):
#         """Retarget hand keypoints to robot commands"""
#         try:
#             self.retargeting_counter += 1
#             self.get_logger().debug(f"Retargeting hand data #{self.retargeting_counter}")
            
#             # Check for pause/resume state
#             pause_state, pause_status, pause_right = self.get_pause_state_from_hand_keypoints()
            
#             # Check if we need to reset teleop (first frame or resuming from pause)
#             if self.is_first_frame or (self.prev_pause_flag == 0 and pause_state == 1):
#                 if not self._reset_teleop():
#                     return  # Return if reset fails
            
#             self.prev_pause_flag = pause_state
            
#             # If paused, don't send commands
#             if pause_state == 0:
#                 self.get_logger().debug("Teleoperation paused")
#                 return
            
#             # Generate control commands
#             cmd_dict = self.retarget_control_to_robot_space(hand_data)
#             if cmd_dict:
#                 self.publish_robot_commands(cmd_dict)
            
#             self.get_logger().debug("Hand data retargeting completed successfully")
            
#         except Exception as e:
#             self.handle_error(f"Error retargeting hand data: {str(e)}")

#     def _reset_teleop(self):
#         """Reset teleoperation and make the current frame as initial frame"""
#         self.get_logger().info('****** RESETTING TELEOP ******')
        
#         # Get robot initial homogeneous matrix
#         if not self.get_robot_init():
#             return False
        
#         # Get hand initial frame
#         if self.latest_hand_frame is None:
#             self.get_logger().warn("No hand frame available for reset")
#             return False
        
#         self.hand_init_H = self._turn_frame_to_homo_mat(self.latest_hand_frame)
#         self.hand_init_t = copy(self.hand_init_H[:3, 3])
#         self.is_first_frame = False
        
#         self.get_logger().info("Resetting complete")
#         return True

#     def _turn_frame_to_homo_mat(self, frame):
#         """Turn a frame to a homogeneous matrix"""
#         t = frame[0]
#         R_mat = frame[1:]

#         homo_mat = np.zeros((4, 4))
#         homo_mat[:3, :3] = np.transpose(R_mat)
#         homo_mat[:3, 3] = t
#         homo_mat[3, 3] = 1

#         return homo_mat

#     def _homo2cart(self, homo_mat):
#         """Turn homogeneous matrix to cartesian vector (position + axis-angle)"""
#         t = homo_mat[:3, 3]
#         R_mat = Rotation.from_matrix(homo_mat[:3, :3]).as_rotvec(degrees=False)
#         cart = np.concatenate([t, R_mat], axis=0)
#         return cart

#     def _get_scaled_cart_pose(self, moving_robot_homo_mat):
#         """Get the scaled cartesian pose based on resolution scale"""
#         # Get the cart pose without the scaling
#         unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

#         # Get the current cart pose from latest robot state
#         if self.latest_robot_state is None:
#             self.get_logger().warn("No robot state available")
#             return unscaled_cart_pose
        
#         current_cart_pose = self._homo2cart(self.latest_robot_state)

#         # Get the difference in translation between these two cart poses
#         diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
#         scaled_diff_in_translation = diff_in_translation * self.resolution_scale
        
#         scaled_cart_pose = np.zeros(6)
#         scaled_cart_pose[3:] = unscaled_cart_pose[3:]  # Get the rotation directly
#         scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation  # Get the scaled translation only

#         return scaled_cart_pose

#     def get_gripper_state_from_hand_keypoints(self):
#         """Get gripper state from hand keypoints (pinch detection)"""
#         if self.latest_hand_coords is None:
#             return self.gripper_flag, False, False
        
#         try:
#             # Calculate distance between pinky tip and thumb tip
#             pinky_tip = self.latest_hand_coords[OCULUS_JOINTS['pinky'][-1]]
#             thumb_tip = self.latest_hand_coords[OCULUS_JOINTS['thumb'][-1]]
#             distance = np.linalg.norm(pinky_tip - thumb_tip)
            
#             thresh = 0.03
#             gripper_fl = False
            
#             if distance < thresh:
#                 self.gripper_cnt += 1
#                 if self.gripper_cnt == 1:
#                     self.prev_gripper_flag = self.gripper_flag
#                     self.gripper_flag = not self.gripper_flag 
#                     gripper_fl = True
#             else: 
#                 self.gripper_cnt = 0
            
#             gripper_state = np.asanyarray(self.gripper_flag).reshape(1)[0]
#             status = False  
#             if gripper_state != self.prev_gripper_flag:
#                 status = True
            
#             return gripper_state, status, gripper_fl
#         except Exception as e:
#             self.get_logger().error(f"Error in gripper state detection: {str(e)}")
#             return self.gripper_flag, False, False

#     def get_pause_state_from_hand_keypoints(self):
#         """Toggle pause/resume using ring/middle finger pinch"""
#         if self.latest_hand_coords is None:
#             return self.pause_flag, False, True
        
#         try:
#             # Calculate distances
#             ring_tip = self.latest_hand_coords[OCULUS_JOINTS['ring'][-1]]
#             middle_tip = self.latest_hand_coords[OCULUS_JOINTS['middle'][-1]]
#             thumb_tip = self.latest_hand_coords[OCULUS_JOINTS['thumb'][-1]]
            
#             ring_distance = np.linalg.norm(ring_tip - thumb_tip)
#             middle_distance = np.linalg.norm(middle_tip - thumb_tip)
            
#             thresh = 0.03 
#             pause_right = True
            
#             if ring_distance < thresh or middle_distance < thresh:
#                 self.pause_cnt += 1
#                 if self.pause_cnt == 1:
#                     self.prev_pause_flag = self.pause_flag
#                     self.pause_flag = not self.pause_flag       
#             else:
#                 self.pause_cnt = 0
            
#             pause_state = np.asanyarray(self.pause_flag).reshape(1)[0]
#             pause_status = False  
#             if pause_state != self.prev_pause_flag:
#                 pause_status = True 
            
#             return pause_state, pause_status, pause_right
#         except Exception as e:
#             self.get_logger().error(f"Error in pause state detection: {str(e)}")
#             return self.pause_flag, False, True

#     def retarget_control_to_robot_space(self, hand_data=None):
#         """
#         Generate control commands using bimanual_right.py logic
#         """
#         try:
#             # Check if initialization is complete
#             if self.robot_init_H is None or self.hand_init_H is None:
#                 self.get_logger().warn("Not initialized yet")
#                 return None
            
#             if self.latest_hand_frame is None:
#                 self.get_logger().warn("No hand frame available")
#                 return None
            
#             # Get moving hand frame
#             self.hand_moving_H = self._turn_frame_to_homo_mat(self.latest_hand_frame)
            
#             # Transformation code (from bimanual_right.py)
#             H_HI_HH = copy(self.hand_init_H)  # Homo matrix: P_HI to P_HH
#             H_HT_HH = copy(self.hand_moving_H)  # Homo matrix: P_HT to P_HH
#             H_RI_RH = copy(self.robot_init_H)  # Homo matrix: P_RI to P_RH
            
#             H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH  # Homo matrix: P_HT to P_HI
            
#             # Rotation transformation matrix
#             H_R_V = np.array([[0, 0, -1, 0], 
#                              [0, -1, 0, 0],
#                              [-1, 0, 0, 0],
#                              [0, 0, 0, 1]])
            
#             # Translation transformation matrix
#             H_T_V = np.array([[0, 0, 1, 0],
#                              [0, 1, 0, 0],
#                              [-1, 0, 0, 0],
#                              [0, 0, 0, 1]])
            
#             H_HT_HI_r = (pinv(H_R_V) @ H_HT_HI @ H_R_V)[:3, :3]
#             H_HT_HI_t = (pinv(H_T_V) @ H_HT_HI @ H_T_V)[:3, 3]
            
#             relative_affine = np.block(
#                 [[H_HT_HI_r, H_HT_HI_t.reshape(3, 1)], [0, 0, 0, 1]])
            
#             target_translation = H_RI_RH[:3, 3] + relative_affine[:3, 3]
#             target_rotation = H_RI_RH[:3, :3] @ relative_affine[:3, :3]
            
#             H_RT_RH = np.block(
#                 [[target_rotation, target_translation.reshape(-1, 1)], [0, 0, 0, 1]])
            
#             self.robot_moving_H = copy(H_RT_RH)
#             final_pose = self._get_scaled_cart_pose(self.robot_moving_H)
            
#             # Apply the filter
#             if self.use_filter and self.comp_filter is not None:
#                 final_pose = self.comp_filter(final_pose)
            
#             # Get gripper state
#             gripper_state, status_change, gripper_flag = self.get_gripper_state_from_hand_keypoints()
#             if gripper_flag == 1 and status_change is True:
#                 self.gripper_correct_state = gripper_state
            
#             # Create header
#             header = Header(stamp=self.get_clock().now().to_msg(), frame_id="xarm5")
            
#             # --- Robot cartesian command ---
#             robot_cmd = ControlCmd()
#             robot_cmd.header = header
#             robot_cmd.robot_id = "xarm5"
#             robot_cmd.relative = False  # Using absolute pose based on retargeting logic
#             robot_cmd.space = "cartesian"
#             robot_cmd.joint_names = []
#             robot_cmd.joint_angles = []
#             robot_cmd.joint_velocities = []
            
#             # Set pose from final_pose (position in meters, orientation as axis-angle)
#             robot_cmd.pose = Pose()
#             robot_cmd.pose.position.x = float(final_pose[0])
#             robot_cmd.pose.position.y = float(final_pose[1])
#             robot_cmd.pose.position.z = float(final_pose[2])
            
#             # Convert axis-angle to quaternion
#             rot = R.from_rotvec(final_pose[3:6])
#             quat = rot.as_quat()  # Returns [x, y, z, w]
#             robot_cmd.pose.orientation.x = float(quat[0])
#             robot_cmd.pose.orientation.y = float(quat[1])
#             robot_cmd.pose.orientation.z = float(quat[2])
#             robot_cmd.pose.orientation.w = float(quat[3])
            
#             robot_cmd.timeout = Duration(sec=5)
            
#             # --- Gripper command ---
#             grip_cmd = ControlCmd()
#             grip_cmd.header = header
#             grip_cmd.robot_id = "xarm_gripper"
#             grip_cmd.relative = False
#             grip_cmd.space = "joint"
#             grip_cmd.joint_names = ["drive_joint"]
#             # Scale gripper state (0 or 1) to joint angle (0.0 to 0.8 or 1.0)
#             grip_cmd.joint_angles = [float(self.gripper_correct_state * 0.8)]
#             grip_cmd.joint_velocities = [0.3]
#             grip_cmd.pose = Pose()
#             grip_cmd.timeout = Duration(sec=2)
            
#             return {
#                 "robot_cmd": robot_cmd,
#                 "gripper_cmd": grip_cmd,
#             }
            
#         except Exception as e:
#             self.handle_error(f"Error in retarget_control_to_robot_space: {str(e)}")
#             return None

#     def publish_robot_commands(self, cmd_dict):
#         """Publish given ControlCmd messages to their respective topics"""
#         try:
#             if "robot_cmd" in cmd_dict:
#                 self.publish_msg("robot_control_cmd", cmd_dict["robot_cmd"])
#                 self.get_logger().debug("Published robot control command")

#             if "gripper_cmd" in cmd_dict:
#                 self.publish_msg("gripper_control_cmd", cmd_dict["gripper_cmd"])
#                 self.get_logger().debug("Published gripper control command")

#         except Exception as e:
#             self.handle_error(f"Error publishing control commands: {str(e)}")

#     def urgent_stop(self):
#         """Emergency stop implementation"""
#         self.get_logger().warn("URGENT STOP triggered for MotionRetargetOT")
        
#         # # Send stop command to robot (hold current position)
#         # stop_msg = ControlCmd()
#         # stop_msg.header = Header()
#         # stop_msg.header.stamp = self.get_clock().now().to_msg()
#         # stop_msg.robot_id = "xarm5"
#         # stop_msg.joint_names = self.robot_joint_names
#         # stop_msg.joint_angles = [0.0] * len(self.robot_joint_names)
#         # stop_msg.timeout = Duration(sec=0, nanosec=0)
        
#         self.set_state("disconnected")
        
#         self.get_logger().info("MotionRetargetOT emergency stopped successfully")


from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional, Tuple
import rclpy
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from scipy.spatial.transform import Rotation as R

# Import message types
from component_interfaces.msg import (
    ProcessedHandKeypoint,
    RobotState,
    ControlCmd
)
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


class MotionRetargetOT(DataProcessorBase):
    """
    Motion retargeting processor that maps VR hand movements to robot end-effector commands.
    Transforms hand coordinates and orientation to robot cartesian commands.
    Also maps hand gestures (finger pinching) to gripper control.
    """
    
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Retargeting parameters
        self.position_scale = 1.0  # Scale factor for position mapping
        self.rotation_scale = 1.0  # Scale factor for rotation mapping
        self.workspace_offset = np.array([0.0, 0.0, 0.0])  # Workspace center offset
        
        # State variables
        self.current_hand_coords = None
        self.current_hand_frame = None
        self.current_robot_pose = None
        self.last_hand_frame = None
        self.initialized = False
        
        # Calibration
        self.hand_origin_offset = np.array([0.0, 0.0, 0.0])
        self.robot_origin = None
        
        # Gripper control parameters
        self.gripper_distance_threshold = 0.05  # Distance threshold for pinch detection (meters)
        self.gripper_flag = False  # Current gripper state (False=open, True=closed)
        self.prev_gripper_flag = False  # Previous gripper state
        self.gripper_cnt = 0  # Counter for debouncing pinch detection
        self.gripper_debounce_frames = 3  # Number of frames to confirm pinch
        
        # Oculus joint indices (matching the VR data processor)
        self.OCULUS_JOINTS = {
            'metacarpals': [2, 6, 9, 12, 15],
            'knuckles': [6, 9, 12, 16],
            'thumb': [2, 3, 4, 5, 19],
            'index': [6, 7, 8, 20],
            'middle': [9, 10, 11, 21],
            'ring': [12, 13, 14, 22],
            'pinky': [15, 16, 17, 18, 23]
        }
        
        # Load configuration
        self._load_my_config()
        
        self.get_logger().info("Motion Retargeting initialized")
    
    def _load_my_config(self):
        """Load retargeting-specific configuration"""
        try:
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'motion_retargeting':
                        plugin_params = plugin.get('params', {})
                        self.position_scale = plugin_params.get("position_scale", self.position_scale)
                        self.rotation_scale = plugin_params.get("rotation_scale", self.rotation_scale)
                        self.workspace_offset = np.array(plugin_params.get("workspace_offset", [0.0, 0.0, 0.0]))
                        self.gripper_distance_threshold = plugin_params.get("gripper_distance_threshold", self.gripper_distance_threshold)
                        self.gripper_debounce_frames = plugin_params.get("gripper_debounce_frames", self.gripper_debounce_frames)
                        self.simulate_latency = plugin_params.get("simulate_latency", self.simulate_latency)
                        self.get_logger().info(
                            f"Loaded retargeting config: pos_scale={self.position_scale}, "
                            f"rot_scale={self.rotation_scale}, gripper_threshold={self.gripper_distance_threshold}m"
                        )
                        break
        except Exception as e:
            self.handle_error(f"Failed to load retargeting config: ", e)
    
    def register_pub_sub(self):
        """Register publishers and subscribers"""
        try:
            if not self.data_stream_config:
                self.handle_error("Data stream configuration not loaded")
                return
            
            topics = self.data_stream_config.get("topics", [])
            
            for topic_config in topics:
                topic_id = topic_config["id"]
                
                # Subscribe to processed hand keypoints
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
                    
                    elif msg_type == "component_interfaces/msg/RobotState":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            RobotState,
                            topic_id,
                            self.robot_state_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to robot state: {topic_id}")
                
                # Create publisher for robot control commands
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ControlCmd":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            ControlCmd,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for control commands: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: ", e)
    
    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""
        try:
            self.get_logger().debug("Received hand keypoint data")
            
            # Extract hand coordinates (24 joints x 3 coordinates = 72 values)
            if len(msg.hand_coords) != 72:
                self.get_logger().error(f"Invalid hand_coords length: {len(msg.hand_coords)}")
                return
            
            # Extract hand frame (4 vectors x 3 coordinates = 12 values)
            if len(msg.hand_frame) != 12:
                self.get_logger().error(f"Invalid hand_frame length: {len(msg.hand_frame)}")
                return
            
            # Reshape data
            self.current_hand_coords = np.array(msg.hand_coords).reshape(24, 3)
            hand_frame_flat = np.array(msg.hand_frame)
            
            # Parse hand frame: [origin(3), x_axis(3), y_axis(3), z_axis(3)]
            self.current_hand_frame = {
                'origin': hand_frame_flat[0:3],      # Wrist position in Oculus space
                'x_axis': hand_frame_flat[3:6],      # Index->Pinky direction
                'y_axis': hand_frame_flat[6:9],      # Palm normal
                'z_axis': hand_frame_flat[9:12]      # Forward palm direction
            }
            
            # Initialize calibration on first data
            if not self.initialized and self.current_robot_pose is not None:
                self._initialize_calibration()
            
            # Generate and publish control command
            if self.initialized:
                self._generate_robot_command()
                self._generate_gripper_command()
            
        except Exception as e:
            self.handle_error(f"Error in hand_keypoint_callback: ", e)
    
    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state updates"""
        try:
            if msg.robot_id == "xarm5":
                self.current_robot_pose = msg.end_effector_pose
                self.get_logger().debug("Updated robot pose")
        except Exception as e:
            self.handle_error(f"Error in robot_state_callback: ", e)
    
    def _initialize_calibration(self):
        """Initialize calibration by storing initial hand and robot poses"""
        try:
            if self.current_hand_frame is None or self.current_robot_pose is None:
                return
            
            # Store initial hand origin as reference
            self.hand_origin_offset = self.current_hand_frame['origin'].copy()
            
            # Store initial robot position as workspace origin
            self.robot_origin = np.array([
                self.current_robot_pose.position.x,
                self.current_robot_pose.position.y,
                self.current_robot_pose.position.z
            ])
            
            # Store initial hand frame for relative rotation calculation
            self.last_hand_frame = {
                'x_axis': self.current_hand_frame['x_axis'].copy(),
                'y_axis': self.current_hand_frame['y_axis'].copy(),
                'z_axis': self.current_hand_frame['z_axis'].copy()
            }
            
            self.initialized = True
            self.get_logger().info(f"Calibration initialized - Hand origin: {self.hand_origin_offset}, Robot origin: {self.robot_origin}")
            
        except Exception as e:
            self.handle_error(f"Error in calibration initialization: ", e)
    
    def _generate_robot_command(self):
        """Generate robot control command from current hand state"""
        try:
            if self.current_hand_frame is None or self.current_robot_pose is None:
                return
            
            # Calculate relative position change
            hand_position_delta = self.current_hand_frame['origin'] - self.hand_origin_offset
            
            # Map hand coordinate system to robot coordinate system
            # Oculus hand frame: X=index->pinky, Y=palm_normal, Z=palm_forward
            # Robot frame (typical): X=forward, Y=left, Z=up
            # We need to transform from hand frame to robot frame
            
            # Simple mapping (adjust based on your coordinate system):
            # Hand Z (forward) -> Robot X (forward)
            # Hand X (side) -> Robot Y (left) 
            # Hand Y (up/normal) -> Robot Z (up)
            robot_position_delta = np.array([
                hand_position_delta[2],   # Hand Z -> Robot X (forward)
                -hand_position_delta[0],  # Hand -X -> Robot Y (left, negative because hand X goes right)
                hand_position_delta[1]    # Hand Y -> Robot Z (up)
            ]) * self.position_scale
            
            # Calculate target position
            target_position = self.robot_origin + robot_position_delta + self.workspace_offset
            
            # Calculate rotation from hand frame change
            target_orientation = self._calculate_target_orientation()
            
            # Create control command
            control_cmd = ControlCmd()
            control_cmd.header = Header()
            control_cmd.header.stamp = self.get_clock().now().to_msg()
            control_cmd.header.frame_id = "link_base"
            
            control_cmd.robot_id = "xarm5"
            control_cmd.space = "cartesian"
            control_cmd.relative = False  # Absolute position command
            
            # Set target pose
            control_cmd.pose = Pose()
            control_cmd.pose.position = Point(
                x=float(target_position[0]),
                y=float(target_position[1]),
                z=float(target_position[2])
            )
            control_cmd.pose.orientation = target_orientation
            
            # Publish command
            self.publish_msg("robot_control_cmd", control_cmd)
            
            self.get_logger().debug(
                f"Published robot command - Pos: [{target_position[0]:.3f}, "
                f"{target_position[1]:.3f}, {target_position[2]:.3f}]"
            )
            
            # Update last hand frame for next iteration
            self.last_hand_frame = {
                'x_axis': self.current_hand_frame['x_axis'].copy(),
                'y_axis': self.current_hand_frame['y_axis'].copy(),
                'z_axis': self.current_hand_frame['z_axis'].copy()
            }
            
        except Exception as e:
            self.handle_error(f"Error generating robot command: ", e)
    
    def _calculate_target_orientation(self) -> Quaternion:
        """Calculate target robot orientation from hand frame"""
        try:
            if self.last_hand_frame is None:
                # Return current robot orientation if no previous frame
                return self.current_robot_pose.orientation
            
            # Build rotation matrices from hand frames
            # Current hand frame matrix (columns are axes)
            current_hand_matrix = np.column_stack([
                self.current_hand_frame['x_axis'],
                self.current_hand_frame['y_axis'],
                self.current_hand_frame['z_axis']
            ])
            
            # Previous hand frame matrix
            last_hand_matrix = np.column_stack([
                self.last_hand_frame['x_axis'],
                self.last_hand_frame['y_axis'],
                self.last_hand_frame['z_axis']
            ])
            
            # Calculate relative rotation in hand space
            # R_delta = R_current * R_last^T
            relative_rotation_matrix = current_hand_matrix @ last_hand_matrix.T
            relative_rotation = R.from_matrix(relative_rotation_matrix)
            
            # Transform to robot coordinate system
            # Map hand rotation axes to robot axes (same mapping as position)
            # This creates a coordinate transformation matrix
            hand_to_robot_transform = np.array([
                [0,  0,  1],   # Hand Z -> Robot X
                [-1, 0,  0],   # Hand -X -> Robot Y
                [0,  1,  0]    # Hand Y -> Robot Z
            ])
            
            # Apply coordinate transformation to rotation
            transformed_matrix = hand_to_robot_transform @ relative_rotation_matrix @ hand_to_robot_transform.T
            relative_robot_rotation = R.from_matrix(transformed_matrix)
            
            # Scale the rotation
            rotvec = relative_robot_rotation.as_rotvec()
            scaled_rotvec = rotvec * self.rotation_scale
            scaled_rotation = R.from_rotvec(scaled_rotvec)
            
            # Apply to current robot orientation
            current_robot_rot = R.from_quat([
                self.current_robot_pose.orientation.x,
                self.current_robot_pose.orientation.y,
                self.current_robot_pose.orientation.z,
                self.current_robot_pose.orientation.w
            ])
            
            # Combine: new = current * delta
            target_robot_rot = current_robot_rot * scaled_rotation
            target_quat = target_robot_rot.as_quat()
            
            # Create quaternion message
            orientation = Quaternion()
            orientation.x = float(target_quat[0])
            orientation.y = float(target_quat[1])
            orientation.z = float(target_quat[2])
            orientation.w = float(target_quat[3])
            
            return orientation
            
        except Exception as e:
            self.handle_error(f"Error calculating target orientation: ", e)
            # Return current orientation on error
            return self.current_robot_pose.orientation
    
    def _normalize_vector(self, vector: np.ndarray) -> np.ndarray:
        """Normalize a vector"""
        norm = np.linalg.norm(vector)
        if norm < 1e-6:
            return vector
        return vector / norm
    
    def _generate_gripper_command(self):
        """Generate gripper control command based on thumb-pinky finger distance"""
        try:
            if self.current_hand_coords is None:
                return
            
            # Get gripper state from hand keypoints
            gripper_state, status_changed, trigger_flag = self._get_gripper_state_from_hand_keypoints()
            
            # Only send command if state changed
            if status_changed:
                # Create gripper control command
                gripper_cmd = ControlCmd()
                gripper_cmd.header = Header()
                gripper_cmd.header.stamp = self.get_clock().now().to_msg()
                gripper_cmd.header.frame_id = "xarm_gripper"
                
                gripper_cmd.robot_id = "xarm_gripper"
                gripper_cmd.space = "joint"
                gripper_cmd.relative = False  # Absolute position
                
                # Map boolean state to gripper joint angle
                # gripper_state: False=open (0.0), True=closed (0.85)
                target_angle = 0.85 if gripper_state else 0.0
                gripper_cmd.joint_angles = [target_angle]
                
                # Publish gripper command
                self.publish_msg("gripper_control_cmd", gripper_cmd)
                
                state_str = "CLOSED" if gripper_state else "OPEN"
                self.get_logger().info(f"Gripper command sent: {state_str} (angle={target_angle:.2f})")
            
        except Exception as e:
            self.handle_error(f"Error generating gripper command: ", e)
    
    def _get_gripper_state_from_hand_keypoints(self) -> Tuple[bool, bool, bool]:
        """
        Detect gripper state from hand keypoints based on thumb-pinky distance.
        
        Returns:
            Tuple[bool, bool, bool]: 
                - gripper_state: Current gripper state (False=open, True=closed)
                - status_changed: Whether the state changed from previous
                - trigger_flag: Whether a new pinch was just detected
        """
        try:
            # Calculate distance between thumb tip and pinky tip
            thumb_tip_idx = self.OCULUS_JOINTS['thumb'][-1]  # Index 19
            pinky_tip_idx = self.OCULUS_JOINTS['pinky'][-1]  # Index 23
            
            thumb_pos = self.current_hand_coords[thumb_tip_idx]
            pinky_pos = self.current_hand_coords[pinky_tip_idx]
            
            distance = np.linalg.norm(thumb_pos - pinky_pos)
            
            trigger_flag = False
            
            # Check if fingers are pinched (distance below threshold)
            if distance < self.gripper_distance_threshold:
                self.gripper_cnt += 1
                
                # Debounce: only trigger after consistent frames
                if self.gripper_cnt == self.gripper_debounce_frames:
                    self.prev_gripper_flag = self.gripper_flag
                    self.gripper_flag = not self.gripper_flag  # Toggle state
                    trigger_flag = True
                    self.get_logger().info(f"Pinch detected! Distance: {distance:.4f}m, New state: {self.gripper_flag}")
            else:
                # Reset counter when fingers separate
                self.gripper_cnt = 0
            
            # Check if state changed
            status_changed = (self.gripper_flag != self.prev_gripper_flag)
            
            return self.gripper_flag, status_changed, trigger_flag
            
        except Exception as e:
            self.handle_error(f"Error getting gripper state: ", e)
            return False, False, False
    
    def reset_calibration(self):
        """Reset calibration - useful for recalibrating during runtime"""
        self.initialized = False
        self.hand_origin_offset = np.array([0.0, 0.0, 0.0])
        self.robot_origin = None
        self.last_hand_frame = None
        
        # Also reset gripper state
        self.gripper_flag = False
        self.prev_gripper_flag = False
        self.gripper_cnt = 0
        
        self.get_logger().info("Calibration and gripper state reset")
    
    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for Motion Retargeting")
        
        # Reset state
        self.current_hand_coords = None
        self.current_hand_frame = None
        self.initialized = False
        
        self.set_state("disconnected")
        self.get_logger().info("Motion Retargeting emergency stopped successfully")