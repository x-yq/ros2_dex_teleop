from core_bases.motion_retarget_base import MotionRetargetBase
from typing import Dict, List, Any, Optional
import rclpy
import yaml
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import tf2_ros
import tf2_geometry_msgs
import math
import time
from scipy.spatial.transform import Rotation, Slerp
from scipy.spatial.transform import Rotation as R
from numpy.linalg import pinv

# Import message types based on the interface definitions
from component_interfaces.msg import (
    ProcessedHandKeypoint,
    ControlCmd,
    RobotState
)
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Transform, TransformStamped, Quaternion, PoseStamped
from builtin_interfaces.msg import Time, Duration
from copy import copy, deepcopy

np.set_printoptions(precision=3, suppress=True)

# Oculus joint indices (adjust based on your system)
OCULUS_JOINTS = {
    'thumb': [0, 1, 2, 3, 4],
    'index': [5, 6, 7, 8, 9],
    'middle': [10, 11, 12, 13, 14],
    'ring': [15, 16, 17, 18, 19],
    'pinky': [20, 21, 22, 23, 24]
}

# Filter for removing noise in the teleoperation
class Filter:
    def __init__(self, state, comp_ratio=0.6):
        self.pos_state = state[:3]
        self.ori_state = state[3:6]
        self.comp_ratio = comp_ratio

    def __call__(self, next_state):
        self.pos_state = self.pos_state[:3] * self.comp_ratio + next_state[:3] * (1 - self.comp_ratio)
        ori_interp = Slerp([0, 1], Rotation.from_rotvec(
            np.stack([self.ori_state, next_state[3:6]], axis=0)),)
        self.ori_state = ori_interp([1 - self.comp_ratio])[0].as_rotvec()
        return np.concatenate([self.pos_state, self.ori_state])

class MotionRetargetOT(MotionRetargetBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        
        # Additional attributes for this specific retargeter
        self.processing_frequency = 90  # Hz
        self.retargeting_counter = 0
        
        self.latest_robot_state = None
        self.robot_init_H = None
        self.hand_init_H = None
        self.hand_init_t = None
        self.hand_moving_H = None
        self.robot_moving_H = None
        
        # Use filter
        self.use_filter = True
        self.comp_filter = None
        
        # Class variables for gripper and pause control
        self.gripper_flag = 1
        self.pause_flag = 1
        self.prev_pause_flag = 0
        self.is_first_frame = True
        self.gripper_cnt = 0
        self.prev_gripper_flag = 0
        self.pause_cnt = 0
        self.gripper_correct_state = 1
        self.resolution_scale = 1.0
        
        # Latest hand data
        self.latest_hand_coords = None
        self.latest_hand_frame = None

        # Initialize retargeting parameters
        self._load_my_config()

    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'motion_retarget_ot':
                        plugin_params = plugin.get('params', {})
                        self.processing_frequency = plugin_params.get("freq", self.processing_frequency)
                        self.use_filter = plugin_params.get("use_filter", self.use_filter)
                        self.get_logger().info(f"Loaded processor config: {self.processing_frequency}Hz")
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load configurations: {str(e)}")

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config instead of loading again
            if not self.data_stream_config or 'topics' not in self.data_stream_config:
                self.get_logger().warn("No data stream configuration available")
                return
            
            # Find topics where this node is a subscriber
            for topic_config in self.data_stream_config['topics']:
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
                    elif msg_type == "component_interfaces/msg/RobotState":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            RobotState,
                            topic_id,
                            self.robot_state_callback,
                            10
                        )
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ControlCmd":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            ControlCmd,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for control cmd: {topic_id}")
        
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: {str(e)}")
            
    def hand_keypoint_callback(self, msg: ProcessedHandKeypoint):
        """Callback for processed hand keypoint data"""
        self.get_logger().debug(f"Received processed hand keypoints")

        self.pong_reply("right_hand_keypoint_processed", msg)
        
        # Store latest hand data
        self.latest_hand_coords = np.array(msg.hand_coords).reshape(-1, 3)  # Reshape to (N, 3)
        self.latest_hand_frame = np.array(msg.hand_frame).reshape(4, 3)  # (4, 3) format
        
        # Retarget the hand data to robot commands
        self.retarget_hand_data(msg)

    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state updates"""
        pose = msg.end_effector_pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Rotation matrix from quaternion
        R_mat = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
        ], dtype=np.float64)

        # Homogeneous transform
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_mat
        T[:3, 3] = [x, y, z]

        self.latest_robot_state = copy(T)

    def get_robot_init(self):
        """Non-blocking method to get robot initial state"""
        if self.latest_robot_state is None:
            self.get_logger().warn("No robot state available yet")
            return False
        
        self.robot_init_H = copy(self.latest_robot_state)
        
        # Initialize filter if needed
        if self.use_filter and self.comp_filter is None:
            robot_init_cart = self._homo2cart(self.robot_init_H)
            self.comp_filter = Filter(robot_init_cart, comp_ratio=0.8)
            self.get_logger().info("Filter initialized")
        
        return True

    def retarget_hand_data(self, hand_data: ProcessedHandKeypoint):
        """Retarget hand keypoints to robot commands"""
        try:
            self.retargeting_counter += 1
            self.get_logger().debug(f"Retargeting hand data #{self.retargeting_counter}")
            
            # Check for pause/resume state
            pause_state, pause_status, pause_right = self.get_pause_state_from_hand_keypoints()
            
            # Check if we need to reset teleop (first frame or resuming from pause)
            if self.is_first_frame or (self.prev_pause_flag == 0 and pause_state == 1):
                if not self._reset_teleop():
                    return  # Return if reset fails
            
            self.prev_pause_flag = pause_state
            
            # If paused, don't send commands
            if pause_state == 0:
                self.get_logger().debug("Teleoperation paused")
                return
            
            # Generate control commands
            cmd_dict = self.retarget_control_to_robot_space(hand_data)
            if cmd_dict:
                self.publish_robot_commands(cmd_dict)
            
            self.get_logger().debug("Hand data retargeting completed successfully")
            
        except Exception as e:
            self.handle_error(f"Error retargeting hand data: {str(e)}")

    def _reset_teleop(self):
        """Reset teleoperation and make the current frame as initial frame"""
        self.get_logger().info('****** RESETTING TELEOP ******')
        
        # Get robot initial homogeneous matrix
        if not self.get_robot_init():
            return False
        
        # Get hand initial frame
        if self.latest_hand_frame is None:
            self.get_logger().warn("No hand frame available for reset")
            return False
        
        self.hand_init_H = self._turn_frame_to_homo_mat(self.latest_hand_frame)
        self.hand_init_t = copy(self.hand_init_H[:3, 3])
        self.is_first_frame = False
        
        self.get_logger().info("Resetting complete")
        return True

    def _turn_frame_to_homo_mat(self, frame):
        """Turn a frame to a homogeneous matrix"""
        t = frame[0]
        R_mat = frame[1:]

        homo_mat = np.zeros((4, 4))
        homo_mat[:3, :3] = np.transpose(R_mat)
        homo_mat[:3, 3] = t
        homo_mat[3, 3] = 1

        return homo_mat

    def _homo2cart(self, homo_mat):
        """Turn homogeneous matrix to cartesian vector (position + axis-angle)"""
        t = homo_mat[:3, 3]
        R_mat = Rotation.from_matrix(homo_mat[:3, :3]).as_rotvec(degrees=False)
        cart = np.concatenate([t, R_mat], axis=0)
        return cart

    def _get_scaled_cart_pose(self, moving_robot_homo_mat):
        """Get the scaled cartesian pose based on resolution scale"""
        # Get the cart pose without the scaling
        unscaled_cart_pose = self._homo2cart(moving_robot_homo_mat)

        # Get the current cart pose from latest robot state
        if self.latest_robot_state is None:
            self.get_logger().warn("No robot state available")
            return unscaled_cart_pose
        
        current_cart_pose = self._homo2cart(self.latest_robot_state)

        # Get the difference in translation between these two cart poses
        diff_in_translation = unscaled_cart_pose[:3] - current_cart_pose[:3]
        scaled_diff_in_translation = diff_in_translation * self.resolution_scale
        
        scaled_cart_pose = np.zeros(6)
        scaled_cart_pose[3:] = unscaled_cart_pose[3:]  # Get the rotation directly
        scaled_cart_pose[:3] = current_cart_pose[:3] + scaled_diff_in_translation  # Get the scaled translation only

        return scaled_cart_pose

    def get_gripper_state_from_hand_keypoints(self):
        """Get gripper state from hand keypoints (pinch detection)"""
        if self.latest_hand_coords is None:
            return self.gripper_flag, False, False
        
        try:
            # Calculate distance between pinky tip and thumb tip
            pinky_tip = self.latest_hand_coords[OCULUS_JOINTS['pinky'][-1]]
            thumb_tip = self.latest_hand_coords[OCULUS_JOINTS['thumb'][-1]]
            distance = np.linalg.norm(pinky_tip - thumb_tip)
            
            thresh = 0.03
            gripper_fl = False
            
            if distance < thresh:
                self.gripper_cnt += 1
                if self.gripper_cnt == 1:
                    self.prev_gripper_flag = self.gripper_flag
                    self.gripper_flag = not self.gripper_flag 
                    gripper_fl = True
            else: 
                self.gripper_cnt = 0
            
            gripper_state = np.asanyarray(self.gripper_flag).reshape(1)[0]
            status = False  
            if gripper_state != self.prev_gripper_flag:
                status = True
            
            return gripper_state, status, gripper_fl
        except Exception as e:
            self.get_logger().error(f"Error in gripper state detection: {str(e)}")
            return self.gripper_flag, False, False

    def get_pause_state_from_hand_keypoints(self):
        """Toggle pause/resume using ring/middle finger pinch"""
        if self.latest_hand_coords is None:
            return self.pause_flag, False, True
        
        try:
            # Calculate distances
            ring_tip = self.latest_hand_coords[OCULUS_JOINTS['ring'][-1]]
            middle_tip = self.latest_hand_coords[OCULUS_JOINTS['middle'][-1]]
            thumb_tip = self.latest_hand_coords[OCULUS_JOINTS['thumb'][-1]]
            
            ring_distance = np.linalg.norm(ring_tip - thumb_tip)
            middle_distance = np.linalg.norm(middle_tip - thumb_tip)
            
            thresh = 0.03 
            pause_right = True
            
            if ring_distance < thresh or middle_distance < thresh:
                self.pause_cnt += 1
                if self.pause_cnt == 1:
                    self.prev_pause_flag = self.pause_flag
                    self.pause_flag = not self.pause_flag       
            else:
                self.pause_cnt = 0
            
            pause_state = np.asanyarray(self.pause_flag).reshape(1)[0]
            pause_status = False  
            if pause_state != self.prev_pause_flag:
                pause_status = True 
            
            return pause_state, pause_status, pause_right
        except Exception as e:
            self.get_logger().error(f"Error in pause state detection: {str(e)}")
            return self.pause_flag, False, True

    def retarget_control_to_robot_space(self, hand_data=None):
        """
        Generate control commands using bimanual_right.py logic
        """
        try:
            # Check if initialization is complete
            if self.robot_init_H is None or self.hand_init_H is None:
                self.get_logger().warn("Not initialized yet")
                return None
            
            if self.latest_hand_frame is None:
                self.get_logger().warn("No hand frame available")
                return None
            
            # Get moving hand frame
            self.hand_moving_H = self._turn_frame_to_homo_mat(self.latest_hand_frame)
            
            # Transformation code (from bimanual_right.py)
            H_HI_HH = copy(self.hand_init_H)  # Homo matrix: P_HI to P_HH
            H_HT_HH = copy(self.hand_moving_H)  # Homo matrix: P_HT to P_HH
            H_RI_RH = copy(self.robot_init_H)  # Homo matrix: P_RI to P_RH
            
            H_HT_HI = np.linalg.pinv(H_HI_HH) @ H_HT_HH  # Homo matrix: P_HT to P_HI
            
            # Rotation transformation matrix
            H_R_V = np.array([[0, 0, -1, 0], 
                             [0, -1, 0, 0],
                             [-1, 0, 0, 0],
                             [0, 0, 0, 1]])
            
            # Translation transformation matrix
            H_T_V = np.array([[0, 0, 1, 0],
                             [0, 1, 0, 0],
                             [-1, 0, 0, 0],
                             [0, 0, 0, 1]])
            
            H_HT_HI_r = (pinv(H_R_V) @ H_HT_HI @ H_R_V)[:3, :3]
            H_HT_HI_t = (pinv(H_T_V) @ H_HT_HI @ H_T_V)[:3, 3]
            
            relative_affine = np.block(
                [[H_HT_HI_r, H_HT_HI_t.reshape(3, 1)], [0, 0, 0, 1]])
            
            target_translation = H_RI_RH[:3, 3] + relative_affine[:3, 3]
            target_rotation = H_RI_RH[:3, :3] @ relative_affine[:3, :3]
            
            H_RT_RH = np.block(
                [[target_rotation, target_translation.reshape(-1, 1)], [0, 0, 0, 1]])
            
            self.robot_moving_H = copy(H_RT_RH)
            final_pose = self._get_scaled_cart_pose(self.robot_moving_H)
            
            # Apply the filter
            if self.use_filter and self.comp_filter is not None:
                final_pose = self.comp_filter(final_pose)
            
            # Get gripper state
            gripper_state, status_change, gripper_flag = self.get_gripper_state_from_hand_keypoints()
            if gripper_flag == 1 and status_change is True:
                self.gripper_correct_state = gripper_state
            
            # Create header
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id="xarm5")
            
            # --- Robot cartesian command ---
            robot_cmd = ControlCmd()
            robot_cmd.header = header
            robot_cmd.robot_id = "xarm5"
            robot_cmd.relative = False  # Using absolute pose based on retargeting logic
            robot_cmd.space = "cartesian"
            robot_cmd.joint_names = []
            robot_cmd.joint_angles = []
            robot_cmd.joint_velocities = []
            
            # Set pose from final_pose (position in meters, orientation as axis-angle)
            robot_cmd.pose = Pose()
            robot_cmd.pose.position.x = float(final_pose[0])
            robot_cmd.pose.position.y = float(final_pose[1])
            robot_cmd.pose.position.z = float(final_pose[2])
            
            # Convert axis-angle to quaternion
            rot = R.from_rotvec(final_pose[3:6])
            quat = rot.as_quat()  # Returns [x, y, z, w]
            robot_cmd.pose.orientation.x = float(quat[0])
            robot_cmd.pose.orientation.y = float(quat[1])
            robot_cmd.pose.orientation.z = float(quat[2])
            robot_cmd.pose.orientation.w = float(quat[3])
            
            robot_cmd.timeout = Duration(sec=5)
            
            # --- Gripper command ---
            grip_cmd = ControlCmd()
            grip_cmd.header = header
            grip_cmd.robot_id = "xarm_gripper"
            grip_cmd.relative = False
            grip_cmd.space = "joint"
            grip_cmd.joint_names = ["drive_joint"]
            # Scale gripper state (0 or 1) to joint angle (0.0 to 0.8 or 1.0)
            grip_cmd.joint_angles = [float(self.gripper_correct_state * 0.8)]
            grip_cmd.joint_velocities = [0.3]
            grip_cmd.pose = Pose()
            grip_cmd.timeout = Duration(sec=2)
            
            return {
                "robot_cmd": robot_cmd,
                "gripper_cmd": grip_cmd,
            }
            
        except Exception as e:
            self.handle_error(f"Error in retarget_control_to_robot_space: {str(e)}")
            return None

    def publish_robot_commands(self, cmd_dict):
        """Publish given ControlCmd messages to their respective topics"""
        try:
            if "robot_cmd" in cmd_dict:
                self.publish_msg("robot_control_cmd", cmd_dict["robot_cmd"])
                self.get_logger().debug("Published robot control command")

            if "gripper_cmd" in cmd_dict:
                self.publish_msg("gripper_control_cmd", cmd_dict["gripper_cmd"])
                self.get_logger().debug("Published gripper control command")

        except Exception as e:
            self.handle_error(f"Error publishing control commands: {str(e)}")

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for MotionRetargetOT")
        
        # # Send stop command to robot (hold current position)
        # stop_msg = ControlCmd()
        # stop_msg.header = Header()
        # stop_msg.header.stamp = self.get_clock().now().to_msg()
        # stop_msg.robot_id = "xarm5"
        # stop_msg.joint_names = self.robot_joint_names
        # stop_msg.joint_angles = [0.0] * len(self.robot_joint_names)
        # stop_msg.timeout = Duration(sec=0, nanosec=0)
        
        self.set_state("disconnected")
        
        self.get_logger().info("MotionRetargetOT emergency stopped successfully")