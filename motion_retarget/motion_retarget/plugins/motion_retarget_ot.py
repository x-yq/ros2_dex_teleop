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
        self.system_paused = False  # NEW: System pause state
        
        # Calibration
        self.hand_origin_offset = np.array([0.0, 0.0, 0.0])
        self.robot_origin = None
        
        # Gripper control parameters
        self.gripper_distance_threshold = 0.05  # Distance threshold for pinch detection (meters)
        self.gripper_flag = False  # Current gripper state (False=open, True=closed)
        self.prev_gripper_flag = False  # Previous gripper state
        self.gripper_cnt = 0  # Counter for debouncing pinch detection
        self.gripper_debounce_frames = 3  # Number of frames to confirm pinch
        
        # Wrist rotation control for robot base joint
        self.wrist_rotation_threshold = 1.57  # Rotation threshold in radians (~90 degrees)
        self.wrist_rotation_scale = 0.5  # Scale factor for mapping wrist rotation to joint rotation
        self.accumulated_wrist_rotation = 0.0  # Accumulated wrist rotation angle
        self.base_joint_angle = 0.0  # Current base joint angle
        self.prev_hand_y_axis = None  # Previous hand y-axis for rotation calculation
        
        # Calibration gesture detection parameters
        self.calibration_distance_threshold = 0.04  # Distance threshold for thumb-middle touch (meters)
        self.calibration_gesture_cnt = 0  # Counter for debouncing calibration gesture
        self.calibration_debounce_frames = 5  # Number of frames to confirm calibration gesture
        self.prev_calibration_gesture = False  # Previous calibration gesture state
        
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
                    if plugin.get('id') == 'motion_retarget_ot':
                        plugin_params = plugin.get('params', {})
                        self.position_scale = plugin_params.get("position_scale", self.position_scale)
                        self.rotation_scale = plugin_params.get("rotation_scale", self.rotation_scale)
                        self.workspace_offset = np.array(plugin_params.get("workspace_offset", [0.0, 0.0, 0.0]))
                        self.gripper_distance_threshold = plugin_params.get("gripper_distance_threshold", self.gripper_distance_threshold)
                        self.gripper_debounce_frames = plugin_params.get("gripper_debounce_frames", self.gripper_debounce_frames)
                        self.calibration_distance_threshold = plugin_params.get("calibration_distance_threshold", self.calibration_distance_threshold)
                        self.calibration_debounce_frames = plugin_params.get("calibration_debounce_frames", self.calibration_debounce_frames)
                        self.wrist_rotation_threshold = plugin_params.get("wrist_rotation_threshold", self.wrist_rotation_threshold)
                        self.wrist_rotation_scale = plugin_params.get("wrist_rotation_scale", self.wrist_rotation_scale)
                        self.simulate_latency = plugin_params.get("simulate_latency", self.simulate_latency)
                        self.get_logger().info(
                            f"Loaded retargeting config: pos_scale={self.position_scale}, "
                            f"rot_scale={self.rotation_scale}, gripper_threshold={self.gripper_distance_threshold}m, "
                            f"calibration_threshold={self.calibration_distance_threshold}m"
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
            self.pong_reply("right_hand_keypoint_processed", msg)
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
            
            # NEW: Check for calibration gesture (thumb-middle finger touch)
            self._check_calibration_gesture()
            
            # Initialize calibration on first data (only if not paused)
            if not self.initialized and not self.system_paused and self.current_robot_pose is not None:
                self._initialize_calibration()
            
            # Generate and publish control command (only if initialized and not paused)
            if self.initialized and not self.system_paused:
                self._generate_robot_command()
                self._generate_gripper_command()
                # self._generate_base_joint_command()  # NEW: Handle wrist rotation
            
        except Exception as e:
            self.handle_error(f"Error in hand_keypoint_callback: ", e)
    
    def robot_state_callback(self, msg: RobotState):
        """Callback for robot state updates"""
        try:
            if msg.robot_id == "xarm5":
                self.pong_reply('robot_state',msg)
                self.current_robot_pose = msg.end_effector_pose
                self.get_logger().debug("Updated robot pose")
            else:
                self.pong_reply('gripper_state',msg)
        except Exception as e:
            self.handle_error(f"Error in robot_state_callback: ", e)
    
    def _check_calibration_gesture(self):
        """
        Detect the thumb-to-middle-finger calibration gesture with debouncing.
 
        Computes the Euclidean distance between the thumb tip (joint 19) and the
        middle finger tip (joint 21) in the canonical hand coordinate frame.
        When the distance stays below ``self.calibration_distance_threshold`` for
        ``self.calibration_debounce_frames`` consecutive callbacks, the gesture is
        considered confirmed and :meth:`_handle_calibration_gesture` is called
        **once** (edge-triggered, not level-triggered).
 
        The counter and ``prev_calibration_gesture`` flag are reset as soon as the
        fingers separate, enabling the gesture to be triggered again on the next
        touch.
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        try:
            if self.current_hand_coords is None:
                return
            
            # Get thumb tip and middle finger tip positions
            thumb_tip_idx = self.OCULUS_JOINTS['thumb'][-1]  # Index 19
            middle_tip_idx = self.OCULUS_JOINTS['middle'][-1]  # Index 21
            
            thumb_pos = self.current_hand_coords[thumb_tip_idx]
            middle_pos = self.current_hand_coords[middle_tip_idx]
            
            distance = np.linalg.norm(thumb_pos - middle_pos)
            
            # Check if fingers are touching (distance below threshold)
            if distance < self.calibration_distance_threshold:
                self.calibration_gesture_cnt += 1
                
                # Debounce: only trigger after consistent frames
                if self.calibration_gesture_cnt >= self.calibration_debounce_frames:
                    # Check if this is a new gesture (wasn't touching before)
                    if not self.prev_calibration_gesture:
                        self.prev_calibration_gesture = True
                        self._handle_calibration_gesture()
                    # Keep counter at threshold to prevent repeated triggers
                    self.calibration_gesture_cnt = self.calibration_debounce_frames
            else:
                # Reset counter and state when fingers separate
                if self.calibration_gesture_cnt > 0:
                    self.calibration_gesture_cnt = 0
                    self.prev_calibration_gesture = False
                    
        except Exception as e:
            self.handle_error(f"Error checking calibration gesture: ", e)
    
    def _handle_calibration_gesture(self):
        """
        Respond to a confirmed calibration gesture with a two-state toggle.
 
        **First confirmed touch** — pauses the system and resets all calibration
        state (hand origin, robot origin, last hand frame, gripper state). While
        paused, no robot or gripper commands are published.
 
        **Second confirmed touch** — resumes the system and immediately calls
        :meth:`_initialize_calibration` so the current hand and robot pose become
        the new reference frame. If either datum is missing the resume is aborted
        and an error is logged.
 
        This two-step flow lets the operator reposition themselves freely between
        the two touches without sending unintended motion commands.
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        try:
            if not self.system_paused:
                # First touch: PAUSE and RESET
                self.system_paused = True
                self.initialized = False
                self.hand_origin_offset = np.array([0.0, 0.0, 0.0])
                self.robot_origin = None
                self.last_hand_frame = None
                
                # Reset gripper state
                self.gripper_flag = False
                self.prev_gripper_flag = False
                self.gripper_cnt = 0
                
                self.get_logger().warn("=" * 60)
                self.get_logger().warn("CALIBRATION GESTURE DETECTED: SYSTEM PAUSED")
                self.get_logger().warn("Calibration reset. Touch thumb-middle again to resume.")
                self.get_logger().warn("=" * 60)
                
            else:
                # Second touch: RESUME and RECALIBRATE
                self.system_paused = False
                
                # Initialize calibration with current state as new origin
                if self.current_hand_frame is not None and self.current_robot_pose is not None:
                    self._initialize_calibration()
                    
                    self.get_logger().warn("=" * 60)
                    self.get_logger().warn("CALIBRATION GESTURE DETECTED: SYSTEM RESUMED")
                    self.get_logger().warn("New calibration initialized with current state as origin.")
                    self.get_logger().warn("=" * 60)
                else:
                    self.get_logger().error("Cannot resume: Missing hand or robot state data")
                    
        except Exception as e:
            self.handle_error(f"Error handling calibration gesture: ", e)
    
    def _initialize_calibration(self):
        """
        Snapshot the current hand and robot poses as the retargeting reference frame.
 
        Records three anchors that all subsequent motion commands are expressed
        relative to:
 
        - ``hand_origin_offset`` — wrist world position at calibration time; used
          to compute position deltas.
        - ``robot_origin`` — robot end-effector XYZ at calibration time; deltas
          are added back to this to produce absolute target positions.
        - ``last_hand_frame`` — palm axis triad ``{x, y, z}`` at calibration time;
          used as the *previous* frame in the first incremental rotation
          computation after calibration.
 
        Also initialises wrist-rotation tracking (``prev_hand_y_axis``,
        ``accumulated_wrist_rotation``) and sets ``self.initialized = True``,
        allowing :meth:`hand_keypoint_callback` to start emitting commands.
 
        Returns immediately (no-op) if either ``current_hand_frame`` or
        ``current_robot_pose`` is not yet available.
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """        
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
            
            # NEW: Initialize wrist rotation tracking
            self.prev_hand_y_axis = self.current_hand_frame['y_axis'].copy()
            self.accumulated_wrist_rotation = 0.0
            
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
        """
        Compute the absolute target end-effector orientation from the incremental
        palm frame rotation.
 
        Builds 3×3 rotation matrices from the current and previous palm frame
        axis triads ``{x, y, z}`` and computes the relative rotation between
        them::
 
            R_delta = R_current · R_last^T
 
        The delta is then expressed in the robot base frame via the
        hand→robot axis permutation::
 
            hand_to_robot = [[0, 0, 1],   # Hand Z → Robot X
                             [−1, 0, 0],  # Hand −X → Robot Y
                             [0, 1, 0]]   # Hand Y → Robot Z
 
            R_robot_delta = hand_to_robot · R_delta · hand_to_robot^T
 
        The rotation vector of ``R_robot_delta`` is scaled by
        ``self.rotation_scale`` and then composed onto the current robot
        orientation quaternion::
 
            q_target = q_current ⊗ q(R_robot_delta_scaled)
 
        Falls back to the current robot orientation quaternion if
        ``last_hand_frame`` is not yet set or an exception occurs.
 
        Returns:
            geometry_msgs.msg.Quaternion: Absolute target orientation for the
                                          robot end-effector.
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """        
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
        Detect gripper open/close state from thumb-to-pinky tip distance.
 
        Computes the Euclidean distance between the thumb tip (joint 19) and the
        pinky tip (joint 23) in the canonical hand coordinate frame. When the
        distance falls below ``self.gripper_distance_threshold`` for exactly
        ``self.gripper_debounce_frames`` consecutive callbacks the gripper state
        is toggled (open → closed or closed → open). The debounce counter
        continues to increment after the toggle so the gesture must fully release
        (fingers separate) before a second toggle can occur.
 
        Returns:
            Tuple[bool, bool, bool]:
                - **gripper_state** — current gripper state after this call
                  (``False`` = open, ``True`` = closed).
                - **status_changed** — ``True`` if a toggle just occurred; the
                  caller should publish a gripper command only when this is
                  ``True``.
                - **trigger_flag** — ``True`` on the exact frame the toggle fires
                  (always equals ``status_changed`` in the current implementation).
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        try:
            # Calculate distance between thumb tip and pinky tip
            thumb_tip_idx = self.OCULUS_JOINTS['thumb'][-1]  # Index 19
            pinky_tip_idx = self.OCULUS_JOINTS['pinky'][-1]  # Index 23
            
            thumb_pos = self.current_hand_coords[thumb_tip_idx]
            pinky_pos = self.current_hand_coords[pinky_tip_idx]
            
            distance = np.linalg.norm(thumb_pos - pinky_pos)
            
            trigger_flag = False
            status_changed = False
            
            # Check if fingers are pinched (distance below threshold)
            if distance < self.gripper_distance_threshold:
                self.gripper_cnt += 1
                
                # Debounce: only trigger after consistent frames
                if self.gripper_cnt == self.gripper_debounce_frames:
                    self.prev_gripper_flag = self.gripper_flag
                    self.gripper_flag = not self.gripper_flag  # Toggle state
                    trigger_flag = True
                    status_changed = True
                    self.get_logger().info(f"Pinch detected! Distance: {distance:.4f}m, New state: {self.gripper_flag}")
            else:
                # Reset counter when fingers separate
                self.gripper_cnt = 0
            
            # Check if state changed
            # status_changed = (self.gripper_flag != self.prev_gripper_flag)
            
            return self.gripper_flag, status_changed, trigger_flag
            
        except Exception as e:
            self.handle_error(f"Error getting gripper state: ", e)
            return False, False, False
    
    def _generate_base_joint_command(self):
        """
        Generate base joint control command based on wrist rotation.
        Detects clockwise/counter-clockwise wrist rotation and rotates robot base joint accordingly.
        """
        try:
            if self.current_hand_frame is None or self.prev_hand_y_axis is None:
                return
            
            # Calculate wrist rotation angle
            wrist_rotation_angle = self._calculate_wrist_rotation()
            
            if wrist_rotation_angle is None:
                return
            
            # Accumulate rotation
            self.accumulated_wrist_rotation += wrist_rotation_angle
            
            # Check if accumulated rotation exceeds threshold
            if abs(self.accumulated_wrist_rotation) >= self.wrist_rotation_threshold:
                # Calculate joint angle change (scaled)
                joint_angle_delta = self.accumulated_wrist_rotation * self.wrist_rotation_scale
                
                # Update base joint angle (for tracking purposes)
                self.base_joint_angle += joint_angle_delta
                
                # Create base joint control command
                base_joint_cmd = ControlCmd()
                base_joint_cmd.header = Header()
                base_joint_cmd.header.stamp = self.get_clock().now().to_msg()
                base_joint_cmd.header.frame_id = "link_base"
                
                base_joint_cmd.robot_id = "xarm5"
                base_joint_cmd.space = "joint"
                base_joint_cmd.relative = True  # Use relative mode
                
                # Set joint names and angles - only controlling joint1 (base rotation)
                # Other joints remain at 0.0 (no change)
                base_joint_cmd.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
                base_joint_cmd.joint_angles = [
                    float(joint_angle_delta),  # joint1: base rotation
                    0.0,  # joint2: no change
                    0.0,  # joint3: no change
                    0.0,  # joint4: no change
                    0.0   # joint5: no change
                ]
                
                # Publish command using the robot_control_cmd topic
                self.publish_msg("robot_control_cmd", base_joint_cmd)
                
                rotation_dir = "CLOCKWISE" if joint_angle_delta > 0 else "COUNTER-CLOCKWISE"
                self.get_logger().info(
                    f"Wrist rotation detected: {rotation_dir}, "
                    f"Base joint relative command: {joint_angle_delta:.3f} rad "
                    f"(accumulated total: {self.base_joint_angle:.3f} rad)"
                )
                
                # Reset accumulated rotation after command
                self.accumulated_wrist_rotation = 0.0
            
            # Update previous y-axis for next iteration
            self.prev_hand_y_axis = self.current_hand_frame['y_axis'].copy()
            
        except Exception as e:
            self.handle_error(f"Error generating base joint command: ", e)
    
    def _calculate_wrist_rotation(self) -> Optional[float]:
        """
        Compute the signed wrist roll angle between the previous and current callback.
 
        Wrist roll is defined as rotation of the palm around the hand's forward
        axis (``z_axis``, pointing in the direction the fingers point). The palm
        normal (``y_axis``) is tracked frame-to-frame; its angular displacement
        projected onto the plane perpendicular to ``z_axis`` gives the roll angle.
 
        Algorithm:
 
        1. Project both the current and previous ``y_axis`` vectors onto the
           plane perpendicular to the current ``z_axis`` (removes out-of-plane
           components caused by combined wrist motions).
        2. Normalise both projected vectors.
        3. Compute the unsigned angle between them via ``arccos(dot)`` (clamped
           to ``[−1, 1]`` for numerical safety).
        4. Determine the sign from the cross-product of the two projected
           vectors, dotted with ``z_axis``: positive = clockwise when looking
           down ``z_axis``, negative = counter-clockwise.
 
        Returns:
            float | None: Signed rotation angle in radians for this frame step,
                          or ``None`` if ``prev_hand_y_axis`` is not initialised
                          or a numerical exception occurs.
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        try:
            if self.prev_hand_y_axis is None:
                return None
            
            current_y = self.current_hand_frame['y_axis']
            prev_y = self.prev_hand_y_axis
            
            # Get the hand's forward direction (z-axis) as rotation axis
            rotation_axis = self.current_hand_frame['z_axis']
            
            # Project both y-axes onto the plane perpendicular to z-axis
            # This isolates the rotation around the z-axis (wrist rotation)
            current_y_proj = current_y - np.dot(current_y, rotation_axis) * rotation_axis
            prev_y_proj = prev_y - np.dot(prev_y, rotation_axis) * rotation_axis
            
            # Normalize projected vectors
            current_y_proj_norm = self._normalize_vector(current_y_proj)
            prev_y_proj_norm = self._normalize_vector(prev_y_proj)
            
            # Calculate angle between projected vectors
            dot_product = np.clip(np.dot(current_y_proj_norm, prev_y_proj_norm), -1.0, 1.0)
            angle = np.arccos(dot_product)
            
            # Determine rotation direction using cross product
            cross_product = np.cross(prev_y_proj_norm, current_y_proj_norm)
            direction = np.dot(cross_product, rotation_axis)
            
            # Apply sign based on direction
            # Positive = clockwise when looking down the z-axis
            signed_angle = angle if direction > 0 else -angle
            
            return signed_angle
            
        except Exception as e:
            self.handle_error(f"Error calculating wrist rotation: ", e)
            return None
    
    def reset_calibration(self):
        """
        Programmatically reset all calibration and gesture state.
 
        Clears the hand origin, robot origin, and last hand frame so the node
        will re-initialise on the next callback. Also resets the gripper toggle,
        the calibration gesture debounce, and the wrist rotation accumulator.
        Unpauses the system if it was paused.
 
        Intended for use by the broader component lifecycle framework or external
        service calls when a full re-calibration is required without using the
        in-hand gesture (e.g. at startup, after a robot fault, or from a GUI).
 
        .. note::
            Adapted from the Open-Teach teleoperation system:
 
            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """        
        self.initialized = False
        self.system_paused = False
        self.hand_origin_offset = np.array([0.0, 0.0, 0.0])
        self.robot_origin = None
        self.last_hand_frame = None
        
        # Also reset gripper state
        self.gripper_flag = False
        self.prev_gripper_flag = False
        self.gripper_cnt = 0
        
        # Reset calibration gesture state
        self.calibration_gesture_cnt = 0
        self.prev_calibration_gesture = False
        
        # Reset wrist rotation state
        self.accumulated_wrist_rotation = 0.0
        self.base_joint_angle = 0.0
        self.prev_hand_y_axis = None
        
        self.get_logger().info("Calibration and gripper state reset")
    
    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for Motion Retargeting")
        
        # Reset state
        self.current_hand_coords = None
        self.current_hand_frame = None
        self.initialized = False
        self.system_paused = False
        
        self.set_state("disconnected")
        self.get_logger().info("Motion Retargeting emergency stopped successfully")