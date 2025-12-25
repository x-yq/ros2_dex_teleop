# import rclpy
# from rclpy.timer import Timer
# from rclpy.action import ActionClient
# import json
# import yaml
# import numpy as np
# from typing import Dict, List, Any, Optional, Tuple
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
# from std_msgs.msg import Header
# from builtin_interfaces.msg import Time, Duration
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from control_msgs.action import FollowJointTrajectory
# from controller_manager_msgs.srv import SwitchController
# from component_interfaces.msg import *
# from core_bases.robot_driver_base import RobotDriverBase
# from tf2_ros import TransformListener, Buffer
# import time
# import math
# from scipy.spatial.transform import Rotation as R


# class SimXArm5Driver(RobotDriverBase):
#     """
#     ROS2 Driver node for simulated XArm5 robot and gripper.
#     Handles both joint and cartesian control commands.
#     """

#     def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
#         super().__init__(layer_cn_id, simulated=True, ros2_domain_id=ros2_domain_id)
        
#         # Driver specific attributes from config
#         self.freq = 120
#         self.scale_factor = 1.0
#         self.default_controllers = []
        
#         # Robot state
#         self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
#         self.gripper_joint_names = ["drive_joint"]
        
#         # Current states
#         self.current_robot_joint_state = None
#         self.current_gripper_joint_state = None
#         # self.current_eef_pose = None
#         self.current_gripper_pose = None
        
#         # Robot active flags
#         self.robot_active = False
#         self.robot_connected = False

#         # Controller management
#         self.active_controller = "xarm5_cartesian_motion_controller"  # Default to cartesian
#         self.cartesian_controller_name = "xarm5_cartesian_motion_controller"
#         self.joint_controller_name = "xarm5_traj_controller"
        
#         # Initial poses
#         self.robot_initial_pose = [0.207, 0.0, 0.112, 1.0, 0.0, 0.0, 0.0]  # [x, y, z, qx, qy, qz, qw]
#         self.gripper_initial_pose = {
#             'right_finger': [0.207, 0.07, 0.01, 1.0, 0.0, 0.0, 0.0],
#             'left_finger': [0.207, -0.07, 0.01, 1.0, 0.0, 0.0, 0.0]
#         }
        
#         # TF2 for getting poses
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
        
#         # # ROS2 Control publishers and action clients
#         # self.cartesian_target_pub = None
#         # self.robot_traj_action_client = None
#         # self.gripper_action_client = None
#         # self.controller_switch_client = None
        
#         # Joint state subscribers
#         self.joint_state_sub = None
        
#         # Load configuration
#         self._load_my_config()
        
#         # Initialize robot
#         self._initialize_robot()

#     def _load_my_config(self):
#         """Load all required configurations using ComponentNode methods"""
#         try:
#             # Load component-specific configuration
#             self.my_config = self.load_my_config()
#             if self.my_config and "plugins" in self.my_config:
#                 plugins = self.my_config["plugins"]
#                 for plugin in plugins:
#                     if plugin.get('id') == 'sim_xarm5_driver':
#                         plugin_params = plugin.get('params', {})
#                         self.freq = plugin_params.get("freq", self.freq)
#                         self.scale_factor = plugin_params.get("scale_factor", self.scale_factor)
#                         self.default_controllers = plugin_params.get("default_controller", [])
#                         if 'joint_names' in plugin_params:
#                             self.joint_names = plugin_params['joint_names']
#                         self.get_logger().info(
#                             f"Loaded driver config: freq={self.freq}Hz, "
#                             f"scale_factor={self.scale_factor}, "
#                             f"controllers={self.default_controllers}"
#                         )
#                         break
                
#         except Exception as e:
#             self.handle_error(f"Failed to load configurations: {str(e)}")

#     def register_pub_sub(self):
#         """
#         Register publishers and subscribers based on data stream configuration.
#         Publishers: robot_state, gripper_state
#         Subscribers: robot_control_cmd, gripper_control_cmd
#         """
#         try:
#             if not self.data_stream_config or 'topics' not in self.data_stream_config:
#                 self.handle_error("No data stream configuration available")
#                 return
            
#             topics = self.data_stream_config.get('topics', [])
            
#             # Register based on data_stream.yaml
#             for topic in topics:
#                 topic_id = topic.get('id')
#                 msg_type = topic.get('msg_type')
#                 publisher = topic.get('publisher')
#                 subscribers = topic.get('subscribers', [])
                
#                 # Create publishers for robot_state and gripper_state
#                 if publisher == self.layer_cn_id:
#                     if topic_id in ['robot_state', 'gripper_state']:
#                         if msg_type == 'component_interfaces/msg/RobotState':
#                             self.compo_publishers[topic_id] = self.create_publisher(
#                                 RobotState, topic_id, 10
#                             )
#                             self.logging(f"Created publisher for {topic_id}", "INFO")
                
#                 # Create subscribers for robot_control_cmd and gripper_control_cmd
#                 if self.layer_cn_id in subscribers:
#                     if topic_id in ['robot_control_cmd', 'gripper_control_cmd']:
#                         if msg_type == 'component_interfaces/msg/ControlCmd':
#                             self.compo_subscribers[topic_id] = self.create_subscription(
#                                 ControlCmd,
#                                 topic_id,
#                                 lambda msg, tid=topic_id: self.cmd_sub_callback(msg, tid),
#                                 10
#                             )
#                             self.logging(f"Created subscriber for {topic_id}", "INFO")
            
#             # Subscribe to joint_states from Gazebo/ros2_control
#             self.joint_state_sub = self.create_subscription(
#                 JointState,
#                 '/joint_states',
#                 self.joint_state_callback,
#                 10
#             )
#             self.logging("Subscribed to /joint_states", "INFO")
            
#             # Create cartesian motion controller publisher
#             self.cartesian_target_pub = self.create_publisher(
#                 PoseStamped,
#                 '/xarm5_cartesian_motion_controller/target_frame',
#                 10
#             )
#             self.logging("Created cartesian target publisher", "INFO")
            
#             # Create robot joint trajectory action client
#             self.robot_traj_action_client = ActionClient(
#                 self,
#                 FollowJointTrajectory,
#                 '/xarm5_traj_controller/follow_joint_trajectory'
#             )
#             self.logging("Created robot trajectory action client", "INFO")
            
#             # Create gripper action client
#             self.gripper_action_client = ActionClient(
#                 self,
#                 FollowJointTrajectory,
#                 '/xarm_gripper_traj_controller/follow_joint_trajectory'
#             )
#             self.logging("Created gripper action client", "INFO")

#             # Create controller switch service client
#             self.controller_switch_client = self.create_client(
#                 SwitchController,
#                 '/controller_manager/switch_controller'
#             )
#             self.logging("Created controller switch service client", "INFO")
            
#             self.logging(
#                 f"Registered {len(self.compo_publishers)} publishers and "
#                 f"{len(self.compo_subscribers)} subscribers", 
#                 "INFO"
#             )
            
#         except Exception as e:
#             self.handle_error(f"Error in register_pub_sub: {str(e)}")

#     def joint_state_callback(self, msg: JointState):
#         """Callback for joint_states topic from Gazebo/ros2_control"""
#         try:
#             # Separate robot and gripper joint states
#             robot_indices = []
#             gripper_indices = []
            
#             for i, name in enumerate(msg.name):
#                 if name in self.joint_names:
#                     robot_indices.append(i)
#                 elif name in self.gripper_joint_names:
#                     gripper_indices.append(i)
            
#             # Update robot joint state
#             if robot_indices:
#                 self.current_robot_joint_state = JointState()
#                 self.current_robot_joint_state.header = msg.header
#                 self.current_robot_joint_state.name = [msg.name[i] for i in robot_indices]
#                 self.current_robot_joint_state.position = [msg.position[i] for i in robot_indices]
#                 if msg.velocity:
#                     self.current_robot_joint_state.velocity = [msg.velocity[i] for i in robot_indices]
#                 if msg.effort:
#                     self.current_robot_joint_state.effort = [msg.effort[i] for i in robot_indices]
            
#             # Update gripper joint state
#             if gripper_indices:
#                 self.current_gripper_joint_state = JointState()
#                 self.current_gripper_joint_state.header = msg.header
#                 self.current_gripper_joint_state.name = [msg.name[i] for i in gripper_indices]
#                 self.current_gripper_joint_state.position = [msg.position[i] for i in gripper_indices]
#                 if msg.velocity:
#                     self.current_gripper_joint_state.velocity = [msg.velocity[i] for i in gripper_indices]
#                 if msg.effort:
#                     self.current_gripper_joint_state.effort = [msg.effort[i] for i in gripper_indices]
                    
#         except Exception as e:
#             self.logging(f"Error in joint_state_callback: {str(e)}", "ERROR")

#     def cmd_sub_callback(self, msg: ControlCmd, topic_id: str):
#         """
#         Callback for control commands (robot_control_cmd and gripper_control_cmd).
#         Handles both joint and cartesian space control.
#         """
#         try:
#             self.logging(f"Received command on {topic_id}", "DEBUG")
            
#             # Validate robot ID
#             if msg.robot_id not in ["xarm5", "xarm_gripper"]:
#                 self.logging(f"Command for unknown robot: {msg.robot_id}, ignoring", "WARN")
#                 return
            
#             # Determine if this is for robot or gripper
#             is_gripper = (topic_id == 'gripper_control_cmd' or msg.robot_id == "xarm_gripper")
            
#             # Execute command based on space
#             if msg.space.lower() == 'cartesian':
#                 if is_gripper:
#                     self.logging("Gripper does not support cartesian control", "WARN")
#                     return
#                 self.move_cartesian(msg)
#             elif msg.space.lower() == 'joint':
#                 if is_gripper:
#                     self.move_joint(msg, group='gripper')
#                 else:
#                     self.move_joint(msg, group='robot')
#             else:
#                 self.logging(f"Unknown control space: {msg.space}", "WARN")
                
#         except Exception as e:
#             self.handle_error(f"Error in cmd_sub_callback: {str(e)}")

#     def switch_controller(self, target_controller: str) -> bool:
#         """
#         Switch between cartesian and joint trajectory controllers.
        
#         Args:
#             target_controller: Name of the controller to activate
            
#         Returns:
#             bool: True if switch successful or already active, False otherwise
#         """
#         try:
#             # Check if already using the target controller - no need to switch
#             if self.active_controller == target_controller:
#                 self.logging(f"Controller {target_controller} already active, skipping switch", "DEBUG")
#                 return True
            
#             # Wait for service
#             if not self.controller_switch_client.wait_for_service(timeout_sec=2.0):
#                 self.logging("Controller switch service not available", "WARN")
#                 return False
            
#             # Determine which controller to start and stop
#             if target_controller == self.joint_controller_name:
#                 start_controllers = [self.joint_controller_name]
#                 stop_controllers = [self.cartesian_controller_name]
#             elif target_controller == self.cartesian_controller_name:
#                 start_controllers = [self.cartesian_controller_name]
#                 stop_controllers = [self.joint_controller_name]
#             else:
#                 self.logging(f"Unknown controller: {target_controller}", "ERROR")
#                 return False
            
#             # Create service request
#             request = SwitchController.Request()
#             request.activate_controllers = start_controllers
#             request.deactivate_controllers = stop_controllers
#             request.strictness = SwitchController.Request.STRICT
#             request.start_asap = True
#             request.timeout = Duration(sec=0, nanosec=0)
            
#             # Call service
#             self.logging(f"Switching controller: {self.active_controller} -> {target_controller}", "INFO")
#             future = self.controller_switch_client.call_async(request)
#             rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
#             # if future.result() is not None and future.result().ok:
#             self.active_controller = target_controller
#             self.logging(f"Successfully switched to controller: {target_controller}", "INFO")
#             return True
#             # else:
#             #     self.logging(f"Failed to switch to controller: {target_controller}", "ERROR")
#             #     return False
                
#         except Exception as e:
#             self.logging(f"Error switching controller: {str(e)}", "ERROR")
#             return False

#     def move_cartesian(self, cmd: ControlCmd):
#         """
#         Execute cartesian space control command for robot arm.
#         Only valid for xarm5 robot, not gripper.
#         """
#         try:
#             if not self.robot_active:
#                 self.logging("Robot not active, ignoring command", "WARN")
#                 return

#             # Switch to cartesian controller if needed
#             if not self.switch_controller(self.cartesian_controller_name):
#                 self.logging("Failed to switch to cartesian controller", "ERROR")
#                 return
            
#             # Get current pose
#             current_pose = self.get_eef_pose_from_tf('link_base', 'link5')
#             if current_pose is None:
#                 self.logging("Cannot get current EEF pose", "ERROR")
#                 return
            
#             # Calculate target pose
#             if cmd.relative:
#                 # Apply scale factor to relative motion
#                 target_pose = self._apply_relative_cartesian(
#                     current_pose, 
#                     cmd.pose, 
#                     self.scale_factor
#                 )
#             else:
#                 target_pose = cmd.pose
            
#             # Ensure orientation is in quaternion
#             # target_pose = self._ensure_quaternion(target_pose)
            
#             # Publish to cartesian motion controller
#             pose_stamped = PoseStamped()
#             pose_stamped.header.stamp = self.get_clock().now().to_msg()
#             pose_stamped.header.frame_id = 'link_base'
#             pose_stamped.pose = target_pose
            
#             self.cartesian_target_pub.publish(pose_stamped)
#             self.logging(
#                 f"Published cartesian target: pos=[{target_pose.position.x:.3f}, "
#                 f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f}]",
#                 "DEBUG"
#             )
            
#         except Exception as e:
#             self.handle_error(f"Error in move_cartesian: {str(e)}")

#     def move_joint(self, cmd: ControlCmd, group: str = 'robot'):
#         """
#         Execute joint space control command.
#         Valid for both robot arm and gripper.
        
#         Args:
#             cmd: ControlCmd message
#             group: 'robot' or 'gripper'
#         """
#         try:
#             if not self.robot_active:
#                 self.logging("Robot not active, ignoring command", "WARN")
#                 return
            
#             if group == 'robot':
#                 # Switch to joint trajectory controller if needed
#                 if not self.switch_controller(self.joint_controller_name):
#                     self.logging("Failed to switch to joint trajectory controller", "ERROR")
#                     return
                
#                 # Validate joint count
#                 if len(cmd.joint_angles) != len(self.joint_names):
#                     self.handle_error(
#                         f"Invalid joint count for robot: expected {len(self.joint_names)}, "
#                         f"got {len(cmd.joint_angles)}"
#                     )
#                     return
                
#                 # Get current joint state
#                 if self.current_robot_joint_state is None:
#                     self.logging("No current robot joint state available", "WARN")
#                     return
                
#                 current_positions = list(self.current_robot_joint_state.position)
                
#                 # Calculate target positions
#                 if cmd.relative:
#                     # Apply relative motion
#                     target_positions = [
#                         current + delta
#                         for current, delta in zip(current_positions, cmd.joint_angles)
#                     ]
#                 else:
#                     target_positions = list(cmd.joint_angles)
                
#                 # Send robot joint trajectory action
#                 self._send_robot_trajectory(target_positions)
#                 self.logging(f"Sent robot joint command: {target_positions}", "DEBUG")
                
#             elif group == 'gripper':
#                 # Validate gripper joint count
#                 if len(cmd.joint_angles) != len(self.gripper_joint_names):
#                     self.handle_error(
#                         f"Invalid joint count for gripper: expected {len(self.gripper_joint_names)}, "
#                         f"got {len(cmd.joint_angles)}"
#                     )
#                     return
                
#                 # Get current gripper state
#                 if self.current_gripper_joint_state is None:
#                     self.logging("No current gripper joint state available", "WARN")
#                     current_position = 0.0
#                 else:
#                     current_position = self.current_gripper_joint_state.position[0]
                
#                 # Calculate target position
#                 if cmd.relative:
#                     target_position = current_position + cmd.joint_angles[0] * self.scale_factor
#                 else:
#                     target_position = cmd.joint_angles[0]
                
#                 # Clamp to valid range [0.0, 1.0]
#                 target_position = np.clip(target_position, 0.0, 1.0)
                
#                 # Send gripper trajectory action
#                 self._send_gripper_trajectory(target_position)
#                 self.logging(f"Sent gripper command: {target_position:.3f}", "DEBUG")
                
#         except Exception as e:
#             self.handle_error(f"Error in move_joint: {str(e)}")


#     def _send_robot_trajectory(self, target_positions: List[float]):
#         """
#         Send robot joint trajectory action goal.
        
#         Args:
#             target_positions: Target joint positions for all 5 joints
#         """
#         try:
#             if not self.robot_traj_action_client.wait_for_server(timeout_sec=1.0):
#                 self.logging("Robot trajectory action server not available", "WARN")
#                 return
            
#             # Get current positions
#             if self.current_robot_joint_state is None:
#                 self.logging("No current robot joint state available", "WARN")
#                 return
            
#             current_positions = list(self.current_robot_joint_state.position)
            
#             # Create trajectory goal
#             goal = FollowJointTrajectory.Goal()
#             goal.trajectory.joint_names = self.joint_names
#             goal.trajectory.header.stamp = self.get_clock().now().to_msg()
#             goal.trajectory.header.frame_id = "link_base"
            
#             # Create trajectory points
#             # Point 1: Start from current position
#             point1 = JointTrajectoryPoint()
#             point1.positions = current_positions
#             point1.velocities = [0.0] * len(self.joint_names)
#             point1.time_from_start = Duration(sec=0, nanosec=0)
            
#             # Point 2: Move to target position
#             point2 = JointTrajectoryPoint()
#             point2.positions = target_positions
#             point2.velocities = [0.0] * len(self.joint_names)
            
#             # Calculate trajectory duration based on maximum joint displacement
#             max_displacement = max(abs(t - c) for t, c in zip(target_positions, current_positions))
#             # Assume max joint velocity of 1.0 rad/s, minimum 0.2 seconds
#             time_sec = max(max_displacement / 1.0, 0.2)
#             point2.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            
#             # Add points to trajectory
#             goal.trajectory.points = [point1, point2]
            
#             # Set goal tolerances (optional, but recommended)
#             goal.goal_time_tolerance = Duration(sec=1, nanosec=500000000)  # 1.5 seconds
            
#             # Send goal asynchronously
#             send_goal_future = self.robot_traj_action_client.send_goal_async(goal)
#             self.logging(f"Sent robot trajectory to positions: {target_positions}", "DEBUG")
            
#         except Exception as e:
#             self.logging(f"Error sending robot trajectory: {str(e)}", "ERROR")

#     def _send_gripper_trajectory(self, target_pos: float):
#         """
#         Send gripper trajectory action goal.
        
#         Args:
#             target_pos: Target gripper position (0.0 = open, 1.0 = closed)
#         """
#         try:
#             if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
#                 self.logging("Gripper action server not available", "WARN")
#                 return
            
#             # Get current position
#             if self.current_gripper_joint_state is None:
#                 self.logging("No current gripper joint state available", "WARN")
#                 current_pos = 0.0
#             else:
#                 current_pos = self.current_gripper_joint_state.position[0]
            
#             # Create trajectory
#             goal = FollowJointTrajectory.Goal()
#             goal.trajectory.joint_names = self.gripper_joint_names
#             goal.trajectory.header.stamp = self.get_clock().now().to_msg()
#             goal.trajectory.header.frame_id = "xarm_gripper_base_link"
            
#             # Start point (current position)
#             point1 = JointTrajectoryPoint()
#             point1.positions = [current_pos]
#             point1.velocities = [0.0]
#             point1.time_from_start = Duration(sec=0, nanosec=0)
            
#             # End point (target position)
#             point2 = JointTrajectoryPoint()
#             point2.positions = [target_pos]
#             point2.velocities = [0.0]
            
#             # Calculate time based on distance (1 rad/s max velocity)
#             distance = abs(target_pos - current_pos)
#             time_sec = max(distance / 1.0, 0.1)  # At least 0.1 second
#             point2.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            
#             goal.trajectory.points = [point1, point2]
            
#             # Send goal asynchronously
#             self.gripper_action_client.send_goal_async(goal)
            
#         except Exception as e:
#             self.logging(f"Error sending gripper trajectory: {str(e)}", "ERROR")

#     def get_state(self, robot_id: str) -> Optional[Dict[str, Any]]:
#         """
#         Get current robot or gripper state.
        
#         Args:
#             robot_id: 'xarm5' or 'xarm_gripper'
            
#         Returns:
#             Dictionary containing robot state data
#         """
#         if not self.robot_active:
#             return None
        
#         try:
#             if robot_id == "xarm5":
#                 if self.current_robot_joint_state is None:
#                     return None
                
#                 # Get EEF pose from TF
#                 eef_pose = self.get_eef_pose_from_tf('link_base', 'link5')
#                 if eef_pose is None:
#                     eef_pose = Pose()  # Use default if TF not available
                
#                 robot_state = {
#                     'robot_id': robot_id,
#                     'joint_state': self.current_robot_joint_state,
#                     'end_effector_pose': eef_pose,
#                     'timestamp': self.get_clock().now().to_msg()
#                 }
#                 return robot_state
                
#             elif robot_id == "xarm_gripper":
#                 if self.current_gripper_joint_state is None:
#                     return None
                
#                 # Calculate gripper center pose (average of two fingers)
#                 gripper_pose = self.get_gripper_center_pose()
#                 if gripper_pose is None:
#                     gripper_pose = Pose()
                
#                 gripper_state = {
#                     'robot_id': robot_id,
#                     'joint_state': self.current_gripper_joint_state,
#                     'end_effector_pose': gripper_pose,
#                     'timestamp': self.get_clock().now().to_msg()
#                 }
#                 return gripper_state
            
#             return None
            
#         except Exception as e:
#             self.handle_error(f"Error getting state for {robot_id}: {str(e)}")
#             return None

#     def get_eef_pose_from_tf(self, base_frame: str, eef_frame: str) -> Optional[Pose]:
#         """Get end effector pose from TF tree"""
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 base_frame,
#                 eef_frame,
#                 rclpy.time.Time(),
#                 timeout=rclpy.duration.Duration(seconds=0.1)
#             )
            
#             pose = Pose()
#             pose.position.x = transform.transform.translation.x
#             pose.position.y = transform.transform.translation.y
#             pose.position.z = transform.transform.translation.z
#             pose.orientation = transform.transform.rotation
            
#             return pose
            
#         except Exception as e:
#             self.logging(f"Could not get transform {base_frame} -> {eef_frame}: {str(e)}", "DEBUG")
#             return None

#     def get_gripper_center_pose(self) -> Optional[Pose]:
#         """Calculate gripper center pose as average of right and left fingers"""
#         try:
#             right_pose = self.get_eef_pose_from_tf('xarm_gripper_base_link', 'right_finger')
#             left_pose = self.get_eef_pose_from_tf('xarm_gripper_base_link', 'left_finger')
            
#             if right_pose and left_pose:
#                 center_pose = Pose()
#                 center_pose.position.x = (right_pose.position.x + left_pose.position.x) / 2.0
#                 center_pose.position.y = (right_pose.position.y + left_pose.position.y) / 2.0
#                 center_pose.position.z = (right_pose.position.z + left_pose.position.z) / 2.0
#                 # Use right finger orientation as representative
#                 center_pose.orientation = right_pose.orientation
#                 return center_pose
            
#             return None
            
#         except Exception as e:
#             self.logging(f"Error calculating gripper center pose: {str(e)}", "DEBUG")
#             return None

#     def _create_robot_state_msg(self, robot_state: Dict[str, Any]) -> RobotState:
#         """Create RobotState message from robot state data"""
#         msg = RobotState()
        
#         # Set header
#         msg.header = Header()
#         msg.header.stamp = robot_state['timestamp']
#         msg.header.frame_id = "xarm5"
        
#         # Set robot ID
#         msg.robot_id = robot_state['robot_id']
        
#         # Set joint state
#         msg.joint_states = robot_state['joint_state']
        
#         # Set end effector pose (with both quaternion and euler)
#         msg.end_effector_pose = robot_state['end_effector_pose']
        
#         return msg

#     def publish_state(self):
#         """Publish current robot and gripper state - called by timer"""
#         if not self.robot_active:
#             return
        
#         try:
#             # Publish robot state
#             robot_state = self.get_state("xarm5")
#             if robot_state and 'robot_state' in self.compo_publishers:
#                 msg = self._create_robot_state_msg(robot_state)
#                 self.publish_msg('robot_state', msg)
#                 self.logging("Published robot state", "DEBUG")
            
#             # Publish gripper state
#             gripper_state = self.get_state("xarm_gripper")
#             if gripper_state and 'gripper_state' in self.compo_publishers:
#                 msg = self._create_robot_state_msg(gripper_state)
#                 self.publish_msg('gripper_state', msg)
#                 self.logging("Published gripper state", "DEBUG")
                
#         except Exception as e:
#             self.handle_error(f"Error publishing states: {str(e)}")

#     def home(self):
#         """Move robot to initial pose"""
#         try:
#             self.logging("Moving robot to home position", "INFO")
            
#             # Create home pose
#             home_pose = Pose()
#             home_pose.position.x = self.robot_initial_pose[0]
#             home_pose.position.y = self.robot_initial_pose[1]
#             home_pose.position.z = self.robot_initial_pose[2]
#             home_pose.orientation.x = self.robot_initial_pose[3]
#             home_pose.orientation.y = self.robot_initial_pose[4]
#             home_pose.orientation.z = self.robot_initial_pose[5]
#             home_pose.orientation.w = self.robot_initial_pose[6]
            
#             # Publish home pose
#             pose_stamped = PoseStamped()
#             pose_stamped.header.stamp = self.get_clock().now().to_msg()
#             pose_stamped.header.frame_id = 'link_base'
#             pose_stamped.pose = home_pose
            
#             self.cartesian_target_pub.publish(pose_stamped)
            
#             # Home gripper (fully open)
#             self._send_gripper_trajectory(
#                 0.0
#             )
            
#             self.logging("Robot homing initiated", "INFO")
            
#         except Exception as e:
#             self.handle_error(f"Error in home: {str(e)}")

#     def execute_command(self, robot_id: str, command: Any) -> None:
#         """
#         Execute robot command.
#         This is a wrapper for compatibility with base class.
#         Actual execution is done through move_cartesian and move_joint.
#         """
#         try:
#             if not self.robot_active:
#                 self.logging("Robot not active, ignoring command", "WARN")
#                 return
            
#             self.logging(f"Execute command called for {robot_id}: {command}", "DEBUG")
            
#         except Exception as e:
#             self.handle_error(f"Error executing command: {str(e)}")

#     def register_timer(self):
#         """Register timer for state publishing"""
#         if self.compo_timers:
#             for t in self.compo_timers:
#                 t.cancel()
#             self.compo_timers.clear()
        
#         # Calculate timer period from frequency
#         timer_period = 1.0 / self.freq
#         timer = self.create_timer(timer_period, self.publish_state)
#         self.compo_timers.append(timer)
#         self.logging(f"Registered timer for state publishing at {self.freq}Hz", "INFO")

#     def _initialize_robot(self):
#         """Initialize robot connection and state"""
#         try:
#             self.logging("Initializing XArm5 robot driver", "INFO")
            
#             if self.simulated:
#                 self.logging("Running in simulated mode", "INFO")
#                 self.robot_connected = True
            
#             # Set robot active
#             self.robot_active = True
            
#             self.logging("XArm5 robot driver initialized successfully", "INFO")
            
#         except Exception as e:
#             self.handle_error(f"Error initializing robot: {str(e)}")
    
#     def start_component(self):
#         if self.compo_timers == [] or not self.compo_timers:
#             self.register_timer() #defined in the bottom node
#         else:
#             for timer in self.compo_timers:
#                 # if timer.is_canceled():
#                 timer.reset()
#         self.robot_active = True

#     def urgent_stop(self):
#         """Emergency stop - stop robot motion immediately"""
#         self.logging("Emergency stop triggered", "WARN")
        
#         # Stop robot motion by publishing current pose
#         if self.robot_active and self.current_robot_joint_state:
#             try:
#                 current_pose = self.get_eef_pose_from_tf('link_base', 'link5')
#                 self.home()

#             except Exception as e:
#                 self.logging(f"Error during urgent stop: {str(e)}", "ERROR")
        
#         self.robot_active = False
#         # self.stop_component()
        
#         self.logging("Emergency stop completed", "WARN")

#     # ============ Utility Methods ============
    
#     def _apply_relative_cartesian(self, current: Pose, delta: Pose, scale: float) -> Pose:
#         """Apply relative cartesian motion with scale factor"""
#         target = Pose()
        
#         # Apply position delta
#         target.position.x = current.position.x + delta.position.x * scale
#         target.position.y = current.position.y + delta.position.y * scale
#         target.position.z = current.position.z + delta.position.z * scale
        
#         # Apply orientation delta (quaternion multiplication)
#         # Scale the rotation delta
#         current_rot = R.from_quat([
#             current.orientation.x,
#             current.orientation.y,
#             current.orientation.z,
#             current.orientation.w
#         ])
#         delta_rot = R.from_quat([
#             delta.orientation.x,
#             delta.orientation.y,
#             delta.orientation.z,
#             delta.orientation.w
#         ])
        
#         # Scale rotation by converting to rotvec, scaling, and back
#         delta_rotvec = delta_rot.as_rotvec()
#         scaled_delta_rot = R.from_rotvec(delta_rotvec * scale)
        
#         # Combine rotations
#         target_rot = current_rot * scaled_delta_rot
#         target_quat = target_rot.as_quat()
        
#         target.orientation.x = target_quat[0]
#         target.orientation.y = target_quat[1]
#         target.orientation.z = target_quat[2]
#         target.orientation.w = target_quat[3]
        
#         return target

#     def _ensure_quaternion(self, pose: Pose) -> Pose:
#         """Ensure orientation is in quaternion format"""
#         # If orientation looks like euler (values > 2*pi), convert
#         # This is a simplification - in practice you'd have a flag
#         quat_sum = (pose.orientation.x**2 + pose.orientation.y**2 + 
#                     pose.orientation.z**2 + pose.orientation.w**2)
        
#         if abs(quat_sum - 1.0) > 0.1:  # Not normalized quaternion
#             # Assume it's euler angles in x, y, z (roll, pitch, yaw)
#             rot = R.from_euler('xyz', [
#                 pose.orientation.x,
#                 pose.orientation.y,
#                 pose.orientation.z
#             ])
#             quat = rot.as_quat()
#             pose.orientation.x = quat[0]
#             pose.orientation.y = quat[1]
#             pose.orientation.z = quat[2]
#             pose.orientation.w = quat[3]
        
#         return pose

import rclpy
from rclpy.timer import Timer
from rclpy.action import ActionClient
import json
import yaml
import numpy as np
from typing import Dict, List, Any, Optional, Tuple
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from component_interfaces.msg import *
from core_bases.robot_driver_base import RobotDriverBase
from tf2_ros import TransformListener, Buffer
import time
import math
from scipy.spatial.transform import Rotation as R


class SimXArm5Driver(RobotDriverBase):
    """
    ROS2 Driver node for simulated XArm5 robot and gripper.
    Handles both joint and cartesian control commands.
    """

    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, simulated=True, ros2_domain_id=ros2_domain_id)
        
        # Driver specific attributes from config
        self.freq = 120
        self.scale_factor = 1.0
        self.default_controllers = []
        
        # Robot state
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self.gripper_joint_names = ["drive_joint"]
        
        # Current states
        self.current_robot_joint_state = None
        self.current_gripper_joint_state = None
        # self.current_eef_pose = None
        self.current_gripper_pose = None
        
        # Robot active flags
        self.robot_active = False
        self.robot_connected = False
        
        # Initial poses
        self.robot_initial_pose = [0.207, 0.0, 0.112, 1.0, 0.0, 0.0, 0.0]  # [x, y, z, qx, qy, qz, qw]
        self.gripper_initial_pose = {
            'right_finger': [0.207, 0.07, 0.01, 1.0, 0.0, 0.0, 0.0],
            'left_finger': [0.207, -0.07, 0.01, 1.0, 0.0, 0.0, 0.0]
        }
        
        # TF2 for getting poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # # ROS2 Control publishers and action clients
        # self.cartesian_target_pub = None
        # self.robot_traj_action_client = None
        # self.gripper_action_client = None
        
        # Joint state subscribers
        self.joint_state_sub = None
        
        # Load configuration
        self._load_my_config()
        
        # Initialize robot
        self._initialize_robot()

    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'sim_xarm5_driver':
                        plugin_params = plugin.get('params', {})
                        self.freq = plugin_params.get("freq", self.freq)
                        self.scale_factor = plugin_params.get("scale_factor", self.scale_factor)
                        self.default_controllers = plugin_params.get("default_controller", [])
                        if 'joint_names' in plugin_params:
                            self.joint_names = plugin_params['joint_names']
                        self.get_logger().info(
                            f"Loaded driver config: freq={self.freq}Hz, "
                            f"scale_factor={self.scale_factor}, "
                            f"controllers={self.default_controllers}"
                        )
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load configurations: {str(e)}")

    def register_pub_sub(self):
        """
        Register publishers and subscribers based on data stream configuration.
        Publishers: robot_state, gripper_state
        Subscribers: robot_control_cmd, gripper_control_cmd
        """
        try:
            if not self.data_stream_config or 'topics' not in self.data_stream_config:
                self.handle_error("No data stream configuration available")
                return
            
            topics = self.data_stream_config.get('topics', [])
            
            # Register based on data_stream.yaml
            for topic in topics:
                topic_id = topic.get('id')
                msg_type = topic.get('msg_type')
                publisher = topic.get('publisher')
                subscribers = topic.get('subscribers', [])
                
                # Create publishers for robot_state and gripper_state
                if publisher == self.layer_cn_id:
                    if topic_id in ['robot_state', 'gripper_state']:
                        if msg_type == 'component_interfaces/msg/RobotState':
                            self.compo_publishers[topic_id] = self.create_publisher(
                                RobotState, topic_id, 10
                            )
                            self.logging(f"Created publisher for {topic_id}", "INFO")
                
                # Create subscribers for robot_control_cmd and gripper_control_cmd
                if self.layer_cn_id in subscribers:
                    if topic_id in ['robot_control_cmd', 'gripper_control_cmd']:
                        if msg_type == 'component_interfaces/msg/ControlCmd':
                            self.compo_subscribers[topic_id] = self.create_subscription(
                                ControlCmd,
                                topic_id,
                                lambda msg, tid=topic_id: self.cmd_sub_callback(msg, tid),
                                10
                            )
                            self.logging(f"Created subscriber for {topic_id}", "INFO")
            
            # Subscribe to joint_states from Gazebo/ros2_control
            self.joint_state_sub = self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )
            self.logging("Subscribed to /joint_states", "INFO")
            
            # Create cartesian motion controller publisher
            self.cartesian_target_pub = self.create_publisher(
                PoseStamped,
                '/xarm5_cartesian_motion_controller/target_frame',
                10
            )
            self.logging("Created cartesian target publisher", "INFO")
            
            # Create robot joint trajectory action client
            self.robot_traj_action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/xarm5_traj_controller/follow_joint_trajectory'
            )
            self.logging("Created robot trajectory action client", "INFO")
            
            # Create gripper action client
            self.gripper_action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/xarm_gripper_traj_controller/follow_joint_trajectory'
            )
            self.logging("Created gripper action client", "INFO")
            
            self.logging(
                f"Registered {len(self.compo_publishers)} publishers and "
                f"{len(self.compo_subscribers)} subscribers", 
                "INFO"
            )
            
        except Exception as e:
            self.handle_error(f"Error in register_pub_sub: {str(e)}")

    def joint_state_callback(self, msg: JointState):
        """Callback for joint_states topic from Gazebo/ros2_control"""
        try:
            # Separate robot and gripper joint states
            robot_indices = []
            gripper_indices = []
            
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    robot_indices.append(i)
                elif name in self.gripper_joint_names:
                    gripper_indices.append(i)
            
            # Update robot joint state
            if robot_indices:
                self.current_robot_joint_state = JointState()
                self.current_robot_joint_state.header = msg.header
                self.current_robot_joint_state.name = [msg.name[i] for i in robot_indices]
                self.current_robot_joint_state.position = [msg.position[i] for i in robot_indices]
                if msg.velocity:
                    self.current_robot_joint_state.velocity = [msg.velocity[i] for i in robot_indices]
                if msg.effort:
                    self.current_robot_joint_state.effort = [msg.effort[i] for i in robot_indices]
            
            # Update gripper joint state
            if gripper_indices:
                self.current_gripper_joint_state = JointState()
                self.current_gripper_joint_state.header = msg.header
                self.current_gripper_joint_state.name = [msg.name[i] for i in gripper_indices]
                self.current_gripper_joint_state.position = [msg.position[i] for i in gripper_indices]
                if msg.velocity:
                    self.current_gripper_joint_state.velocity = [msg.velocity[i] for i in gripper_indices]
                if msg.effort:
                    self.current_gripper_joint_state.effort = [msg.effort[i] for i in gripper_indices]
                    
        except Exception as e:
            self.logging(f"Error in joint_state_callback: {str(e)}", "ERROR")

    def cmd_sub_callback(self, msg: ControlCmd, topic_id: str):
        """
        Callback for control commands (robot_control_cmd and gripper_control_cmd).
        Handles both joint and cartesian space control.
        """
        try:
            self.pong_reply(topic_id, msg)

            self.logging(f"Received command on {topic_id}", "DEBUG")
            
            # Validate robot ID
            if msg.robot_id not in ["xarm5", "xarm_gripper"]:
                self.logging(f"Command for unknown robot: {msg.robot_id}, ignoring", "WARN")
                return
            
            # Determine if this is for robot or gripper
            is_gripper = (topic_id == 'gripper_control_cmd' or msg.robot_id == "xarm_gripper")
            
            # Execute command based on space
            if msg.space.lower() == 'cartesian':
                if is_gripper:
                    self.logging("Gripper does not support cartesian control", "WARN")
                    return
                self.move_cartesian(msg)
            elif msg.space.lower() == 'joint':
                if is_gripper:
                    self.move_joint(msg, group='gripper')
                else:
                    self.move_joint(msg, group='robot')
            else:
                self.logging(f"Unknown control space: {msg.space}", "WARN")
                
        except Exception as e:
            self.handle_error(f"Error in cmd_sub_callback: {str(e)}")

    def move_cartesian(self, cmd: ControlCmd):
        """
        Execute cartesian space control command for robot arm.
        Only valid for xarm5 robot, not gripper.
        """
        try:
            if not self.robot_active:
                self.logging("Robot not active, ignoring command", "WARN")
                return
            
            # Get current pose
            current_pose = self.get_eef_pose_from_tf('link_base', 'link5')
            if current_pose is None:
                self.logging("Cannot get current EEF pose", "ERROR")
                return
            
            # Calculate target pose
            if cmd.relative:
                # Apply scale factor to relative motion
                target_pose = self._apply_relative_cartesian(
                    current_pose, 
                    cmd.pose, 
                    self.scale_factor
                )
            else:
                target_pose = cmd.pose
            
            # Ensure orientation is in quaternion
            # target_pose = self._ensure_quaternion(target_pose)
            
            # Publish to cartesian motion controller
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'link_base'
            pose_stamped.pose = target_pose
            
            self.cartesian_target_pub.publish(pose_stamped)
            self.logging(
                f"Published cartesian target: pos=[{target_pose.position.x:.3f}, "
                f"{target_pose.position.y:.3f}, {target_pose.position.z:.3f}]",
                "DEBUG"
            )
            
        except Exception as e:
            self.handle_error(f"Error in move_cartesian: {str(e)}")

    def move_joint(self, cmd: ControlCmd, group: str = 'robot'):
        """
        Execute joint space control command.
        Valid for both robot arm and gripper.
        
        Args:
            cmd: ControlCmd message
            group: 'robot' or 'gripper'
        """
        try:
            if not self.robot_active:
                self.logging("Robot not active, ignoring command", "WARN")
                return
            
            if group == 'robot':
                # Validate joint count
                if len(cmd.joint_angles) != len(self.joint_names):
                    self.handle_error(
                        f"Invalid joint count for robot: expected {len(self.joint_names)}, "
                        f"got {len(cmd.joint_angles)}"
                    )
                    return
                
                # Get current joint state
                if self.current_robot_joint_state is None:
                    self.logging("No current robot joint state available", "WARN")
                    return
                
                current_positions = list(self.current_robot_joint_state.position)
                
                # Calculate target positions
                if cmd.relative:
                    # Apply scale factor to relative motion
                    target_positions = [
                        current + delta
                        for current, delta in zip(current_positions, cmd.joint_angles)
                    ]
                else:
                    target_positions = list(cmd.joint_angles)
                
                # Send robot joint trajectory action
                self._send_robot_trajectory(current_positions, target_positions)
                self.logging(f"Sent robot joint command", "DEBUG")
                
            elif group == 'gripper':
                # Validate gripper joint count
                if len(cmd.joint_angles) != len(self.gripper_joint_names):
                    self.handle_error(
                        f"Invalid joint count for gripper: expected {len(self.gripper_joint_names)}, "
                        f"got {len(cmd.joint_angles)}"
                    )
                    return
                
                # Get current gripper state
                if self.current_gripper_joint_state is None:
                    self.logging("No current gripper joint state available", "WARN")
                    current_position = 0.0
                else:
                    current_position = self.current_gripper_joint_state.position[0]
                
                # Calculate target position
                if cmd.relative:
                    target_position = current_position + cmd.joint_angles[0] * self.scale_factor
                else:
                    target_position = cmd.joint_angles[0]
                
                # Clamp to valid range [0.0, 1.0]
                target_position = np.clip(target_position, 0.0, 1.0)
                
                # Send gripper trajectory action
                self._send_gripper_trajectory(current_position, target_position)
                self.logging(f"Sent gripper command: {target_position:.3f}", "DEBUG")
                
        except Exception as e:
            self.handle_error(f"Error in move_joint: {str(e)}")

    def _send_robot_trajectory(self, current_positions: List[float], target_positions: List[float]):
        """Send robot joint trajectory action goal"""
        try:
            if not self.robot_traj_action_client.wait_for_server(timeout_sec=1.0):
                self.logging("Robot trajectory action server not available", "WARN")
                return
            
            # Create trajectory goal
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.joint_names
            
            # Start point (current position)
            point1 = JointTrajectoryPoint()
            point1.positions = current_positions
            point1.velocities = [0.0] * len(self.joint_names)
            point1.time_from_start = Duration(sec=0, nanosec=0)
            
            # End point (target position)
            point2 = JointTrajectoryPoint()
            point2.positions = target_positions
            point2.velocities = [0.0] * len(self.joint_names)
            
            # Calculate time based on maximum joint displacement
            max_displacement = max(abs(t - c) for t, c in zip(target_positions, current_positions))
            # Assume max joint velocity of 1.0 rad/s
            time_sec = max(max_displacement / 1.0, 0.1)  # At least 0.1 second
            point2.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            
            goal.trajectory.points = [point1, point2]
            
            # Send goal asynchronously
            self.robot_traj_action_client.send_goal_async(goal)
            self.logging(f"Sent robot trajectory to joints: {target_positions}", "DEBUG")
            
        except Exception as e:
            self.logging(f"Error sending robot trajectory: {str(e)}", "ERROR")

    def _send_gripper_trajectory(self, current_pos: float, target_pos: float):
        """Send gripper trajectory action goal"""
        try:
            if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
                self.logging("Gripper action server not available", "WARN")
                return
            
            # Create trajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.gripper_joint_names
            
            # Start point (current position)
            point1 = JointTrajectoryPoint()
            point1.positions = [current_pos]
            point1.time_from_start = Duration(sec=0, nanosec=0)
            
            # End point (target position)
            point2 = JointTrajectoryPoint()
            point2.positions = [target_pos]
            # Calculate time based on distance (1 rad/s as specified)
            distance = abs(target_pos - current_pos)
            time_sec = max(distance / 1.0, 0.1)  # At least 0.1 second
            point2.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1) * 1e9))
            
            goal.trajectory.points = [point1, point2]
            
            # Send goal asynchronously
            self.gripper_action_client.send_goal_async(goal)
            
        except Exception as e:
            self.logging(f"Error sending gripper trajectory: {str(e)}", "ERROR")

    def get_state(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """
        Get current robot or gripper state.
        
        Args:
            robot_id: 'xarm5' or 'xarm_gripper'
            
        Returns:
            Dictionary containing robot state data
        """
        if not self.robot_active:
            return None
        
        try:
            if robot_id == "xarm5":
                if self.current_robot_joint_state is None:
                    return None
                
                # Get EEF pose from TF
                eef_pose = self.get_eef_pose_from_tf('link_base', 'link5')
                if eef_pose is None:
                    eef_pose = Pose()  # Use default if TF not available
                
                robot_state = {
                    'robot_id': robot_id,
                    'joint_state': self.current_robot_joint_state,
                    'end_effector_pose': eef_pose,
                    'timestamp': self.get_clock().now().to_msg()
                }
                return robot_state
                
            elif robot_id == "xarm_gripper":
                if self.current_gripper_joint_state is None:
                    return None
                
                # Calculate gripper center pose (average of two fingers)
                gripper_pose = self.get_gripper_center_pose()
                if gripper_pose is None:
                    gripper_pose = Pose()
                
                gripper_state = {
                    'robot_id': robot_id,
                    'joint_state': self.current_gripper_joint_state,
                    'end_effector_pose': gripper_pose,
                    'timestamp': self.get_clock().now().to_msg()
                }
                return gripper_state
            
            return None
            
        except Exception as e:
            self.handle_error(f"Error getting state for {robot_id}: {str(e)}")
            return None

    def get_eef_pose_from_tf(self, base_frame: str, eef_frame: str) -> Optional[Pose]:
        """Get end effector pose from TF tree"""
        try:
            transform = self.tf_buffer.lookup_transform(
                base_frame,
                eef_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.logging(f"Could not get transform {base_frame} -> {eef_frame}: {str(e)}", "DEBUG")
            return None

    def get_gripper_center_pose(self) -> Optional[Pose]:
        """Calculate gripper center pose as average of right and left fingers"""
        try:
            right_pose = self.get_eef_pose_from_tf('xarm_gripper_base_link', 'right_finger')
            left_pose = self.get_eef_pose_from_tf('xarm_gripper_base_link', 'left_finger')
            
            if right_pose and left_pose:
                center_pose = Pose()
                center_pose.position.x = (right_pose.position.x + left_pose.position.x) / 2.0
                center_pose.position.y = (right_pose.position.y + left_pose.position.y) / 2.0
                center_pose.position.z = (right_pose.position.z + left_pose.position.z) / 2.0
                # Use right finger orientation as representative
                center_pose.orientation = right_pose.orientation
                return center_pose
            
            return None
            
        except Exception as e:
            self.logging(f"Error calculating gripper center pose: {str(e)}", "DEBUG")
            return None

    def _create_robot_state_msg(self, robot_state: Dict[str, Any]) -> RobotState:
        """Create RobotState message from robot state data"""
        msg = RobotState()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = robot_state['timestamp']
        msg.header.frame_id = "xarm5"
        
        # Set robot ID
        msg.robot_id = robot_state['robot_id']
        
        # Set joint state
        msg.joint_states = robot_state['joint_state']
        
        # Set end effector pose (with both quaternion and euler)
        msg.end_effector_pose = robot_state['end_effector_pose']
        
        return msg

    def publish_state(self):
        """Publish current robot and gripper state - called by timer"""
        if not self.robot_active:
            return
        
        try:
            # Publish robot state
            robot_state = self.get_state("xarm5")
            if robot_state and 'robot_state' in self.compo_publishers:
                msg = self._create_robot_state_msg(robot_state)
                self.publish_msg('robot_state', msg)
                self.logging("Published robot state", "DEBUG")
            
            # Publish gripper state
            gripper_state = self.get_state("xarm_gripper")
            if gripper_state and 'gripper_state' in self.compo_publishers:
                msg = self._create_robot_state_msg(gripper_state)
                self.publish_msg('gripper_state', msg)
                self.logging("Published gripper state", "DEBUG")
                
        except Exception as e:
            self.handle_error(f"Error publishing states: {str(e)}")

    def home(self):
        """Move robot to initial pose"""
        try:
            self.logging("Moving robot to home position", "INFO")
            
            # Create home pose
            home_pose = Pose()
            home_pose.position.x = self.robot_initial_pose[0]
            home_pose.position.y = self.robot_initial_pose[1]
            home_pose.position.z = self.robot_initial_pose[2]
            home_pose.orientation.x = self.robot_initial_pose[3]
            home_pose.orientation.y = self.robot_initial_pose[4]
            home_pose.orientation.z = self.robot_initial_pose[5]
            home_pose.orientation.w = self.robot_initial_pose[6]
            
            # Publish home pose
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'link_base'
            pose_stamped.pose = home_pose
            
            self.cartesian_target_pub.publish(pose_stamped)
            
            # Home gripper (fully open)
            self._send_gripper_trajectory(
                self.current_gripper_joint_state.position[0] if self.current_gripper_joint_state else 0.5,
                0.0
            )
            
            self.logging("Robot homing initiated", "INFO")
            
        except Exception as e:
            self.handle_error(f"Error in home: {str(e)}")

    def execute_command(self, robot_id: str, command: Any) -> None:
        """
        Execute robot command.
        This is a wrapper for compatibility with base class.
        Actual execution is done through move_cartesian and move_joint.
        """
        try:
            if not self.robot_active:
                self.logging("Robot not active, ignoring command", "WARN")
                return
            
            self.logging(f"Execute command called for {robot_id}: {command}", "DEBUG")
            
        except Exception as e:
            self.handle_error(f"Error executing command: {str(e)}")

    def register_timer(self):
        """Register timer for state publishing"""
        if self.compo_timers:
            for t in self.compo_timers:
                t.cancel()
            self.compo_timers.clear()
        
        # Calculate timer period from frequency
        timer_period = 1.0 / self.freq
        timer = self.create_timer(timer_period, self.publish_state)
        self.compo_timers.append(timer)
        self.logging(f"Registered timer for state publishing at {self.freq}Hz", "INFO")

    def _initialize_robot(self):
        """Initialize robot connection and state"""
        try:
            self.logging("Initializing XArm5 robot driver", "INFO")
            
            if self.simulated:
                self.logging("Running in simulated mode", "INFO")
                self.robot_connected = True
            
            # Set robot active
            self.robot_active = True
            
            self.logging("XArm5 robot driver initialized successfully", "INFO")
            
        except Exception as e:
            self.handle_error(f"Error initializing robot: {str(e)}")
    
    def start_component(self):
        if self.compo_timers == [] or not self.compo_timers:
            self.register_timer() #defined in the bottom node
        else:
            for timer in self.compo_timers:
                # if timer.is_canceled():
                timer.reset()
        self.robot_active = True

    def urgent_stop(self):
        """Emergency stop - stop robot motion immediately"""
        self.logging("Emergency stop triggered", "WARN")
        
        # Stop robot motion by publishing current pose
        if self.robot_active and self.current_robot_joint_state:
            try:
                current_pose = self.get_eef_pose_from_tf('link_base', 'link5')
                self.home()
                # if current_pose:
                #     pose_stamped = PoseStamped()
                #     pose_stamped.header.stamp = self.get_clock().now().to_msg()
                #     pose_stamped.header.frame_id = 'link_base'
                #     pose_stamped.pose = current_pose
                #     self.cartesian_target_pub.publish(pose_stamped)
                
                # # Stop gripper
                # if self.current_gripper_joint_state:
                #     current_gripper_pos = self.current_gripper_joint_state.position[0]
                #     self._send_gripper_trajectory(current_gripper_pos, current_gripper_pos)
                    
            except Exception as e:
                self.logging(f"Error during urgent stop: {str(e)}", "ERROR")
        
        self.robot_active = False
        # self.stop_component()
        
        self.logging("Emergency stop completed", "WARN")

    # ============ Utility Methods ============
    
    def _apply_relative_cartesian(self, current: Pose, delta: Pose, scale: float) -> Pose:
        """Apply relative cartesian motion with scale factor"""
        target = Pose()
        
        # Apply position delta
        target.position.x = current.position.x + delta.position.x * scale
        target.position.y = current.position.y + delta.position.y * scale
        target.position.z = current.position.z + delta.position.z * scale
        
        # Apply orientation delta (quaternion multiplication)
        # Scale the rotation delta
        current_rot = R.from_quat([
            current.orientation.x,
            current.orientation.y,
            current.orientation.z,
            current.orientation.w
        ])
        delta_rot = R.from_quat([
            delta.orientation.x,
            delta.orientation.y,
            delta.orientation.z,
            delta.orientation.w
        ])
        
        # Scale rotation by converting to rotvec, scaling, and back
        delta_rotvec = delta_rot.as_rotvec()
        scaled_delta_rot = R.from_rotvec(delta_rotvec * scale)
        
        # Combine rotations
        target_rot = current_rot * scaled_delta_rot
        target_quat = target_rot.as_quat()
        
        target.orientation.x = target_quat[0]
        target.orientation.y = target_quat[1]
        target.orientation.z = target_quat[2]
        target.orientation.w = target_quat[3]
        
        return target

    def _ensure_quaternion(self, pose: Pose) -> Pose:
        """Ensure orientation is in quaternion format"""
        # If orientation looks like euler (values > 2*pi), convert
        # This is a simplification - in practice you'd have a flag
        quat_sum = (pose.orientation.x**2 + pose.orientation.y**2 + 
                    pose.orientation.z**2 + pose.orientation.w**2)
        
        if abs(quat_sum - 1.0) > 0.1:  # Not normalized quaternion
            # Assume it's euler angles in x, y, z (roll, pitch, yaw)
            rot = R.from_euler('xyz', [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            ])
            quat = rot.as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
        
        return pose
