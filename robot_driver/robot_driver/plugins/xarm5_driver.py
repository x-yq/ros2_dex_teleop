import rclpy
from rclpy.timer import Timer
import json
import yaml
import numpy as np
from typing import Dict, List, Any, Optional
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time, Duration
from component_interfaces.msg import JointAngles, RobotState
from core_bases.robot_driver_base import RobotDriverBase
from omegaconf import OmegaConf, DictConfig
import time
import math


class XArm5Driver(RobotDriverBase):
    """
    ROS2 Driver node for XArm5 robot.
    Handles joint angle commands and publishes robot state.
    """

    def __init__(self, layer_cn_id: str, simulated: bool = True, ros2_domain_id: Optional[int] = None):
        super().__init__(layer_cn_id, simulated, ros2_domain_id)
        
        # Driver specific attributes
        self.freq = 90  # Default frequency
        self.robot_active = False
        self.robot_connected = False

        self.robot_ip = None
        
        # Robot state
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_joint_efforts = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # End effector pose (dummy forward kinematics)
        self.end_effector_pose = Pose()
        self.end_effector_pose.position = Point(x=0.5, y=0.0, z=0.3)
        self.end_effector_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Command tracking
        self.last_command_time = None
        self.command_timeout = 5.0  # seconds

        self._load_my_config()

        self._initialize_robot()

        # # Start state publishing timer
        # self._start_state_publishing()
            
    def _load_my_config(self):
        """Load all required configurations using ComponentNode methods"""
        try:
            # Load component-specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                # Extract processing frequency from plugin parameters if available
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'xarm5_driver' or plugin.get('_target_', '').endswith('XArm5Driver'):
                        plugin_params = plugin.get('params', {})
                        self.freq = plugin_params.get("freq", self.freq)
                        if 'joint_names' in plugin_params:
                            self.joint_names = plugin_params['joint_names']
                        if 'joint_limits' in plugin_params:
                            # Store joint limits for validation
                            self.joint_limits = plugin_params['joint_limits']
                        self.get_logger().info(f"Loaded processor config: {self.freq}Hz")
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load configurations: {str(e)}")

    def register_pub_sub(self):
        """Register publishers and subscribers based on data stream configuration."""
        try:
            # Use the loaded data_stream_config instead of loading again
            if not self.data_stream_config or 'topics' not in self.data_stream_config:
                self.handle_error("No data stream configuration available")
                return
            
            topics = self.data_stream_config.get('topics', [])
            
            # Register publishers and subscribers
            for topic in topics:
                topic_id = topic.get('id')
                msg_type = topic.get('msg_type')
                publisher = topic.get('publisher')
                subscribers = topic.get('subscribers', [])
                
                # Create publishers for topics where this node is the publisher
                if publisher == self.layer_cn_id:
                    if msg_type == 'component_interfaces/msg/RobotState':
                        self.compo_publishers[topic_id] = self.create_publisher(RobotState, topic_id, 10)
                        self.logging(f"Created publisher for {topic_id} with type {msg_type}", "INFO")
                    else:
                        self.logging(f"Unsupported publisher message type: {msg_type}", "WARN")
                
                # Create subscribers for topics where this node is a subscriber
                if self.layer_cn_id in subscribers:
                    if msg_type == 'component_interfaces/msg/JointAngles':
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            JointAngles,
                            topic_id,
                            lambda msg, tid=topic_id: self.cmd_sub_callback(msg, tid),
                            10
                        )
                        self.logging(f"Created subscriber for {topic_id} with type {msg_type}", "INFO")
                    else:
                        self.logging(f"Unsupported subscriber message type: {msg_type}", "WARN")
            
            self.logging(f"Registered {len(self.compo_publishers)} publishers and {len(self.compo_subscribers)} subscribers", "INFO")
            
        except Exception as e:
            self.handle_error(f"Error in register_pub_sub: {str(e)}")

    def cmd_sub_callback(self, msg: JointAngles, topic_id: str):
        """Callback for joint angle commands."""
        try:
            ##NOTE
            # if self.state!='running':
            #     return  
 

            self.logging(f"Received joint angles command on {topic_id}", "DEBUG")
            
            # Validate robot ID
            if hasattr(msg, 'robot_id') and msg.robot_id != "xarm5_robot":
                self.logging(f"Command for different robot: {msg.robot_id}, ignoring", "WARN")
                return
            
            # Validate joint count
            if len(msg.joint_angles) != len(self.joint_names):
                self.handle_error(f"Invalid joint count: expected {len(self.joint_names)}, got {len(msg.joint_angles)}")
                return
            
            # Update target positions
            self.target_joint_positions = list(msg.joint_angles)
            self.last_command_time = time.time()
            
            # Execute command
            self.execute_command("xarm5_robot", {
                "type": "joint_angles",
                "positions": self.target_joint_positions,
                "joint_names": msg.joint_names if hasattr(msg, 'joint_names') else self.joint_names
            })
            
            self.logging(f"Updated target joint positions: {self.target_joint_positions}", "DEBUG")
            
        except Exception as e:
            self.handle_error(f"Error in cmd_sub_callback: {str(e)}")

    def get_state(self, robot_id: str) -> Any:
        """Get current robot state."""
        if not self.robot_active:
            return None
            
        try:
            # Simulate robot motion towards target positions
            self._update_robot_simulation()
            
            # Create robot state data
            robot_state = {
                'robot_id': robot_id,
                'joint_names': self.joint_names.copy(),
                'joint_positions': self.current_joint_positions.copy(),
                'joint_velocities': self.current_joint_velocities.copy(),
                'joint_efforts': self.current_joint_efforts.copy(),
                'end_effector_pose': self.end_effector_pose,
                'timestamp': self.get_clock().now().to_msg()
            }
            
            return robot_state
            
        except Exception as e:
            self.handle_error(f"Error getting state for robot {robot_id}: {str(e)}")
            return None

    def _update_robot_simulation(self):
        """Update simulated robot state - move towards target positions."""
        if not self.simulated:
            return
            
        dt = 1.0 / self.freq  # Time step
        max_velocity = 1.0  # rad/s
        position_tolerance = 0.001  # rad
        
        # Move each joint towards target position
        for i in range(len(self.joint_names)):
            target = self.target_joint_positions[i]
            current = self.current_joint_positions[i]
            
            # Calculate position error
            error = target - current
            
            if abs(error) > position_tolerance:
                # Calculate desired velocity (proportional control)
                desired_velocity = np.clip(error * 5.0, -max_velocity, max_velocity)
                
                # Update position
                self.current_joint_positions[i] += desired_velocity * dt
                self.current_joint_velocities[i] = desired_velocity
                
                # Simulate some effort (torque)
                self.current_joint_efforts[i] = error * 10.0  # Simple proportional torque
            else:
                self.current_joint_velocities[i] = 0.0
                self.current_joint_efforts[i] = 0.0
        
        # Update end effector pose using simple forward kinematics
        self._update_end_effector_pose()

    def _update_end_effector_pose(self):
        """Update end effector pose using simplified forward kinematics."""
        # Simplified 5-DOF arm forward kinematics
        # In reality, this would use proper DH parameters
        
        # Link lengths (approximate for XArm5)
        l1, l2, l3, l4, l5 = 0.267, 0.289, 0.077, 0.342, 0.097
        
        # Joint angles
        q1, q2, q3, q4, q5 = self.current_joint_positions
        
        # Simplified forward kinematics (not exact but reasonable for demo)
        x = (l2 * np.cos(q2) + l3 * np.cos(q2 + q3) + l4 * np.cos(q2 + q3 + q4)) * np.cos(q1)
        y = (l2 * np.cos(q2) + l3 * np.cos(q2 + q3) + l4 * np.cos(q2 + q3 + q4)) * np.sin(q1)
        z = l1 + l2 * np.sin(q2) + l3 * np.sin(q2 + q3) + l4 * np.sin(q2 + q3 + q4)
        
        self.end_effector_pose.position.x = float(x)
        self.end_effector_pose.position.y = float(y)
        self.end_effector_pose.position.z = float(z)
        
        # Simplified orientation (roll-pitch-yaw to quaternion)
        roll = q4
        pitch = q2 + q3
        yaw = q1
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        self.end_effector_pose.orientation.w = float(cr * cp * cy + sr * sp * sy)
        self.end_effector_pose.orientation.x = float(sr * cp * cy - cr * sp * sy)
        self.end_effector_pose.orientation.y = float(cr * sp * cy + sr * cp * sy)
        self.end_effector_pose.orientation.z = float(cr * cp * sy - sr * sp * cy)

    def _create_robot_state_msg(self, robot_state: Dict[str, Any]) -> RobotState:
        """Create RobotState message from robot state data."""
        msg = RobotState()
        
        # Set header
        msg.header = Header()
        msg.header.stamp = robot_state['timestamp']
        msg.header.frame_id = "xarm5_robot"  # From data_stream.yaml frame_id
        
        # Set robot ID
        msg.robot_id = robot_state['robot_id']
        
        # Set joint state
        msg.joint_states = JointState()
        msg.joint_states.header = msg.header
        msg.joint_states.name = robot_state['joint_names']
        msg.joint_states.position = robot_state['joint_positions']
        msg.joint_states.velocity = robot_state['joint_velocities']
        msg.joint_states.effort = robot_state['joint_efforts']
        
        # Set end effector pose
        msg.end_effector_pose = robot_state['end_effector_pose']
        
        return msg

    def publish_robot_state(self):
        """Publish current robot state - called by timer."""
        if not self.robot_active:
            return
            
        try:
            ##NOTE
            # if self.state!='running':
            #     return  

                
            # Get current robot state
            robot_state = self.get_state("xarm5_robot")
            
            if robot_state is not None:
                # Create and publish RobotState message
                if 'robot_state' in self.compo_publishers:
                    msg = self._create_robot_state_msg(robot_state)
                    self.publish_msg('robot_state', msg)
                    self.logging("Published robot state", "DEBUG")
                else:
                    self.logging("No publisher found for robot_state", "WARN")
            else:
                self.logging("No robot state available", "DEBUG")
                
        except Exception as e:
            self.handle_error(f"Error publishing robot state: {str(e)}")

    def home(self):
        """Move robot to home position."""
        try:
            self.logging("Moving robot to home position", "INFO")
            
            # Home position (all joints at 0)
            home_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.target_joint_positions = home_positions
            self.last_command_time = time.time()
            
            self.logging("Robot homing initiated", "INFO")
            
        except Exception as e:
            self.handle_error(f"Error in home: {str(e)}")

    def execute_command(self, robot_id: str, command: Any) -> None:
        """Execute robot command."""
        try:
            if not self.robot_active:
                self.logging("Robot not active, ignoring command", "WARN")
                return
                
            command_type = command.get('type', 'unknown')
            
            if command_type == 'joint_angles':
                positions = command.get('positions', [])
                if len(positions) == len(self.joint_names):
                    self.get_logger().info(f"Executing joint angles command: {positions}")
                    # Target positions already set in callback
                else:
                    self.handle_error(f"Invalid joint angles command: wrong number of joints")
                    
            elif command_type == 'stop':
                self.logging("Executing stop command", "WARN")
                # Stop motion by setting target to current position
                self.target_joint_positions = self.current_joint_positions.copy()
                
            elif command_type == 'home':
                self.home()
                
            else:
                self.logging(f"Unknown command type: {command_type}", "WARN")
                
        except Exception as e:
            self.handle_error(f"Error executing command for robot {robot_id}: {str(e)}")

    
    def register_timer(self):
        
        if self.compo_timers != []:
            for t in self.compo_timers:
                t.cancel()

        # Calculate timer period from frequency
        timer_period = 1.0 / self.freq
        timer = self.create_timer(timer_period, self.publish_robot_state)
        self.logging(f"Registered a timer for state publishing at {self.freq}Hz", "INFO")
        self.compo_timers.append(timer)


    def _initialize_robot(self):
        """Initialize robot connection and state."""
        try:
            self.logging("Initializing XArm5 robot", "INFO")
            
            if self.simulated:
                self.logging("Running in simulated mode", "INFO")
                self.robot_connected = True
            
            # Set initial state
            self.robot_active = True
            self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            self.logging("XArm5 robot initialized successfully", "INFO")
            
        except Exception as e:
            self.handle_error(f"Error initializing robot: {str(e)}")

    def urgent_stop(self):
        """Emergency stop - stop robot motion immediately."""
        self.logging("Emergency stop triggered", "WARN")
        
        # Stop robot motion
        if self.robot_active:
            self.execute_command("xarm5_robot", {"type": "stop", "urgent": True})
            self.robot_active = False

        self.stop_component()
        self.set_state("disconnected")
            
        self.logging("Emergency stop completed", "WARN")
