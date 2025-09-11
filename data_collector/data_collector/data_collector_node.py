#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import json
import yaml
import sys
import importlib
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String, Header
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import serialize_message
from system_interfaces.srv import GetConfig, StartDataCollection, StopDataCollection, Logging
from core_bases.component_node import ComponentNode

class DataCollector(ComponentNode):
    """Node responsible for collecting and recording demonstration data"""
    
    def __init__(self, layer_cn_id: str = 'data_collector', ros2_domain_id: int = None):
        super().__init__(layer_cn_id, ros2_domain_id)

        # Configuration data
        self.demo_config = {}
        self.data_stream_config = {}
        self.init_topics = []
        self.topic_metadata = {}
        self.init_demo_id = ''
        
        # State tracking
        self.active_collections = {}
        self.collection_subscribers = {}  # {demo_id: {topic_id: subscriber}}
        self.collection_log_files = {}    # {demo_id: {topic_id: file_handle}}
        
        # Default configuration
        self.demo_root_path = "/tmp/demos"
        self.supported_formats = ['rosbag2', 'hdf5', 'vla']
        self.supported_codecs = ['h264', 'h265']


        # Initialize the node
        self._initialize_node()
        
        self.srv_callback_group = ReentrantCallbackGroup()

        # Service servers
        self.start_data_collection_srv = self.create_service(
            StartDataCollection, 'start_data_collection', 
            self.start_data_collection_srv_callback, callback_group=self.srv_callback_group)
            
        self.stop_data_collection_srv = self.create_service(
            StopDataCollection, 'stop_data_collection', 
            self.stop_data_collection_srv_callback, callback_group=self.srv_callback_group)
        
        
        self.get_logger().info('DataCollector node initialized successfully')


    def _update_demo_config(self, request):
        """Update or create demo configuration based on the request.
        
        If the demo_id doesn't exist in config, create a new entry using request values.
        If request attributes are empty, use values from the initial demo config.
        If demo_id exists, update only the non-empty attributes from the request.
        """
        demo_id = request.demo_id
        
        if demo_id not in self.demo_config:
            self.get_logger().info(f"Registering new demo config for: {demo_id}")
            
            # Create new config entry, using request values or fallback to init_demo_id config
            new_config = {}
            
            # Use request values if provided, otherwise use init_demo_id config values
            new_config['demo_id'] = demo_id
            new_config['demo_root_path'] = request.demo_root_path if request.demo_root_path else self.demo_config[self.init_demo_id].get('demo_root_path', self.demo_root_path)
            new_config['topics'] = list(request.topics) if request.topics else self.demo_config[self.init_demo_id].get('topics', [])
            new_config['video_codec'] = request.video_codec if request.video_codec else self.demo_config[self.init_demo_id].get('video_codec', 'h264')
            new_config['storage_format'] = request.storage_format if request.storage_format else self.demo_config[self.init_demo_id].get('storage_format', 'vla')
            
            # Store the new config
            self.demo_config[demo_id] = new_config
            
            self.get_logger().info(f"Created new config for demo {demo_id}: {new_config}")
            
        else:
            self.get_logger().info(f"Updating existing demo config for: {demo_id}")
            
            # Update only the non-empty attributes from the request
            current_config = self.demo_config[demo_id]
            
            if request.demo_root_path:
                current_config['demo_root_path'] = request.demo_root_path
                self.get_logger().debug(f"Updated demo_root_path to: {request.demo_root_path}")
                
            if request.topics:
                current_config['topics'] = list(request.topics)
                self.get_logger().debug(f"Updated topics to: {request.topics}")
                
            if request.video_codec:
                current_config['video_codec'] = request.video_codec
                self.get_logger().debug(f"Updated video_codec to: {request.video_codec}")
                
            if request.storage_format:
                current_config['storage_format'] = request.storage_format
                self.get_logger().debug(f"Updated storage_format to: {request.storage_format}")
            
            self.get_logger().info(f"Updated config for demo {demo_id}: {current_config}")
            
    def _load_root_configs(self):
        """Load root configuration files needed by this component"""

        # Load demo configuration
        demo_config = self.load_demo_config()
        if not demo_config:
            self.get_logger().error("Failed to load demo configuration")
            return False

        # Load data stream configuration
        self.data_stream_config = self.load_data_stream_config()
        if not self.data_stream_config:
            self.get_logger().error("Failed to load data stream configuration")
            return False
            
        # Parse topics to collect from demo confi
        self.init_demo_id = self.demo_config.get('demo_id', "demo")
        self.demo_config[self.init_demo_id] = demo_config
        self.init_topics = demo_config.get('topics', [])
        self.get_logger().info(f"Topics to collect: {self.init_topics}")
        
        # Parse topic metadata from data stream config
        self._parse_topic_metadata()
        
        return True

    def _parse_topic_metadata(self):
        """Parse topic metadata from data stream configuration"""
        self.topic_metadata = {}
        
        if 'topics' not in self.data_stream_config:
            self.get_logger().warn("No topics found in data stream configuration")
            return
            
        for topic_config in self.data_stream_config['topics']:
            topic_id = topic_config['id']
            self.topic_metadata[topic_id] = {
                'msg_type': topic_config['msg_type'],
                'publish_freq': topic_config.get('publish_freq', 30.0),
                'frame_id': topic_config.get('frame_id', ''),
                'shape': topic_config.get('shape', []),
                'dtype': topic_config.get('dtype', 'float32'),
                'encoding': topic_config.get('encoding', '')
            }
            
        self.get_logger().info(f"Parsed metadata for {len(self.topic_metadata)} topics")


    def start_data_collection_srv_callback(self, request, response):
        """Handle start data collection requests"""
        try:
            demo_id = request.demo_id.strip()
            
            # Validate demo_id
            if not demo_id:
                response.success = False
                response.topic_ids = []
                response.storage_path = ""
                response.start_time = Time()
                self.get_logger().error("Demo ID cannot be empty")
                return response
                
            # Check if demo already exists
            if demo_id in self.active_collections:
                response.success = False
                response.topic_ids = []
                response.storage_path = ""
                response.start_time = Time()
                self.get_logger().warn(f"Demo {demo_id} is already being collected")
                return response
                
            # Determine storage path

            ## BUG: make it async!!!!
            # self._get_demo_config()
            self._update_demo_config(request)

            demo_root = self.demo_config[demo_id].get('demo_root_path', self.demo_root_path)
            storage_path = os.path.join(demo_root, demo_id)
            
            # Create storage directory
            os.makedirs(storage_path, exist_ok=True)
            
            # Validate storage format and codec
            storage_format = self.demo_config[demo_id].get('storage_format', 'vla')
            video_codec = self.demo_config[demo_id].get('video_codec', 'h264')
            
            if storage_format not in self.supported_formats:
                self.get_logger().warn(f"Unsupported storage format {storage_format}, using vla")
                storage_format = 'vla'
                
            if video_codec not in self.supported_codecs:
                self.get_logger().warn(f"Unsupported video codec {video_codec}, using h264")
                video_codec = 'h264'
            
            # Subscribe to topics and create log files
            topics = self.demo_config[demo_id].get('topics', self.init_topics)
            success = self._start_topic_collection(demo_id, topics, storage_path)
            
            if not success:
                response.success = False
                response.topic_ids = []
                response.storage_path = ""
                response.start_time = Time()
                self.get_logger().error(f"Failed to start topic collection for {demo_id}")
                return response
            
            # Create collection metadata
            start_time = self.get_clock().now()
            collection_info = {
                'demo_id': demo_id,
                'storage_path': storage_path,
                'topics': topics,
                'storage_format': storage_format,
                'video_codec': video_codec,
                'start_time': start_time,
            }
            
            # Track active collection
            self.active_collections[demo_id] = collection_info
            
            # Save initial metadata
            self._save_collection_metadata(collection_info, storage_path)
            
            # Prepare response
            response.success = True
            response.topic_ids = topics
            response.storage_path = storage_path
            response.start_time = start_time.to_msg()
            
            # Log successful start
            # self.logging(f"Started data collection for demo {demo_id} on {len(topics)} topics", 'INFO')
            self.get_logger().info(f"Data collection started: {demo_id} -> {storage_path}")
            
        except Exception as e:
            response.success = False
            response.topic_ids = []
            response.storage_path = ""
            response.start_time = Time()
            
            error_msg = f"Failed to start data collection for {demo_id}: {e}"
            # self.logging(error_msg, 'ERROR')
            self.get_logger().error(error_msg)
            
        return response
        
    def stop_data_collection_srv_callback(self, request, response):
        """Handle stop data collection requests"""
        try:
            self.get_logger().warn("Got stop data collection request!!!")
            demo_id = request.demo_id.strip()
            
            if not demo_id or demo_id not in self.active_collections:
                response.success = False
                response.storage_path = ""
                response.collection_duration = Duration()
                response.message = f"Demo {demo_id} not found in active collections"
                
                self.get_logger().warn(f"Attempted to stop non-existent collection: {demo_id}")
                return response
                
            collection_info = self.active_collections[demo_id]
            
            # Calculate collection duration
            start_time = collection_info['start_time']
            end_time = self.get_clock().now()
            duration_ns = (end_time - start_time).nanoseconds
            
            # Stop topic collection and cleanup
            self._stop_topic_collection(demo_id)
            
            # Update metadata with final statistics
            self._finalize_collection_metadata(collection_info, end_time)
            
            # Prepare response
            response.success = True
            response.storage_path = collection_info['storage_path']
            response.message = f"Data collection completed for demo {demo_id}. "
            
            # Set duration
            duration_msg = Duration()
            duration_msg.sec = int(duration_ns // 1e9)
            duration_msg.nanosec = int(duration_ns % 1e9)
            response.collection_duration = duration_msg
            
            # Log successful completion
            duration_sec = duration_ns / 1e9
            # self.logging( 
            #     f"Completed data collection for demo {demo_id}. "
            #     f"Duration: {duration_sec:.2f}s, "
            #     f"Messages: {sum(collection_info['message_count'].values())}", 'INFO')
            
            self.get_logger().info(f"Data collection completed: {demo_id} ({duration_sec:.2f}s)")
            
            # Remove from active collections
            del self.active_collections[demo_id]
            
        except Exception as e:
            response.success = False
            response.storage_path = ""
            response.collection_duration = Duration()
            response.message = f"Failed to stop data collection: {e}"
            
            error_msg = f"Failed to stop data collection for {demo_id}: {e}"
            # self.logging(error_msg, 'ERROR')
            self.get_logger().error(error_msg)
            
        return response
        
    def _start_topic_collection(self, demo_id, topics, storage_path):
        """Start collecting data from specified topics"""
        try:
            # Initialize storage for this demo
            self.collection_subscribers[demo_id] = {}
            self.collection_log_files[demo_id] = {}
            self.topic_callback_group = ReentrantCallbackGroup()
            
            for topic_id in topics:
                topic_id = topic_id.replace('/', '')
                # Skip if no metadata available
                if topic_id not in self.topic_metadata:
                    self.get_logger().warn(f"No metadata found for topic {topic_id}, skipping")
                    continue
                    
                metadata = self.topic_metadata[topic_id]
                msg_type_str = metadata['msg_type']
                
                # Skip image topics as requested
                if 'Image' in msg_type_str or 'image' in topic_id.lower():
                    self.get_logger().info(f"Skipping image topic: {topic_id}")
                    continue
                
                try:
                    # Resolve message type (using inherited method from ComponentNode)
                    msg_type_class = self._resolve_msg_type(msg_type_str)
                    
                    # # Create QoS profile
                    # qos_profile = QoSProfile(
                    #     reliability=ReliabilityPolicy.BEST_EFFORT,
                    #     durability=DurabilityPolicy.VOLATILE,
                    #     depth=10
                    # )
                    
                    # Create dynamic callback
                    callback = self._create_topic_callback(demo_id, topic_id)
                    
                    ##NOTE
                    # Create subscriber
                    subscriber = self.create_subscription(
                        msg_type_class, topic_id, callback, 10)
                    
                    # Store subscriber
                    self.collection_subscribers[demo_id][topic_id] = subscriber
                    # self.collection_subscribers[demo_id][topic_id] = None
                    
                    # Create log file
                    log_file_path = os.path.join(storage_path, f"{topic_id.replace('/', '_')}.log")
                    log_file = open(log_file_path, 'w')
                    self.collection_log_files[demo_id][topic_id] = log_file
                    
                    # Write header to log file
                    header_info = {
                        'topic_id': topic_id,
                        'msg_type': msg_type_str,
                        'metadata': metadata,
                        'start_time': datetime.now().isoformat()
                    }
                    log_file.write(f"# HEADER: {json.dumps(header_info)}\n")
                    log_file.flush()
                    
                    self.get_logger().info(f"Started collecting topic: {topic_id}")
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to setup collection for topic {topic_id}: {e}")
                    
            return len(self.collection_subscribers[demo_id]) > 0
            # return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to start topic collection: {e}")
            return False
    
    def _create_topic_callback(self, demo_id, topic_id):
        """Create a dynamic callback for a specific topic"""
        def topic_callback(msg):
            try:
                if demo_id not in self.active_collections:
                    return
                                    
                # Write to log file
                ##NOTE: Dummy method
                if demo_id in self.collection_log_files and topic_id in self.collection_log_files[demo_id]:
                    log_file = self.collection_log_files[demo_id][topic_id]
                    
                    # Create log entry
                    timestamp = self.get_clock().now()
                    log_entry = {
                        'timestamp': timestamp.nanoseconds,
                        'data': self._serialize_message_for_logging(msg)
                    }
                    
                    # Write to file
                    log_file.write(f"{json.dumps(log_entry)}\n")
                    log_file.flush()
                
            except Exception as e:
                self.get_logger().error(f"Error in callback for {demo_id}/{topic_id}: {e}")
                
        return topic_callback

    def _serialize_message_for_logging(self, msg):
        ##NOTE: turn to dynamic. Dummy method
        """Convert ROS message to loggable format, handling Header objects"""
        try:
            # Handle specific message types based on their structure
            if hasattr(msg, 'header'):
                # Start with basic header info (convert to dict to avoid serialization issues)
                result = {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    }
                }
                
                # Handle JointAngles message
                if hasattr(msg, 'joint_angles') and hasattr(msg, 'joint_names'):
                    result.update({
                        'send_id': msg.send_id,
                        'robot_id': msg.robot_id,
                        'joint_names': list(msg.joint_names),
                        'joint_angles': list(msg.joint_angles),
                        'joint_velocities': list(msg.joint_velocities) if hasattr(msg, 'joint_velocities') else [],
                        'joint_efforts': list(msg.joint_efforts) if hasattr(msg, 'joint_efforts') else [],
                        'timeout': {
                            'sec': msg.timeout.sec,
                            'nanosec': msg.timeout.nanosec
                        } if hasattr(msg, 'timeout') else None
                    })
                    
                # Handle RobotState message
                elif hasattr(msg, 'joint_states') and hasattr(msg, 'end_effector_pose'):
                    result.update({
                        'send_id': msg.send_id,
                        'robot_id': msg.robot_id,
                        'joint_names': list(msg.joint_names),
                        'joint_states': {
                            'name': list(msg.joint_states.name),
                            'position': list(msg.joint_states.position),
                            'velocity': list(msg.joint_states.velocity),
                            'effort': list(msg.joint_states.effort)
                        },
                        'end_effector_pose': {
                            'position': {
                                'x': msg.end_effector_pose.position.x,
                                'y': msg.end_effector_pose.position.y,
                                'z': msg.end_effector_pose.position.z
                            },
                            'orientation': {
                                'x': msg.end_effector_pose.orientation.x,
                                'y': msg.end_effector_pose.orientation.y,
                                'z': msg.end_effector_pose.orientation.z,
                                'w': msg.end_effector_pose.orientation.w
                            }
                        }
                    })
                    
                # Handle RawHandKeypoint message
                elif hasattr(msg, 'fingertips') and hasattr(msg, 'finger_names'):
                    result.update({
                        'send_id': msg.send_id,
                        'fingertips': [{
                            'position': {'x': tip.position.x, 'y': tip.position.y, 'z': tip.position.z},
                            'orientation': {'x': tip.orientation.x, 'y': tip.orientation.y, 'z': tip.orientation.z, 'w': tip.orientation.w}
                        } for tip in msg.fingertips],
                        'finger_names': list(msg.finger_names),
                        'confidence': msg.confidence
                    })
                    
                # Handle ProcessedHandKeypoint message
                elif (hasattr(msg, 'thumb_joints') and hasattr(msg, 'index_joints') and 
                    hasattr(msg, 'middle_joints') and hasattr(msg, 'ring_joints') and 
                    hasattr(msg, 'pinky_joints') and hasattr(msg, 'wrist')):
                    
                    def serialize_pose_array(pose_array):
                        return [{
                            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
                            'orientation': {'x': pose.orientation.x, 'y': pose.orientation.y, 'z': pose.orientation.z, 'w': pose.orientation.w}
                        } for pose in pose_array]
                    
                    result.update({
                        'send_id': msg.send_id,
                        'thumb_joints': serialize_pose_array(msg.thumb_joints),
                        'index_joints': serialize_pose_array(msg.index_joints),
                        'middle_joints': serialize_pose_array(msg.middle_joints),
                        'ring_joints': serialize_pose_array(msg.ring_joints),
                        'pinky_joints': serialize_pose_array(msg.pinky_joints),
                        'wrist': {
                            'position': {'x': msg.wrist.position.x, 'y': msg.wrist.position.y, 'z': msg.wrist.position.z},
                            'orientation': {'x': msg.wrist.orientation.x, 'y': msg.wrist.orientation.y, 'z': msg.wrist.orientation.z, 'w': msg.wrist.orientation.w}
                        },
                        'joint_names': list(msg.joint_names) if hasattr(msg, 'joint_names') else []
                    })
                    
                else:
                    # Generic approach for other message types
                    for field_name, field_type in msg.get_fields_and_field_types().items():
                        if field_name != 'header' and hasattr(msg, field_name):
                            value = getattr(msg, field_name)
                            result[field_name] = self._convert_field_to_serializable(value)
                    
                return result
                
            else:
                # Handle messages without header
                msg_dict = {}
                for field_name, field_type in msg.get_fields_and_field_types().items():
                    if hasattr(msg, field_name):
                        value = getattr(msg, field_name)
                        msg_dict[field_name] = self._convert_field_to_serializable(value)
                        
                return msg_dict
                
        except Exception as e:
            self.get_logger().warn(f"Failed to serialize message: {e}")
            return {'error': 'serialization_failed', 'message_type': str(type(msg))}

    def _convert_field_to_serializable(self, value):
        """Convert a field value to JSON-serializable format"""
        if isinstance(value, (str, int, float, bool, type(None))):
            return value
        elif isinstance(value, (list, tuple)):
            return [self._convert_field_to_serializable(v) for v in value]
        elif hasattr(value, '__dict__'):
            # Handle ROS message objects
            if hasattr(value, 'get_fields_and_field_types'):
                # It's a ROS message
                return self._serialize_message_for_logging(value)
            else:
                # Other objects with __dict__
                return {k: self._convert_field_to_serializable(v) for k, v in value.__dict__.items()}
        else:
            # Convert to string as fallback
            return str(value)

    def _stop_topic_collection(self, demo_id):
        """Stop collecting data for a demo and cleanup resources"""
        try:
            # Destroy subscribers
            if demo_id in self.collection_subscribers:
                for topic_id, subscriber in self.collection_subscribers[demo_id].items():
                    self.destroy_subscription(subscriber)
                    self.get_logger().debug(f"Destroyed subscriber for {topic_id}")
                del self.collection_subscribers[demo_id]
            
            # Close log files
            if demo_id in self.collection_log_files:
                for topic_id, log_file in self.collection_log_files[demo_id].items():
                    try:
                        log_file.close()
                        self.get_logger().debug(f"Closed log file for {topic_id}")
                    except Exception as e:
                        self.get_logger().warn(f"Error closing log file for {topic_id}: {e}")
                del self.collection_log_files[demo_id]
                
            self.get_logger().info(f"Cleaned up collection resources for demo {demo_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error during collection cleanup: {e}")
    
    def _save_collection_metadata(self, collection_info, storage_path):
        """Save collection metadata to file"""
        try:
            metadata_path = os.path.join(storage_path, 'collection_metadata.json')
            
            metadata = {
                'demo_id': collection_info['demo_id'],
                'topics': collection_info['topics'],
                'storage_format': collection_info['storage_format'],
                'video_codec': collection_info['video_codec'],
                'start_time': collection_info['start_time'].nanoseconds,
                'created_by': 'data_collector'
            }
            
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Failed to save metadata: {e}")
            
    def _finalize_collection_metadata(self, collection_info, end_time):
        """Update collection metadata with final statistics"""
        try:
            metadata_path = os.path.join(collection_info['storage_path'], 'collection_metadata.json')
            
            # Calculate final statistics
            duration_ns = (end_time - collection_info['start_time']).nanoseconds
            
            final_metadata = {
                'demo_id': collection_info['demo_id'],
                'topics': collection_info['topics'],
                'storage_format': collection_info['storage_format'],
                'video_codec': collection_info['video_codec'],
                'start_time': collection_info['start_time'].nanoseconds,
                'end_time': end_time.nanoseconds,
                'duration_seconds': duration_ns / 1e9,
                'created_by': 'data_collector',
                'finalized_at': datetime.now().isoformat()
            }
            
            with open(metadata_path, 'w') as f:
                json.dump(final_metadata, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Failed to finalize metadata: {e}")
    
    # Abstract methods implementation
    def register_pub_sub(self):
        """Register publishers and subscribers (not needed for data collector)"""
        # Data collector creates subscribers dynamically, so no static pub/sub setup needed
        pass
    
    def register_timer(self):
        """Register timer for publishers (not needed for data collector)"""
        # Data collector doesn't need periodic publishing
        pass

    def urgent_stop(self):
        """Emergency stop triggered by SystemManager"""
        self.get_logger().warn("URGENT STOP called on DataCollector")
        
        try:
            # Stop all active collections
            active_demos = list(self.active_collections.keys())
            for demo_id in active_demos:
                self.get_logger().warn(f"Force stopping collection: {demo_id}")
                self._stop_topic_collection(demo_id)
                
                # Save emergency metadata
                if demo_id in self.active_collections:
                    collection_info = self.active_collections[demo_id]
                    end_time = self.get_clock().now()
                    self._finalize_collection_metadata(collection_info, end_time)
                    
            # Clear all collections
            self.active_collections.clear()
            self.collection_subscribers.clear()
            self.collection_log_files.clear()
            
            # Call parent stop
            self.stop_component()
            
            self.get_logger().warn("DataCollector emergency stop completed")
            
        except Exception as e:
            self.get_logger().error(f"Error during urgent stop: {e}")

    def _load_my_config(self):
        """Load the component plugin configuration"""
        # Load component-specific config if needed
        component_config = self.load_my_config()
        if component_config:
            self.get_logger().info("Loaded component-specific configuration")
            return component_config
        return {}

    def check_plugins(self):
        """Verify that plugins implement required interfaces."""
        # For data collector, verify configs are loaded properly
        if not self.demo_config or not self.data_stream_config:
            self.get_logger().error("Required configurations not loaded")
            return False
            
        if not self.init_topics:
            self.get_logger().warn("No topics configured for collection")
            return False
            
        self.get_logger().info("DataCollector plugins check passed")
        return True
    
    def get_active_collections(self):
        """Return information about currently active collections"""
        return {demo_id: {
            'demo_id': info['demo_id'],
            'storage_path': info['storage_path'],
            'topics': info['topics'],
            'start_time': info['start_time'].nanoseconds,
        } for demo_id, info in self.active_collections.items()}

def main(args=None):
    rclpy.init(args=args)

    layer_cn_id = 'data_collector'
    ros2_domain_id = 0

    if args:
        # Parse custom arguments
        for i, arg in enumerate(args):
            if arg == "--layer_cn_id" and i + 1 < len(args):
                layer_cn_id = args[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(args):
                ros2_domain_id = int(args[i + 1])
    else:
        # If no args provided, try to get from sys.argv
        for i, arg in enumerate(sys.argv):
            if arg == "--layer_cn_id" and i + 1 < len(sys.argv):
                layer_cn_id = sys.argv[i + 1]
            elif arg == "--ros2_domain_id" and i + 1 < len(sys.argv):
                ros2_domain_id = int(sys.argv[i + 1])

    # Create node instance
    node = DataCollector(layer_cn_id=layer_cn_id, ros2_domain_id=ros2_domain_id)

    try:
        node.logging(f"DataCollection node started with layer_cn_id: {layer_cn_id}", "INFO")

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        node.logging("Keyboard interrupt received", "INFO")
    except Exception as e:
        node.logging(f"Unexpected error: {str(e)}", "ERROR")
    finally:
        node.urgent_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()