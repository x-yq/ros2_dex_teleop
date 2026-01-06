#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import json
import yaml
import sys
import traceback #debug
import importlib
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String, Header
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.serialization import serialize_message
from system_interfaces.srv import GetConfig, StartDataCollection, StopDataCollection, Logging
from core_bases.component_node import ComponentNode
import ros2_numpy as rnp
import robodm
from cv_bridge import CvBridge
import numpy as np
import inspect
from sensor_msgs.msg import Image, PointCloud2, LaserScan, JointState
from geometry_msgs.msg import Point, Pose, PoseStamped, Transform, Vector3, Quaternion
from nav_msgs.msg import OccupancyGrid
from robodm import FeatureType
import time
from collections import defaultdict
from threading import Lock
import h5py

class DataCollector(ComponentNode):
    """Node responsible for collecting and recording demonstration data"""
    
    def __init__(self, layer_cn_id: str = 'data_collector', ros2_domain_id: int = None):
        super().__init__(layer_cn_id, ros2_domain_id)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)



        # Configuration data
        self.demo_config = {}
        self.data_stream_config = {}
        self.trajectories = {}
        self.registered_features = {}

        self.init_topics = []
        self.topic_metadata = {}
        self.init_demo_id = ''
        
        # State tracking
        self.active_collections = {}
        self.collection_subscribers = {}  # {demo_id: {topic_id: subscriber}}
        self.collection_log_files = {}    # {demo_id: {topic_id: file_handle}}

        # demo_id -> feature_name -> list[(timestamp_ns, value)]
        self.data_buffer = {}

        # demo_id -> Lock (only for buffer writes)
        self.buffer_locks = {}

        # Default configuration
        self.demo_root_path = "/tmp/demos"
        self.supported_formats = ['hdf5', 'vla']
        self.supported_codecs = ['libx264', 'libx265', 'libaom-av1', 'ffv1', 'rawvideo', 'rawvideo_pickle', 'rawvideo_pyarrow']


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
            new_config['video_codec'] = request.video_codec if request.video_codec else self.demo_config[self.init_demo_id].get('video_codec', 'libx265')
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

            self._update_demo_config(request)

            demo_root = self.demo_config[demo_id].get('demo_root_path', self.demo_root_path)
            storage_path = os.path.join(demo_root, demo_id)
            
            # Create storage directory
            os.makedirs(storage_path, exist_ok=True)
            
            # Validate storage format and codec
            storage_format = self.demo_config[demo_id].get('storage_format', 'vla')
            video_codec = self.demo_config[demo_id].get('video_codec', 'libx265')
            
            if storage_format not in self.supported_formats:
                self.get_logger().warn(f"Unsupported storage format {storage_format}, using vla")
                storage_format = 'vla'
                
            if video_codec not in self.supported_codecs:
                self.get_logger().warn(f"Unsupported video codec {video_codec}, using libx265")
                video_codec = 'libx265'
            
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
            self.logging(f"Started data collection for demo {demo_id} on {len(topics)} topics", 'INFO')
            
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
        """Start collecting data from specified topics with Robo-DM integration"""
        try:
            # Initialize storage for this demo
            self.collection_subscribers[demo_id] = {}
            self.collection_log_files[demo_id] = {}
            self.topic_callback_group = ReentrantCallbackGroup()
            
            # Initialize Robo-DM trajectory for this demo
            trajectory_path = os.path.join(storage_path, f"{demo_id}.vla")
            
            self.get_logger().info(f"Creating trajectory at: {trajectory_path}")
            
            try:
                # Create trajectory with write mode
                trajectory = robodm.Trajectory(
                    path=trajectory_path, 
                    mode="w"
                )
                
                # Store trajectory reference
                self.trajectories[demo_id] = trajectory
                self.data_buffer[demo_id] = defaultdict(list)
                self.buffer_locks[demo_id] = Lock()
                self.get_logger().info(f"Successfully created trajectory for {demo_id}")
                
            except Exception as traj_error:
                self.get_logger().error(f"Failed to create trajectory: {str(traj_error)}")
                self.get_logger().error(f"Trajectory creation traceback: {traceback.format_exc()}")
                return False
            
            # Initialize CV bridge for image conversion
            try:
                self.cv_bridge = CvBridge()
            except Exception as cv_error:
                self.get_logger().warn(f"Failed to initialize CV bridge: {cv_error}")
            
            # Store registered features for this demo to avoid duplicate registration
            self.registered_features[demo_id] = set()
            
            successful_topics = 0
            
            for topic_id in topics:
                original_topic_id = topic_id
                topic_id = topic_id.replace('/', '')
                
                self.get_logger().info(f"Setting up collection for topic: {original_topic_id} -> {topic_id}")
                
                # Skip if no metadata available
                if topic_id not in self.topic_metadata:
                    self.get_logger().warn(f"No metadata found for topic {topic_id}, skipping")
                    continue
                    
                metadata = self.topic_metadata[topic_id]
                msg_type_str = metadata['msg_type']
                
                try:
                    # Resolve message type
                    msg_type_class = self._resolve_msg_type(msg_type_str)
                    self.get_logger().debug(f"Resolved message type {msg_type_str} to {msg_type_class}")
                    
                    # Register features for this topic dynamically
                    self._register_topic_features(demo_id, topic_id, msg_type_class, metadata)
                    
                    # Create dynamic callback
                    callback = self._create_topic_callback(demo_id, topic_id, msg_type_class, metadata)
                    
                    # Create subscriber
                    subscriber = self.create_subscription(
                        msg_type_class, original_topic_id, callback, 10,
                        callback_group=self.topic_callback_group
                    )
                    
                    # Store subscriber
                    self.collection_subscribers[demo_id][topic_id] = subscriber
                    successful_topics += 1
                    
                    self.get_logger().info(f"Successfully started collecting topic: {original_topic_id} ({msg_type_str})")
                    
                except Exception as e:
                    self.get_logger().error(f"Failed to setup collection for topic {topic_id}: {str(e)}")
                    self.get_logger().error(f"Setup traceback: {traceback.format_exc()}")
                    
            self.get_logger().info(f"Successfully set up {successful_topics}/{len(topics)} topics for collection")
            return successful_topics > 0
            
        except Exception as e:
            self.get_logger().error(f"Failed to start topic collection: {str(e)}")
            self.get_logger().error(f"Start collection traceback: {traceback.format_exc()}")
            return False
    
    def _register_topic_features(self, demo_id, topic_id, msg_type_class, metadata):
        """Dynamically register hierarchical features for a topic based on its message structure"""
        try:
            # Get message instance to inspect its structure
            msg_instance = msg_type_class()
            
            # Parse message fields recursively
            self._parse_and_register_fields(
                demo_id, topic_id, msg_instance, prefix=topic_id
            )
            
            self.get_logger().debug(f"Registered features for topic {topic_id}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to register features for topic {topic_id}: {e}")

    def _parse_and_register_fields(self, demo_id, topic_id, msg_obj, prefix=""):
        """Parse and register fields with leaf check first, before introspection"""
        
        # First, try to treat msg_obj itself as a leaf field
        feature_name = prefix or topic_id
        self.get_logger().debug(f"Checking if {feature_name} is leaf field...")
        
        if self._is_leaf_field(msg_obj):
            if feature_name not in self.registered_features[demo_id]:
                self.registered_features[demo_id].add(feature_name)
                self.get_logger().debug(f"Registered leaf feature: {feature_name}")
            return  # Stop recursion, already leaf

        # If not a leaf, try to get subfields
        if hasattr(msg_obj, 'get_fields_and_field_types'):
            self.get_logger().debug(f"Parsing subfields for {feature_name}...")

            fields = msg_obj.get_fields_and_field_types()
            for field_name, field_type in fields.items():
                if 'header' in field_name or 'send_id' in field_name or 'space' in field_name or 'joint_names' in field_name or 'timeout' in field_name or 'relative' in field_name or 'robot_id' in field_name:
                    continue
                sub_feature_name = f"{prefix}/{field_name}" if prefix else field_name

                # Skip if already registered
                if sub_feature_name in self.registered_features[demo_id]:
                    continue

                try:
                    field_value = getattr(msg_obj, field_name)

                    # Check if this is a leaf based on TYPE definition
                    if self._is_leaf_field_by_type(field_type, field_value):
                        if sub_feature_name not in self.registered_features[demo_id]:
                            self.registered_features[demo_id].add(sub_feature_name)
                            self.get_logger().debug(f"Registered leaf feature: {sub_feature_name} (type: {field_type})")
                    else:
                        # Only recurse if it's actually a nested message (not an empty array)
                        # Check if field_value has subfields to parse
                        if hasattr(field_value, 'get_fields_and_field_types'):
                            self._parse_and_register_fields(demo_id, topic_id, field_value, sub_feature_name)
                        else:
                            # It's not a message type and not recognized as leaf - log warning
                            self.get_logger().warn(
                                f"Field {sub_feature_name} (type: {field_type}) is neither leaf nor nested message. "
                                f"Value type: {type(field_value)}"
                            )

                except Exception as e:
                    self.get_logger().warn(f"Failed to parse field {field_name}: {e}")

    def _is_leaf_field_by_type(self, field_type_str, field_value):
        """Determine if a field is a leaf based on its type definition string"""
        
        # Handle sequence types: sequence<float32>, sequence<float64>, sequence<string>, etc.
        if field_type_str.startswith('sequence<'):
            # Extract the inner type
            inner_type = field_type_str[9:-1]  # Remove 'sequence<' and '>'
            # Basic numeric sequence types are leaf fields
            if inner_type in ['float32', 'float64', 'int8', 'int16', 'int32', 'int64', 
                            'uint8', 'uint16', 'uint32', 'uint64', 'bool', 'string']:
                return True
            # If it's a sequence of anything, treat as leaf
            return True
        
        # Handle bounded sequence types: sequence<float32, 10>
        if 'sequence<' in field_type_str:
            return True
        
        # Handle array types (older ROS1 style: float32[], float64[], etc.)
        if field_type_str.endswith('[]'):
            base_type = field_type_str[:-2]  # Remove '[]'
            # Basic numeric array types are leaf fields
            if base_type in ['float32', 'float64', 'int8', 'int16', 'int32', 'int64', 
                            'uint8', 'uint16', 'uint32', 'uint64', 'bool', 'string']:
                return True
        
        # Basic types
        if field_type_str in ['float32', 'float64', 'int8', 'int16', 'int32', 'int64',
                            'uint8', 'uint16', 'uint32', 'uint64', 'bool', 'string',
                            'boolean', 'int32']:  # Added 'boolean' and explicit 'int32'
            return True
        
        # Check known ROS message types that should be leaf
        if field_type_str in ['sensor_msgs/Image', 'sensor_msgs/PointCloud2', 
                            'sensor_msgs/LaserScan',
                            'sensor_msgs/msg/Image', 'sensor_msgs/msg/PointCloud2',
                            'sensor_msgs/msg/LaserScan',]:
            return True
        
        if field_type_str in ['geometry_msgs/Point', 'geometry_msgs/Vector3', 
                            'geometry_msgs/Quaternion', 'geometry_msgs/Pose',
                            'geometry_msgs/Transform', 'geometry_msgs/PoseStamped', 'sensor_msgs/JointState',
                            'geometry_msgs/msg/Point', 'geometry_msgs/msg/Vector3',
                            'geometry_msgs/msg/Quaternion', 'geometry_msgs/msg/Pose',
                            'geometry_msgs/msg/Transform', 'geometry_msgs/msg/PoseStamped', 'sensor_msgs/msg/JointState']:
            return True
        
        if field_type_str in ['nav_msgs/OccupancyGrid', 'nav_msgs/msg/OccupancyGrid']:
            return True
        
        if field_type_str in ['builtin_interfaces/Time', 'builtin_interfaces/Duration',
                            'builtin_interfaces/msg/Time', 'builtin_interfaces/msg/Duration']:
            return True
        
        # If we still don't know, fallback to value-based check
        # BUT: if field_value is an empty list, we should trust the type definition
        if isinstance(field_value, (list, tuple)) and not field_value:
            # Empty array - can't determine from value, but if we got here, 
            # it's probably not a recognized type, so return False
            self.get_logger().debug(f"Empty array with unrecognized type: {field_type_str}")
            return False
        
        # Fallback to value-based check for runtime instances
        return self._is_leaf_field(field_value)
    
    def _is_leaf_field(self, field_value):
        """Determine if a field should be treated as a leaf feature based on its actual type"""
        
        # Handle None values
        if field_value is None:
            return False
            
        # Basic Python types
        if isinstance(field_value, (int, float, bool, str, bytes)):
            return True
            
        # NumPy arrays
        if isinstance(field_value, np.ndarray):
            return True

        # Sensor message types
        if isinstance(field_value, (Image, PointCloud2, LaserScan, JointState)):
            return True
            
        # Geometry message types
        if isinstance(field_value, (Point, Vector3, Quaternion, Pose, Transform)):
            return True
            
        # Navigation message types
        if isinstance(field_value, OccupancyGrid):
            return True
            
        # Time/Duration types
        if isinstance(field_value, (Time, Duration)):
            return True
            
        # Lists/tuples of basic types
        if isinstance(field_value, (list, tuple)):
            if not field_value:  # empty list/tuple
                return True
            # Check if all elements are basic types
            if all(isinstance(v, (int, float, bool, str, bytes)) for v in field_value):
                return True
            # Check if all elements are geometry types
            if all(isinstance(v, (Point, Vector3, Quaternion, Pose, Transform)) for v in field_value):
                return True
            return False
        
            
        # Default: not a leaf field
        return False

    # def _create_topic_callback(self, demo_id, topic_id, msg_type_class, metadata):
    #     """Create a dynamic callback for a specific topic with enhanced debugging"""
    #     def topic_callback(msg):
    #         try:
    #             if demo_id not in self.active_collections:
    #                 self.get_logger().debug(f"Demo {demo_id} not in active collections, skipping message")
    #                 return
                
    #             # Get trajectory for this demo
    #             trajectory = self.trajectories.get(demo_id)
    #             if trajectory is None:
    #                 self.get_logger().error(f"No trajectory found for demo {demo_id}")
    #                 return

    #             self.get_logger().debug(f"Processing message for topic {topic_id}")
                
    #             # Extract and add data for each registered feature
    #             lock = self.trajectory_locks.get(demo_id)
    #             if lock is None:
    #                 return
                
    #             with lock:
    #                 self._process_message_fields(demo_id, topic_id, msg, trajectory, prefix=topic_id)
                
    #             self.get_logger().debug(f"Successfully processed message for topic {topic_id}")
                    
    #         except Exception as e:
    #             # Enhanced error logging with full traceback
    #             error_info = {
    #                 'demo_id': demo_id,
    #                 'topic_id': topic_id,
    #                 'msg_type': str(type(msg)),
    #                 'error': str(e),
    #                 'traceback': traceback.format_exc()
    #             }
    #             self.get_logger().error(f"Error in callback for {demo_id}/{topic_id}: {error_info}")
                
    #     return topic_callback
    def _create_topic_callback(self, demo_id, topic_id, msg_type_class, metadata):
        def topic_callback(msg):
            try:
                if demo_id not in self.active_collections:
                    return

                # Timestamp (ns)
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    ts = msg.header.stamp
                    timestamp_ns = ts.sec * 1_000_000_000 + ts.nanosec
                else:
                    now = self.get_clock().now()
                    sec, nsec = now.seconds_nanoseconds()
                    timestamp_ns = sec * 1_000_000_000 + nsec

                registered = self.registered_features.get(demo_id, set())
                buffer = self.data_buffer.get(demo_id)
                lock = self.buffer_locks.get(demo_id)

                if buffer is None or lock is None:
                    return

                with lock:
                    for feature_name in registered:
                        if not feature_name.startswith(topic_id):
                            continue

                        field_value = self._extract_field_value(
                            feature_name, topic_id, msg
                        )

                        if self._is_empty_field(field_value):
                            continue

                        numpy_value = self._convert_field_to_numpy(
                            feature_name, field_value
                        )

                        if self._is_empty_field(numpy_value):
                            continue

                        buffer[feature_name].append(
                            (timestamp_ns, numpy_value)
                        )

            except Exception:
                self.get_logger().error(
                    f"Callback error [{demo_id}/{topic_id}]\n{traceback.format_exc()}"
                )

        return topic_callback

    def _extract_field_value(self, feature_name, topic_prefix, msg):
        attr_parts = feature_name.split("/")

        prefix_parts = topic_prefix.split("/")
        if attr_parts[:len(prefix_parts)] == prefix_parts:
            attr_parts = attr_parts[len(prefix_parts):]

        value = msg
        for part in attr_parts:
            if not part:
                continue
            if not hasattr(value, part):
                return None
            value = getattr(value, part)

        return value

    def _process_message_fields(self, demo_id, topic_id, msg_obj, trajectory, prefix=""):
        """
        Process pre-registered message fields and add data to trajectory.
        Uses self.registered_features to avoid dynamic introspection.
        """
        try:
            # Get the set of registered features for this demo
            registered = self.registered_features.get(demo_id, set())
            
            # Get timestamp from header if available
            if hasattr(msg_obj, 'header') and hasattr(msg_obj.header, 'stamp'):
                ts = msg_obj.header.stamp
                timestamp_ns = ts.sec * 1_000_000_000 + ts.nanosec
            else:
                # Fallback to current time
                now = self.get_clock().now()
                sec, nsec = now.seconds_nanoseconds()
                timestamp_ns = sec * 1_000_000_000 + nsec

            for feature_name in registered:
                # Skip fields not under this topic/prefix
                if prefix and not feature_name.startswith(prefix):
                    continue

                try:
                    # Extract the nested attribute dynamically by traversing the path
                    attr_parts = feature_name.split("/")
                    
                    # Skip the topic prefix to get to the actual field path
                    if prefix:
                        # Remove prefix parts from the path
                        prefix_parts = prefix.split("/")
                        if attr_parts[:len(prefix_parts)] == prefix_parts:
                            attr_parts = attr_parts[len(prefix_parts):]
                    
                    # Traverse the message structure
                    field_value = msg_obj
                    for part in attr_parts:
                        if not part:  # Skip empty parts
                            continue
                        try:
                            field_value = getattr(field_value, part)
                        except AttributeError:
                            self.get_logger().warn(f"Field {feature_name} not found in message (failed at '{part}')")
                            field_value = None
                            break

                    # ---------- NEW: skip empty raw field values ----------
                    if self._is_empty_field(field_value):
                        self.get_logger().debug(
                            f"Skipping {feature_name} - empty field value"
                        )
                        continue
                    # ------------------------------------------------------

                    numpy_data = self._convert_field_to_numpy(feature_name, field_value)

                    # ---------- NEW: skip empty converted values ----------
                    if self._is_empty_field(numpy_data):
                        self.get_logger().debug(
                            f"Skipping {feature_name} - empty converted data"
                        )
                        continue
                    # -----------------------------------------------------

                    # Convert to numpy and add to trajectory
                    if numpy_data is not None:
                        try:
                            trajectory.add(feature_name, numpy_data, timestamp=timestamp_ns, time_unit='ns')
                        except Exception as e:
                            self.get_logger().error(
                                f"Failed to add {feature_name} to trajectory: {e}"
                            )
                    else:
                        self.get_logger().debug(
                            f"Skipping {feature_name} - conversion returned None"
                        )

                except Exception as e:
                    self.get_logger().error(f"Error processing field {feature_name}: {e}")
                    self.get_logger().debug(f"Traceback: {traceback.format_exc()}")

        except Exception as e:
            self.get_logger().error(f"Critical error in _process_message_fields for {prefix}: {str(e)}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            raise

    # def _is_empty_field(self, value) -> bool:
    #     """Return True if value should be considered empty and skipped."""
    #     if value is None:
    #         return True

    #     # Empty string
    #     if isinstance(value, str) and value == "":
    #         return True

    #     # Empty containers
    #     if isinstance(value, (list, tuple, dict)) and len(value) == 0:
    #         return True

    #     # Empty numpy array
    #     try:
    #         import numpy as np
    #         if isinstance(value, np.ndarray) and value.size == 0:
    #             return True
    #     except Exception:
    #         pass

    #     return False

    def _is_empty_field(self, value) -> bool:
        """Return True if value should be considered empty and skipped."""
        
        # None values
        if value is None:
            return True
        
        # Empty strings
        if isinstance(value, str) and not value:
            return True
        
        # Empty lists/tuples
        if isinstance(value, (list, tuple)) and len(value) == 0:
            return True
        
        # Empty array.array (ROS2)
        if hasattr(value, '__class__') and value.__class__.__name__ == 'array':
            import array
            if isinstance(value, array.array): # and len(value) == 0:
                return False #True
        
        # Empty numpy arrays
        if isinstance(value, np.ndarray): # and value.size == 0:
            return False #True
        
        # Check ROS2 JointState - empty if no joint names or positions
        if hasattr(value, '__class__') and value.__class__.__name__ == 'JointState':
            # if not hasattr(value, 'name') or len(value.name) == 0:
            #     return True
            if not hasattr(value, 'position') or len(value.position) == 0:
                return True
        
        # Check geometry_msgs types - consider valid if they exist (even with zero values)
        geometry_types = ['Pose', 'Point', 'Vector3', 'Quaternion', 'Transform', 'PoseStamped']
        if hasattr(value, '__class__') and value.__class__.__name__ in geometry_types:
            return False  # Geometry messages with zero values are still valid
        
        # Check Image messages - empty if no data
        if isinstance(value, Image):
            if not hasattr(value, 'data') or len(value.data) == 0:
                return True
            if value.width == 0 or value.height == 0:
                return True
        
        # Check PointCloud2 - empty if no data
        if isinstance(value, PointCloud2):
            if not hasattr(value, 'data') or len(value.data) == 0:
                return True
        
        # Check LaserScan - empty if no ranges
        if isinstance(value, LaserScan):
            if not hasattr(value, 'ranges') or len(value.ranges) == 0:
                return True
        
        # Check OccupancyGrid - empty if no data
        if isinstance(value, OccupancyGrid):
            if not hasattr(value, 'data') or len(value.data) == 0:
                return True
        
        # All other values are considered non-empty
        return False


    def _convert_field_to_numpy(self, field_name, field_value):
        """Convert ROS field to numpy array or appropriate format for robodm trajectory"""
        
        try:
            # Handle None
            if field_value is None:
                self.get_logger().debug(f"Field {field_name} is None")
                return None
            
            # Handle basic Python types - convert to numpy
            if isinstance(field_value, (int, float)):
                return np.array([field_value], dtype=np.float32)
            
            if isinstance(field_value, bool):
                return np.array([field_value], dtype=np.bool_)
            
            if isinstance(field_value, str):
                return field_value  # Keep as string for robodm
            
            if isinstance(field_value, bytes):
                return np.frombuffer(field_value, dtype=np.uint8)
            
            # Handle Time and Duration
            if isinstance(field_value, (Time, Duration)):
                return np.array([field_value.sec, field_value.nanosec], dtype=np.int64)
            
            # Handle JointState
            if hasattr(field_value, '__class__') and field_value.__class__.__name__ == 'JointState':
                # Only extract position data, drop velocity and effort
                position_array = np.array(field_value.position, dtype=np.float64) if hasattr(field_value, 'position') and field_value.position else np.array([], dtype=np.float64)
                
                # Return only positions as a simple numpy array
                return position_array
            
            # Handle Pose
            if isinstance(field_value, Pose):
                # Convert to 7-element array: [x, y, z, qx, qy, qz, qw]
                return np.array([
                    field_value.position.x,
                    field_value.position.y,
                    field_value.position.z,
                    field_value.orientation.x,
                    field_value.orientation.y,
                    field_value.orientation.z,
                    field_value.orientation.w
                ], dtype=np.float64)
            
            # Handle PoseStamped
            if isinstance(field_value, PoseStamped):
                # Extract just the pose part
                return np.array([
                    field_value.pose.position.x,
                    field_value.pose.position.y,
                    field_value.pose.position.z,
                    field_value.pose.orientation.x,
                    field_value.pose.orientation.y,
                    field_value.pose.orientation.z,
                    field_value.pose.orientation.w
                ], dtype=np.float64)
            
            # Handle Point
            if isinstance(field_value, Point):
                return np.array([field_value.x, field_value.y, field_value.z], dtype=np.float64)
            
            # Handle Vector3
            if isinstance(field_value, Vector3):
                return np.array([field_value.x, field_value.y, field_value.z], dtype=np.float64)
            
            # Handle Quaternion
            if isinstance(field_value, Quaternion):
                return np.array([field_value.x, field_value.y, field_value.z, field_value.w], dtype=np.float64)
            
            # Handle Transform
            if isinstance(field_value, Transform):
                # Convert to 7-element array: [tx, ty, tz, qx, qy, qz, qw]
                return np.array([
                    field_value.translation.x,
                    field_value.translation.y,
                    field_value.translation.z,
                    field_value.rotation.x,
                    field_value.rotation.y,
                    field_value.rotation.z,
                    field_value.rotation.w
                ], dtype=np.float64)
            
            # Handle Image
            if isinstance(field_value, Image):
                try:
                    # Use CV bridge to convert to numpy array
                    if hasattr(self, 'cv_bridge'):
                        # Determine encoding
                        if field_value.encoding in ['rgb8', 'bgr8']:
                            cv_image = self.cv_bridge.imgmsg_to_cv2(field_value, desired_encoding='rgb8')
                        elif field_value.encoding in ['mono8', 'mono16']:
                            cv_image = self.cv_bridge.imgmsg_to_cv2(field_value, desired_encoding='passthrough')
                        else:
                            cv_image = self.cv_bridge.imgmsg_to_cv2(field_value, desired_encoding='passthrough')
                        
                        return np.array(cv_image)
                    else:
                        # Fallback: reshape raw data
                        img_array = np.frombuffer(field_value.data, dtype=np.uint8)
                        if field_value.encoding == 'rgb8' or field_value.encoding == 'bgr8':
                            return img_array.reshape((field_value.height, field_value.width, 3))
                        elif field_value.encoding == 'mono8':
                            return img_array.reshape((field_value.height, field_value.width))
                        else:
                            return img_array
                except Exception as e:
                    self.get_logger().error(f"Failed to convert Image: {e}")
                    return None
            
            # Handle PointCloud2
            if isinstance(field_value, PointCloud2):
                # Return the raw data as numpy array
                # For full parsing, you'd need ros2_numpy or similar
                return np.frombuffer(field_value.data, dtype=np.uint8)
            
            # Handle LaserScan
            if isinstance(field_value, LaserScan):
                # Convert to a structured numpy array instead of dict
                ranges = np.array(field_value.ranges, dtype=np.float32)
                intensities = np.array(field_value.intensities, dtype=np.float32) if hasattr(field_value, 'intensities') and field_value.intensities else np.array([], dtype=np.float32)
                
                # Create metadata array: [angle_min, angle_max, angle_increment, range_min, range_max, num_ranges, num_intensities]
                metadata = np.array([
                    field_value.angle_min,
                    field_value.angle_max,
                    field_value.angle_increment,
                    field_value.range_min,
                    field_value.range_max,
                    len(ranges),
                    len(intensities)
                ], dtype=np.float32)
                
                # Combine everything into a single array: [metadata..., ranges..., intensities...]
                combined = np.concatenate([metadata, ranges, intensities])
                return combined
            
            # Handle OccupancyGrid
            if isinstance(field_value, OccupancyGrid):
                # Convert grid data to 2D numpy array
                width = field_value.info.width
                height = field_value.info.height
                grid_array = np.array(field_value.data, dtype=np.int8).reshape((height, width))
                return grid_array
            
            # Handle array.array (ROS2 uses this for float32[], int32[], etc.)
            if hasattr(field_value, '__class__') and field_value.__class__.__name__ == 'array':
                import array
                if isinstance(field_value, array.array):
                    # Convert array.array to numpy array
                    # array.array typecodes: 'f' = float, 'd' = double, 'i' = int, etc.
                    if field_value.typecode == 'f':
                        return np.array(field_value, dtype=np.float32)
                    elif field_value.typecode == 'd':
                        return np.array(field_value, dtype=np.float64)
                    elif field_value.typecode in ['i', 'l']:
                        return np.array(field_value, dtype=np.int32)
                    elif field_value.typecode in ['I', 'L']:
                        return np.array(field_value, dtype=np.uint32)
                    elif field_value.typecode in ['h']:
                        return np.array(field_value, dtype=np.int16)
                    elif field_value.typecode in ['H']:
                        return np.array(field_value, dtype=np.uint16)
                    elif field_value.typecode in ['b']:
                        return np.array(field_value, dtype=np.int8)
                    elif field_value.typecode in ['B']:
                        return np.array(field_value, dtype=np.uint8)
                    else:
                        # Default: try to convert to float32
                        return np.array(field_value, dtype=np.float32)
            
            # Handle lists/tuples of basic types
            if isinstance(field_value, (list, tuple)):
                if not field_value:
                    return None
                
                # Check if all elements are numbers
                if all(isinstance(v, (int, float)) for v in field_value):
                    return np.array(field_value, dtype=np.float32)
                
                # Check if all elements are Pose
                if all(isinstance(v, Pose) for v in field_value):
                    # Convert each pose to 7-element array and stack
                    pose_arrays = []
                    for pose in field_value:
                        pose_arrays.append([
                            pose.position.x, pose.position.y, pose.position.z,
                            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                        ])
                    return np.array(pose_arrays, dtype=np.float64)
                
                # Check if all elements are Point
                if all(isinstance(v, Point) for v in field_value):
                    point_arrays = [[p.x, p.y, p.z] for p in field_value]
                    return np.array(point_arrays, dtype=np.float64)
                
                # Check if all elements are Vector3
                if all(isinstance(v, Vector3) for v in field_value):
                    vec_arrays = [[v.x, v.y, v.z] for v in field_value]
                    return np.array(vec_arrays, dtype=np.float64)
                
                # Check if all elements are Quaternion
                if all(isinstance(v, Quaternion) for v in field_value):
                    quat_arrays = [[q.x, q.y, q.z, q.w] for q in field_value]
                    return np.array(quat_arrays, dtype=np.float64)
                
                # Check if all elements are strings
                if all(isinstance(v, str) for v in field_value):
                    return list(field_value)  # Keep as list of strings
                
                # For mixed or unknown types, return as-is for pickling
                return list(field_value)
            
            # Handle numpy arrays - return as-is
            if isinstance(field_value, np.ndarray):
                return field_value
            
            # For any other type, try to return as-is (robodm will pickle it)
            self.get_logger().debug(f"Returning field {field_name} as-is (type: {type(field_value)})")
            return field_value
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert field {field_name}: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return None

    def _save_buffer_to_hdf5(self, demo_id):
        buffer = self.data_buffer.get(demo_id)
        lock = self.buffer_locks.get(demo_id)

        if buffer is None or lock is None:
            return

        # Save alongside trajectory
        demo_root = self.demo_config[demo_id].get('demo_root_path', self.demo_root_path)
        h5_path = os.path.join(demo_root,demo_id, f"{demo_id}_buffer.h5")

        self.get_logger().info(f"Saving buffered data to HDF5: {h5_path}")

        with lock, h5py.File(h5_path, "w") as h5f:
            for feature_name, samples in buffer.items():
                if not samples:
                    continue

                # sort by timestamp
                samples.sort(key=lambda x: x[0])

                timestamps = np.array(
                    [ts for ts, _ in samples], dtype=np.int64
                )

                try:
                    values = np.stack([v for _, v in samples])
                except Exception:
                    # fallback for variable-sized data
                    values = np.array(
                        [v for _, v in samples], dtype=object
                    )

                # split feature name into groups
                group = h5f
                for part in feature_name.split("/"):
                    group = group.require_group(part)

                group.create_dataset(
                    "timestamps",
                    data=timestamps,
                    compression="gzip",
                    compression_opts=4,
                )

                group.create_dataset(
                    "data",
                    data=values,
                    compression="gzip",
                    compression_opts=4,
                )

        self.get_logger().info(f"HDF5 buffer saved for {demo_id}")

    def _flush_buffer_to_trajectory(self, demo_id):
        trajectory = self.trajectories.get(demo_id)
        buffer = self.data_buffer.get(demo_id)
        lock = self.buffer_locks.get(demo_id)

        if trajectory is None or buffer is None or lock is None:
            return

        self.get_logger().info(
            f"Flushing buffered data to trajectory for {demo_id}"
        )

        with lock:
            for feature_name, samples in buffer.items():
                # samples: list[(timestamp_ns, value)]
                samples.sort(key=lambda x: x[0])  # ensure monotonic

                for ts_ns, value in samples:
                    trajectory.add(
                        feature_name,
                        value,
                        timestamp=ts_ns,
                        time_unit="ns"
                    )

    def _stop_topic_collection(self, demo_id):
        """Stop collecting data for a demo and cleanup resources"""
        try:
            
            # Destroy subscribers
            if demo_id in self.collection_subscribers:
                for topic_id, subscriber in self.collection_subscribers[demo_id].items():
                    if subscriber:  # Check if subscriber exists
                        self.destroy_subscription(subscriber)
                        self.get_logger().debug(f"Destroyed subscriber for {topic_id}")
                del self.collection_subscribers[demo_id]

            self._save_buffer_to_hdf5(demo_id)
            self._flush_buffer_to_trajectory(demo_id)
            
            
            # Clean up registered features
            if hasattr(self, 'registered_features') and demo_id in self.registered_features:
                del self.registered_features[demo_id]
            
            # Note: collection_log_files is no longer used since we're using Robo-DM
            if demo_id in self.collection_log_files:
                del self.collection_log_files[demo_id]

            # Close and save trajectory
            if hasattr(self, 'trajectories') and demo_id in self.trajectories:
                trajectory = self.trajectories[demo_id]
                try:
                    if trajectory:
                        trajectory.close()
                        self.get_logger().info(f"Trajectory saved for demo {demo_id}")
                except Exception as e:
                    self.get_logger().error(f"Failed to close trajectory for {demo_id}: {e}")
                
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
