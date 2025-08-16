#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import json
import yaml
from datetime import datetime
from builtin_interfaces.msg import Time, Duration
from std_msgs.msg import String, Header
from system_interfaces.srv import GetConfig, StartDataCollection, StopDataCollection, Logging
# from core_bases.component_node import ComponentNode

## TODO: dummy topic_writer replaced with Robo-DM API. 

class DataCollector(Node):
    """Node responsible for collecting and recording demonstration data"""
    
    def __init__(self):
        super().__init__("data_collector")
        self.layer_cn_id = 'data_collector'

        # Initialize clients
        self._create_clients()

        # Initialize services
        self._create_services()
        
        # State tracking
        self.active_collections = {}
        self.subscribers = {}
        self.topic_writers = {}
        
        # Default configuration
        self.demo_root_path = "/tmp/demos"
        self.supported_formats = ['rosbag2', 'hdf5', 'vla']
        self.supported_codecs = ['h264', 'h265']
        
        self.get_logger().info('DataCollector node initialized successfully')
        
    def _create_clients(self):
        """Create service clients"""
        self.get_config_client = self.create_client(GetConfig, 'get_config')
        self.logging_client = self.create_client(Logging, 'logging')
        
        # Wait for SystemManager to be available
        self.get_logger().info('Waiting for SystemManager services...')
        self.get_config_client.wait_for_service(timeout_sec=10.0)
        self.logging_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Connected to SystemManager services')
        

    def _create_services(self):
        """Create services provided by this node"""
        self.start_data_collection_srv = self.create_service(
            StartDataCollection, 'start_data_collection', 
            self.start_data_collection_srv_callback)
            
        self.stop_data_collection_srv = self.create_service(
            StopDataCollection, 'stop_data_collection', 
            self.stop_data_collection_srv_callback)
            
    def _log_to_system(self, level, message):
        """Send log message to SystemManager"""
        if self.logging_client.service_is_ready():
            request = Logging.Request()
            request.layer_cn_id = self.layer_cn_id
            request.log_level = level
            request.message = message
            request.timestamp = self.get_clock().now().to_msg()
            
            future = self.logging_client.call_async(request)
            # Note: Not waiting for response to avoid blocking
        
    def start_data_collection_srv_callback(self, request, response):
        """Handle start data collection requests"""
        try:
            demo_id = request.demo_id.strip()
            
            # Validate demo_id
            if not demo_id:
                response.success = False
                response.topic_ids = []
                response.storage_path = ""
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
                
            # Get demo configuration from SystemManager
            demo_config = self._get_demo_config()
            
            # Determine storage path
            demo_root = request.demo_root_path.strip() if request.demo_root_path.strip() else \
                       demo_config.get('demo_root_path', self.demo_root_path)
            storage_path = os.path.join(demo_root, demo_id)
            
            # Create storage directory
            os.makedirs(storage_path, exist_ok=True)
            
            # Determine topics to subscribe to
            topics = self._determine_topics(request, demo_config)
            
            # Validate storage format and codec
            storage_format = request.storage_format or demo_config.get('storage_format', 'vla')
            video_codec = request.video_codec or demo_config.get('video_codec', 'h264')
            
            if storage_format not in self.supported_formats:
                self.get_logger().warn(f"Unsupported storage format {storage_format}, using rosbag2")
                storage_format = 'rosbag2'
                
            if video_codec not in self.supported_codecs:
                self.get_logger().warn(f"Unsupported video codec {video_codec}, using h264")
                video_codec = 'h264'
            
            # Register features with Robo-DM API
            self._register_features(demo_id, topics)
            
            # Subscribe to topics
            self._subscribe_topics(topics, demo_id, storage_path, storage_format)
            
            # Create collection metadata
            collection_info = {
                'demo_id': demo_id,
                'storage_path': storage_path,
                'topics': topics,
                'storage_format': storage_format,
                'video_codec': video_codec,
                'description': request.description,
                'start_time': self.get_clock().now(),
                'message_count': {topic: 0 for topic in topics},
                'total_size': 0
            }
            
            # # Save metadata file
            # metadata_path = os.path.join(storage_path, 'collection_metadata.json')
            # with open(metadata_path, 'w') as f:
            #     json.dump({
            #         'demo_id': demo_id,
            #         'description': request.description,
            #         'topics': topics,
            #         'storage_format': storage_format,
            #         'video_codec': video_codec,
            #         'start_time': collection_info['start_time'].nanoseconds,
            #         'created_by': 'data_collector'
            #     }, f, indent=2)
            
            # Track active collection
            self.active_collections[demo_id] = collection_info
            
            # Prepare response
            response.success = True
            response.topic_ids = topics
            response.storage_path = storage_path
            response.start_time = collection_info['start_time'].to_msg()
            
            # Log successful start
            self._log_to_system('INFO', f"Started data collection for demo {demo_id} on {len(topics)} topics")
            self.get_logger().info(f"Data collection started: {demo_id} -> {storage_path}")
            
        except Exception as e:
            response.success = False
            response.topic_ids = []
            response.storage_path = ""
            response.start_time = Time()
            
            error_msg = f"Failed to start data collection for {demo_id}: {e}"
            self._log_to_system('ERROR', error_msg)
            self.get_logger().error(error_msg)
            
        return response
        
    def stop_data_collection_srv_callback(self, request, response):
        """Handle stop data collection requests"""
        try:
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
            
            # Clean up subscribers for this demo
            self._cleanup_demo_subscriptions(demo_id, collection_info)
            
            # Update metadata with final statistics
            self._finalize_collection_metadata(collection_info, end_time)
            
            # Prepare response
            response.success = True
            response.storage_path = collection_info['storage_path']
            response.message = f"Data collection completed for demo {demo_id}. " \
                              f"Collected {sum(collection_info['message_count'].values())} messages"
            
            # Set duration
            duration_msg = Duration()
            duration_msg.sec = int(duration_ns // 1e9)
            duration_msg.nanosec = int(duration_ns % 1e9)
            response.collection_duration = duration_msg
            
            # Log successful completion
            duration_sec = duration_ns / 1e9
            self._log_to_system('INFO', 
                f"Completed data collection for demo {demo_id}. "
                f"Duration: {duration_sec:.2f}s, "
                f"Messages: {sum(collection_info['message_count'].values())}")
            
            self.get_logger().info(f"Data collection completed: {demo_id} ({duration_sec:.2f}s)")
            
            # Remove from active collections
            del self.active_collections[demo_id]
            
        except Exception as e:
            response.success = False
            response.storage_path = ""
            response.collection_duration = Duration()
            response.message = f"Failed to stop data collection: {e}"
            
            error_msg = f"Failed to stop data collection for {demo_id}: {e}"
            self._log_to_system('ERROR', error_msg)
            self.get_logger().error(error_msg)
            
        return response
        
    def _get_demo_config(self):
        """Get demo configuration from SystemManager"""
        try:
            if self.get_config_client.service_is_ready():
                request = GetConfig.Request()
                request.config_file_name = 'demo'
                request.config_keys = []
                
                future = self.get_config_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    return json.loads(future.result().config_data)
                    
        except Exception as e:
            self.get_logger().warn(f"Failed to get demo config: {e}")
            
        # Return default config
        return {
            'default_root_path': '/tmp/demos',
            'default_storage_format': 'rosbag2',
            'default_video_codec': 'h264',
            'max_duration': 600
        }
        
    def _determine_topics(self, request, demo_config):
        """Determine which topics to record"""
        if request.topic_ids:
            return list(request.topic_ids)
        
        # Get default topics from config or use hardcoded defaults
        default_topics = demo_config.get('topics', [
            '/robot_state'
        ])
        
        return default_topics
        
    def _register_features(self, demo_id, topics):
        ## NOTE: dummy impl
        """Register features with Robo-DM API (dummy implementation)"""
        try:
            # This would integrate with actual Robo-DM API
            # For now, just create a feature registry file
            feature_registry = {
                'demo_id': demo_id,
                'features': [],
                'registration_time': datetime.now().isoformat()
            }
            
            for topic in topics:
                # Get metadata for topic from SystemManager
                metadata = self._get_topic_metadata(topic)
                
                feature_info = {
                    'topic_id': topic,
                    'feature_type': metadata.get('topic_type', 'unknown'),
                    'shape': metadata.get('shape', []),
                    'frequency': metadata.get('frequency', 0.0),
                    'registered': True
                }
                
                feature_registry['features'].append(feature_info)
                
            self.get_logger().info(f"Registered {len(topics)} features for demo {demo_id}")
            
        except Exception as e:
            self.get_logger().warn(f"Feature registration failed for {demo_id}: {e}")
            
    def _get_topic_metadata(self, topic):
        ## NOTE: dummy impl
        """Get metadata for a specific topic from SystemManager"""
        # This would call a service to get topic metadata
        # For now, return dummy metadata
        dummy_metadata = {
            '/hand_keypoints': {
                'topic_type': 'component_interfaces/msg/RawHandKeypoint',
                'shape': [21, 3],
                'frequency': 30.0
            },
            '/robot_state': {
                'topic_type': 'component_interfaces/msg/DexRobotState', 
                'shape': [7],
                'frequency': 100.0
            },
            '/camera/image_raw': {
                'topic_type': 'component_interfaces/msg/Image',
                'shape': [480, 640, 3],
                'frequency': 30.0
            },
            '/joint_commands': {
                'topic_type': 'component_interfaces/msg/JointAngles',
                'shape': [7],
                'frequency': 50.0
            }
        }
        
        return dummy_metadata.get(topic, {'topic_type': 'unknown', 'shape': [], 'frequency': 0.0})
        
    def _subscribe_topics(self, topics, demo_id, storage_path, storage_format):
        """Subscribe to all specified topics for data collection"""
        for topic in topics:
            try:
                # Create topic-specific callback
                callback = self._make_callback(topic, demo_id, storage_path)
                
                # Create QoS profile (using sensor data profile for reliability)
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )
                
                # Create subscriber (using String as dummy message type)
                subscriber = self.create_subscription(
                    String, topic, callback, qos_profile)
                    
                # Store subscriber with unique key
                subscriber_key = f"{demo_id}_{topic}"
                self.subscribers[subscriber_key] = subscriber
                
                # Initialize topic writer for this demo
                writer_key = f"{demo_id}_{topic}"
                self.topic_writers[writer_key] = self._create_topic_writer(
                    topic, storage_path, storage_format)
                    
                self.get_logger().debug(f"Subscribed to {topic} for demo {demo_id}")
                
            except Exception as e:
                self.get_logger().error(f"Failed to subscribe to {topic} for demo {demo_id}: {e}")
                
    def _make_callback(self, topic_id, demo_id, storage_path):
        """Create callback function for a specific topic"""
        def callback(msg):
            self.topic_callback(msg, topic_id, demo_id, storage_path)
        return callback
        
    def topic_callback(self, msg, topic_id, demo_id, storage_path):
        """Handle incoming topic messages during data collection"""
        try:
            if demo_id not in self.active_collections:
                self.get_logger().warn(f"Received message for inactive demo: {demo_id}")
                return
                
            # Update message count
            self.active_collections[demo_id]['message_count'][topic_id] += 1
            
            # Write message to storage
            writer_key = f"{demo_id}_{topic_id}"
            if writer_key in self.topic_writers:
                self.topic_writers[writer_key].write_message(msg)
                
            # Log periodic status (every 100 messages per topic)
            msg_count = self.active_collections[demo_id]['message_count'][topic_id]
            if msg_count % 100 == 0:
                self.get_logger().debug(
                    f"Demo {demo_id}, Topic {topic_id}: {msg_count} messages recorded")
                    
        except Exception as e:
            self.get_logger().error(f"Error processing message for {demo_id}/{topic_id}: {e}")
            
    def _create_topic_writer(self, topic, storage_path, storage_format):
        """Create a writer for a specific topic (dummy implementation)"""
        class DummyTopicWriter:
            def __init__(self, topic, path, format_type):
                self.topic = topic
                self.path = path
                self.format = format_type
                self.message_count = 0
                
                # Create topic-specific file
                safe_topic_name = topic.replace('/', '_')
                self.file_path = os.path.join(path, f"{safe_topic_name}.{format_type}")
                
            def write_message(self, msg):
                # Dummy implementation - in real scenario, this would serialize and write message
                self.message_count += 1
                
                # Write to a simple text log for demonstration
                try:
                    with open(self.file_path, 'a') as f:
                        f.write(f"[{datetime.now().isoformat()}] Message {self.message_count}: {msg.data}\n")
                except Exception as e:
                    pass  # Silent fail for demo
                    
        return DummyTopicWriter(topic, storage_path, storage_format)
        
    def _cleanup_demo_subscriptions(self, demo_id, collection_info):
        """Clean up all subscriptions for a completed demo"""
        for topic in collection_info['topics']:
            # Clean up subscriber
            subscriber_key = f"{demo_id}_{topic}"
            if subscriber_key in self.subscribers:
                self.destroy_subscription(self.subscribers[subscriber_key])
                del self.subscribers[subscriber_key]
                
            # Clean up topic writer
            writer_key = f"{demo_id}_{topic}"
            if writer_key in self.topic_writers:
                # Finalize and close writer
                writer = self.topic_writers[writer_key]
                # In real implementation, call writer.close() or similar
                del self.topic_writers[writer_key]
                
        self.get_logger().debug(f"Cleaned up subscriptions for demo {demo_id}")
        
    def _finalize_collection_metadata(self, collection_info, end_time):
        """Update collection metadata with final statistics"""
        try:
            metadata_path = os.path.join(collection_info['storage_path'], 'collection_metadata.json')
            
            # Calculate final statistics
            duration_ns = (end_time - collection_info['start_time']).nanoseconds
            total_messages = sum(collection_info['message_count'].values())
            
            final_metadata = {
                'demo_id': collection_info['demo_id'],
                'description': collection_info['description'],
                'topics': collection_info['topics'],
                'storage_format': collection_info['storage_format'],
                'video_codec': collection_info['video_codec'],
                'start_time': collection_info['start_time'].nanoseconds,
                'end_time': end_time.nanoseconds,
                'duration_seconds': duration_ns / 1e9,
                'total_messages': total_messages,
                'message_count_per_topic': collection_info['message_count'],
                'created_by': 'data_collector',
                'finalized_at': datetime.now().isoformat()
            }
            
            with open(metadata_path, 'w') as f:
                json.dump(final_metadata, f, indent=2)
                
        except Exception as e:
            self.get_logger().error(f"Failed to finalize metadata: {e}")
            
    def get_active_collections(self):
        """Return information about currently active collections"""
        return {demo_id: {
            'demo_id': info['demo_id'],
            'storage_path': info['storage_path'],
            'topics': info['topics'],
            'start_time': info['start_time'].nanoseconds,
            'message_counts': info['message_count']
        } for demo_id, info in self.active_collections.items()}


def main(args=None):
    """Main entry point for DataCollector node"""
    rclpy.init(args=args)
    
    try:
        node = DataCollector()
        node.get_logger().info("DataCollector node initialized, spinning...")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("DataCollector node interrupted by user")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f"DataCollector node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("DataCollector node shutdown complete")


if __name__ == '__main__':
    main()
