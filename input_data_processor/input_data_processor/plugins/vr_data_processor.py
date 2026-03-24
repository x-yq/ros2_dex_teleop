from core_bases.data_processor_base import DataProcessorBase
from typing import Dict, List, Any, Optional
import rclpy
import json
import numpy as np
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

# Import message types based on the interface definitions
from component_interfaces.msg import (
    RawHandKeypoint,
    ProcessedHandKeypoint
)
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from builtin_interfaces.msg import Time
from copy import copy

class VRHandKeypointProcessor(DataProcessorBase):
    def __init__(self, layer_cn_id: str, ros2_domain_id: Optional[int] = None):
        """
        .. note::
            OCULUS_JOINTS adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
            
        super().__init__(layer_cn_id, ros2_domain_id)
        
        OCULUS_JOINTS = {
            'metacarpals': [2, 6, 9, 12, 15],
            'knuckles': [6, 9, 12, 16],
            'thumb': [2, 3, 4, 5, 19],
            'index': [6, 7, 8, 20],
            'middle': [9, 10, 11, 21],
            'ring': [12, 13, 14, 22],
            'pinky': [15, 16, 17, 18, 23]
        }

        self.knuckle_points = (OCULUS_JOINTS['knuckles'][0], OCULUS_JOINTS['knuckles'][-1])
        # Moving average queue
        self.moving_average_limit = 5
        # Create a queue for moving average
        self.coord_moving_average_queue, self.frame_moving_average_queue = [], []

        self._load_my_config()
        if self.simulate_latency > 0:
            self._setup_latency_simulation()


    def _load_my_config(self):
        """Load processor-specific configuration using component_node config methods"""
        try:
            # Load this component's specific configuration
            self.my_config = self.load_my_config()
            if self.my_config and "plugins" in self.my_config:
                plugins = self.my_config["plugins"]
                for plugin in plugins:
                    if plugin.get('id') == 'vr_handkeypoint_processor':
                        plugin_params = plugin.get('params', {})
                        self.moving_average_limit = plugin_params.get("moving_average_limit", self.moving_average_limit)
                        self.get_logger().info(f"Loaded processor config")
                        break
                
        except Exception as e:
            self.handle_error(f"Failed to load processor config: ", e)

    def register_pub_sub(self):
        """Register publishers and subscribers based on data_stream configuration"""
        try:
            # Use the loaded data_stream_config from _load_processor_config()
            if not self.data_stream_config:
                self.handle_error("Data stream configuration not loaded")
                return
                
            # Find topics where this node is a subscriber or publisher
            topics = self.data_stream_config.get("topics", [])
            if not topics:
                self.get_logger().warn("No topics found in data stream configuration")
                return
                
            for topic_config in topics:
                topic_id = topic_config["id"]
                
                # Subscribe to topics where this node is a subscriber
                if self.layer_cn_id in topic_config.get("subscribers", []):
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/RawHandKeypoint":
                        self.compo_subscribers[topic_id] = self.create_subscription(
                            RawHandKeypoint,
                            topic_id,
                            self.raw_data_callback,
                            10
                        )
                        self.get_logger().info(f"Subscribed to raw data: {topic_id}")
                
                # Create publishers for topics where this node is the publisher
                if topic_config.get("publisher") == self.layer_cn_id:
                    msg_type = topic_config["msg_type"]
                    
                    if msg_type == "component_interfaces/msg/ProcessedHandKeypoint":
                        self.compo_publishers[topic_id] = self.create_publisher(
                            ProcessedHandKeypoint,
                            topic_id,
                            10
                        )
                        self.get_logger().info(f"Created publisher for processed data: {topic_id}")
            
            self.get_logger().info("All publishers and subscribers registered successfully")
            
        except Exception as e:
            self.handle_error(f"Failed to register publishers/subscribers: ", e)

    def raw_data_callback(self, msg: RawHandKeypoint):
        """Callback for raw hand keypoint data"""

        self.get_logger().debug(f"Received raw hand keypoint data with {len(msg.keypoints)} fingertips")
        
        self.pong_reply("right_hand_keypoint_raw", msg)

        if msg.mode == 0:
            data_type = 'absolute'
        else:
            data_type = 'relative'
        
        if len(msg.keypoints) != 72:
            self.get_logger().error(
                f"Invalid keypoints length: expected 72 floats, got {len(msg.keypoints)}"
            )
            return
         
        transformed_hand_coords, translated_hand_coord_frame = self.transform_keypoints(np.asanyarray(msg.keypoints).reshape(24, 3))
        
        # Passing the transformed coords into a moving average
        self.averaged_hand_coords = self._moving_average(
            transformed_hand_coords, 
            self.coord_moving_average_queue, 
            self.moving_average_limit
        )

        self.averaged_hand_frame = self._moving_average(
            translated_hand_coord_frame, 
            self.frame_moving_average_queue, 
            self.moving_average_limit
        )

        processed_msg = ProcessedHandKeypoint()
        processed_msg.header = msg.header
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.data_type = data_type
        processed_msg.hand_coords = self.averaged_hand_coords.flatten().tolist()
        processed_msg.hand_frame = np.array(self.averaged_hand_frame).flatten().tolist()

        self.publish_msg("right_hand_keypoint_processed", processed_msg)

    def _normalize_vector(self, vector):
        """
        Return the unit vector of ``vector``.

        Divides the input vector by its L2 norm. Used throughout the hand
        coordinate frame construction to produce orthonormal basis vectors.

        Args:
            vector (np.ndarray): Any non-zero numeric array.

        Returns:
            np.ndarray: Unit vector with the same shape as ``vector``.

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        return vector / np.linalg.norm(vector)

    def _moving_average(self, vector, moving_average_queue, limit):
        """
        Append ``vector`` to a FIFO queue and return the element-wise mean.

        Maintains a sliding window of the last ``limit`` vectors. On each call
        the oldest entry is evicted once the queue exceeds ``limit``, then the
        mean across all queued vectors is computed along axis 0. The queue is
        mutated in-place, allowing the caller to maintain separate queues for
        independent data streams (e.g. joint coordinates vs. hand frame).

        Args:
            vector (np.ndarray): New observation to append (any shape).
            moving_average_queue (list): Mutable list used as the sliding window.
                Pass ``self.coord_moving_average_queue`` or
                ``self.frame_moving_average_queue`` as appropriate.
            limit (int): Maximum number of entries to retain in the window.

        Returns:
            np.ndarray: Element-wise mean of all entries currently in the queue,
                        with the same shape as ``vector``.

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        moving_average_queue.append(vector)

        if len(moving_average_queue) > limit:
            moving_average_queue.pop(0)

        mean_vector = np.mean(moving_average_queue, axis = 0)
        return mean_vector
    
    def _translate_coords(self, hand_coords):
        """
        Translate all joint positions to be relative to the wrist (joint 0).

        Subtracts ``hand_coords[0]`` (the wrist joint) from every joint in the
        array, moving the origin of the coordinate system to the wrist. This
        removes the global position of the hand in the VR tracking volume so
        that downstream processing is position-invariant.

        Args:
            hand_coords (np.ndarray): Shape ``(24, 3)`` array of joint positions
                in world space. Joint 0 must be the wrist.

        Returns:
            np.ndarray: Shape ``(24, 3)`` array of joint positions expressed as
                        offsets from the wrist. The wrist entry itself becomes
                        the zero vector.

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        return copy(hand_coords) - hand_coords[0]

    def _get_coord_frame(self, index_knuckle_coord, pinky_knuckle_coord):
        """
        Construct an orthonormal palm coordinate frame from two knuckle landmarks.

        Uses the index knuckle (joint 6) and pinky knuckle (joint 16) — both
        expressed in wrist-relative coordinates — to derive three orthogonal
        axes that describe the current orientation of the palm:

        - **Z** (palm normal): cross product of the two knuckle vectors,
          pointing out of the back of the hand.
        - **Y** (palm bisector / forward): sum of the two knuckle vectors,
          pointing roughly toward the middle fingers.
        - **X** (lateral): cross product of Y and Z, pointing from pinky
          toward index.

        The three vectors form a right-handed orthonormal basis ``[X, Y, Z]``
        that captures wrist rotation. Solving ``[X, Y, Z] · R = I`` for ``R``
        gives the rotation matrix used in :meth:`transform_keypoints` to map
        joint coordinates into the canonical frame.

        Args:
            index_knuckle_coord (np.ndarray): Shape ``(3,)`` wrist-relative
                position of the index knuckle (joint 6).
            pinky_knuckle_coord (np.ndarray): Shape ``(3,)`` wrist-relative
                position of the pinky knuckle (joint 16).

        Returns:
            list[np.ndarray]: ``[X, Y, Z]`` — three unit vectors forming the
                              palm coordinate frame.

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        palm_normal = self._normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Current Z
        palm_direction = self._normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Current Y
        cross_product = self._normalize_vector(np.cross(palm_direction, palm_normal))              # Current X
        return [cross_product, palm_direction, palm_normal]

    def _get_hand_dir_frame(self, origin_coord, index_knuckle_coord, pinky_knuckle_coord):
        """
        Build a compact hand pose descriptor in the original (non-rotated) space.

        While :meth:`_get_coord_frame` and :meth:`transform_keypoints` produce
        rotation-invariant joint coordinates, this method preserves the global
        wrist orientation so that downstream controllers can determine how to
        orient the robot end-effector. The descriptor follows the Unity
        coordinate convention used by the VR runtime:

        - **origin**: wrist position in world space (not wrist-relative).
        - **X** (lateral):  ``normalise(index_knuckle - pinky_knuckle)``
        - **Y** (palm normal): ``normalise(index_knuckle × pinky_knuckle)``
        - **Z** (forward / pointing): ``normalise(index_knuckle + pinky_knuckle)``

        Args:
            origin_coord (np.ndarray): Shape ``(3,)`` wrist position in world space
                (``hand_coords[0]`` before translation).
            index_knuckle_coord (np.ndarray): Shape ``(3,)`` wrist-relative position
                of the index knuckle (joint 6).
            pinky_knuckle_coord (np.ndarray): Shape ``(3,)`` wrist-relative position
                of the pinky knuckle (joint 16).

        Returns:
            list[np.ndarray]: ``[origin, X, Y, Z]`` — four shape-``(3,)`` vectors
                              encoding the wrist position and palm orientation.
                              Flattened to 12 floats in ``ProcessedHandKeypoint.hand_frame``.

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        palm_normal = self._normalize_vector(np.cross(index_knuckle_coord, pinky_knuckle_coord))   # Unity space - Y
        palm_direction = self._normalize_vector(index_knuckle_coord + pinky_knuckle_coord)         # Unity space - Z
        cross_product = self._normalize_vector(index_knuckle_coord - pinky_knuckle_coord)              # Unity space - X
        
        return [origin_coord, cross_product, palm_normal, palm_direction]

    def transform_keypoints(self, hand_coords):
        """
        Transform raw 24-joint hand coordinates into a canonical, pose-invariant frame.

        Applies a four-step pipeline:

        1. **Translate** all joints to be wrist-relative via :meth:`_translate_coords`.
        2. **Build the palm frame** ``[X, Y, Z]`` from the index and pinky knuckles
           via :meth:`_get_coord_frame`.
        3. **Rotate** all joints into the canonical frame by solving
           ``[X, Y, Z] · R = I`` and applying ``R`` — making the representation
           invariant to wrist rotation.
        4. **Compute the hand direction frame** in the original (pre-rotation)
           wrist-relative space via :meth:`_get_hand_dir_frame`, preserving
           global palm orientation for downstream end-effector control.

        Args:
            hand_coords (np.ndarray): Shape ``(24, 3)`` array of raw joint
                positions in world space. Joint 0 must be the wrist.

        Returns:
            tuple:
                - **transformed_hand_coords** (*np.ndarray*, shape ``(24, 3)``):
                  Joint positions in the canonical palm frame — translation- and
                  rotation-invariant.
                - **hand_dir_frame** (*list[np.ndarray]*):
                  Four shape-``(3,)`` vectors ``[origin, X, Y, Z]`` encoding the
                  wrist pose and palm orientation in world space (see
                  :meth:`_get_hand_dir_frame`).

        .. note::
            Adapted from the Open-Teach teleoperation system:

            | Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
            | *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
            | arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870
            |
            | Source: https://github.com/aadhithya14/Open-Teach
        """
        translated_coords = self._translate_coords(hand_coords)
        original_coord_frame = self._get_coord_frame(
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        # Finding the rotation matrix and rotating the coordinates
        rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T
        transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
        
        hand_dir_frame = self._get_hand_dir_frame(
            hand_coords[0],
            translated_coords[self.knuckle_points[0]], 
            translated_coords[self.knuckle_points[1]]
        )

        return transformed_hand_coords, hand_dir_frame

    def urgent_stop(self):
        """Emergency stop implementation"""
        self.get_logger().warn("URGENT STOP triggered for VRHandKeypointProcessor")

        # self.stop_component()
        self.set_state("disconnected")
        
        self.get_logger().info("VRHandKeypointProcessor emergency stopped successfully")