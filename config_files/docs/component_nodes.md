# Pipeline Nodes — Overview

This document summarises all ROS2 nodes in the VR teleoperation pipeline. For full per-node documentation follow the links in the table of contents. For complete topic definitions, message types, and data flow diagrams see **`ros2_interface.md`**.

---

## Table of Contents

- [Input Sensor Driver: VRDetectorDriver Node Documentation](#input-sensor-driver-vrdetectordriver-node-documentation)
- [Input Data Processor: VRHandKeypointProcessor Node Documentation](#input-data-processor-vrhandkeypointprocessor-node-documentation)
- [Motion Retarget: MotionRetargetOT Node Documentation](#motion-retarget-motionretargetot-node-documentation)
- [Robot Driver: SimXArm5Driver Node Documentation](#robot-driver-simxarm5driver-node-documentation)
- [Feedback Data Processor: MultiFeedbackDataProcessor Node Documentation](#feedback-data-processor-multifeedbackdataprocessor-node-documentation)
- [Feedback Projector Driver: VRDriver Node Documentation](#feedback-projector-driver-vrdriver-node-documentation)
---

## Node Summary Table

| # | Node | `layer_cn_id` | Subscribed Topics | Published Topics | Role | Container |
|---|------|--------------|-------------------|------------------|------|-----------|
| 1 | `VRDetectorDriver` | `input_sensor_driver_vr` | — *(ZMQ input only)* | `right_hand_keypoint_raw` | Reads raw ZMQ hand data from VR headset; publishes `RawHandKeypoint` | `teleop-drivers` |
| 2 | `VRHandKeypointProcessor` | `input_control_data_processor` | `right_hand_keypoint_raw` | `right_hand_keypoint_processed` | Transforms and smooths 24-joint hand pose into wrist-centred canonical frame | `teleop-components` |
| 3 | `MotionRetargetOT` | `motion_retarget` | `right_hand_keypoint_processed`, `robot_state`, `gripper_state` | `robot_control_cmd`, `gripper_control_cmd` | Maps hand pose deltas to robot Cartesian commands; handles gripper and calibration gestures | `teleop-components` |
| 4 | `MultiFeedbackDataProcessor` | `feedback_data_processor` | `right_hand_keypoint_processed` | `handkeypoint_plot_2d` | Renders real-time 2D hand skeleton plot as an image message | `teleop-components` |
| 5 | `VRDriver` | `feedback_projector_driver` | `handkeypoint_plot_2d`, *(scene image)* | — *(ZMQ output only)* | Compresses and streams feedback images over ZMQ to the VR headset display | `teleop-drivers` |
| 6 | `SimXArm5Driver` | `robot_driver` | `robot_control_cmd`, `gripper_control_cmd`, `/joint_states` | `robot_state`, `gripper_state` | Bridges `ControlCmd` messages to `ros2_control` Cartesian controller and trajectory action servers in Gazebo | `teleop-drivers` |

> For topic message types, field definitions, and the full data flow diagram, see [**`ros2_interface.md`**](ros2_interface.md).

---

## Config File Format Summary

All nodes follow the same YAML structure. The file **must** be named after the node's `layer_cn_id` and placed at:

```
config_files/component_node/<layer_cn_id>.yaml
```

```yaml
layer_cn_id: <node_name>        # Must match the node's layer_cn_id exactly
pkg_name:    <ros2_package>     # ROS2 package the node belongs to

plugins:
  - id: <plugin_id>             # Must match the id checked in _load_my_config()
    params:
      param_key: value          # Node-specific parameters (see per-node docs)
```

Quick reference of config files per node:

| Node | Config file |
|------|-------------|
| `VRDetectorDriver` | `input_sensor_driver_vr.yaml` |
| `VRHandKeypointProcessor` | `input_control_data_processor.yaml` |
| `MotionRetargetOT` | `motion_retarget.yaml` |
| `MultiFeedbackDataProcessor` | `feedback_data_processor.yaml` |
| `VRDriver` | `feedback_projector_driver.yaml` |
| `SimXArm5Driver` | `robot_driver.yaml` |

---

## Common Initialisation Order

Every node follows the same three-phase startup sequence enforced by `ComponentNode._initialize_node()`. Understanding this order matters when extending a node or diagnosing startup errors:

```
Phase 1 — Base class chain
  ComponentNode.__init__()
    └── _load_root_configs()          # Loads hardware.yaml and data_stream.yaml
                                      # into self.hardware_config / self.data_stream_config

Phase 2 — Plugin configuration
  _load_my_config()                   # Reads config_files/component_node/<layer_cn_id>.yaml
                                      # Overwrites default attribute values from params block

Phase 3 — ROS2 interface registration
  register_pub_sub()                  # Creates publishers, subscribers, action clients
  register_timer()                    # Creates periodic timers (if any)
  check_plugins()                     # Verifies required methods are implemented
```

**Key implications:**
- `self.data_stream_config` is available by the time `register_pub_sub()` runs — topic names and types are never hardcoded, always resolved from this config.
- Nodes with no timer (`VRHandKeypointProcessor`, `MotionRetargetOT`, `MultiFeedbackDataProcessor`, `VRDriver`) leave `register_timer()` as a no-op; all processing is callback-driven.

---
---

# Input Sensor Driver: VRDetectorDriver Node Documentation

## Overview

`VRDetectorDriver` is a ROS2 sensor driver node that captures hand keypoint data from VR headset and publishes it as `RawHandKeypoint` messages. It inherits from `SensorDriverBase` and uses ZeroMQ (ZMQ) PULL sockets to receive raw streaming data from a remote execution platform over a local network.

The node is designed for use in teleoperation pipelines where hand pose data from a VR device must be streamed at high frequency into a ROS2 ecosystem.


## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── SensorDriverBase (ABC)
              └── VRDetectorDriver
```


## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` of the node and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/input_sensor_driver_vr.yaml
```

### Config File Format

```yaml
# config_files/component_node/input_sensor_driver_vr.yaml

layer_cn_id: input_sensor_driver_vr
pkg_name: input_sensor_driver

plugins:
  - id: vr_detector_driver
    params:
      freq: 90                          # Streaming frequency in Hz (default: 120)
      execution_platform_ip: 192.168.178.92   # IP of the machine that connecting with the VR headset
      left_hand_port: 8110              # ZMQ port for left hand keypoints (reserved, see note)
      right_hand_port: 8087             # ZMQ port for right hand keypoints
      button_port: 8095                 # ZMQ port for button/reset feedback
```

### Parameter Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `freq` | `int` | `120` | Timer frequency in Hz for the `stream()` callback |
| `execution_platform_ip` | `str` | `192.168.178.54` | IP address of the VR execution platform |
| `left_hand_port` | `int` | _(unused currently)_ | ZMQ port for left hand data (reserved for future use) |
| `right_hand_port` | `int` | `8087` | ZMQ port for right hand raw keypoints |
| `button_port` | `int` | `8095` | ZMQ port for button and reset feedback |

> **Note:** The `reset_port` (default `8100`) is currently hardcoded in the driver and not exposed via the config file.

---

## What the Node Does

`VRDetectorDriver` performs the following at runtime:

1. **Loads configuration** from the YAML file (via `_load_my_config`) to set up IP/port addresses and frequency.
2. **Registers ZMQ PULL sockets** (`_register_zmq_sub`) bound to the local machine on the configured ports, receiving data pushed from the VR execution platform.
3. **Registers ROS2 publishers** (`register_pub_sub`) for topics defined in the data stream configuration.
4. **Registers a periodic ROS2 timer** (`register_timer`) that fires at `freq` Hz and calls `stream()`.
5. **Streams data** (`stream`): on each timer tick, reads raw bytes from ZMQ sockets, parses keypoint vectors, builds a `RawHandKeypoint` message, and publishes it.
6. **Handles lifecycle**: exposes `start_component`, `urgent_stop`, `start_sensor`, and `stop_sensor` for integration with the broader component lifecycle framework.

## Relevant Features and Functions

### Initialization

```python
VRDetectorDriver(layer_cn_id: str, ros2_domain_id: Optional[int] = None)
```

Called by the component framework. Chains through `SensorDriverBase.__init__` → `ComponentNode.__init__`, then:
- Sets default attribute values (ports, frequency, connection status flags)
- Calls `_load_my_config()` to overwrite defaults from YAML
- Calls `_register_zmq_sub()` to bind ZMQ sockets
- Calls `start_sensor('vr_detector')` to mark the sensor as active


### `register_pub_sub()`

Iterates through the data stream configuration's `topics` list. For each entry with `msg_type == "component_interfaces/msg/RawHandKeypoint"`, creates a ROS2 publisher and stores it in `self.compo_publishers[topic_name]`.


### `_register_zmq_sub()`

Binds three ZMQ PULL sockets (with `CONFLATE=1`, keeping only the latest message):

| Socket | Port | Purpose |
|--------|------|---------|
| `reset_socket` | `reset_port` (8100) | Receives reset/stop signals (`b'Low'` = stop) |
| `raw_keypoint_right_socket` | `right_hand_port` | Receives right-hand keypoint stream |
| `button_keypoint_socket` | `button_port` | Receives button feedback |


### `_extract_data_from_token(token: bytes) -> dict`

Parses a raw byte token from the ZMQ socket into a structured dictionary.

**Token format:**

```
<mode>:<x1>,<y1>,<z1>|<x2>,<y2>,<z2>|...
```

- Tokens starting with `absolute` → mode `0`
- All other tokens → mode `1` (relative)
- Returns: `{ "keypoints": [mode, x1, y1, z1, x2, y2, z2, ...] }`


### `_create_raw_hand_keypoint_msg(hand_data: dict) -> RawHandKeypoint`

Assembles a `RawHandKeypoint` ROS2 message:
- `header.stamp`: current ROS2 clock time
- `header.frame_id`: `"VR"`
- `mode`: first element of `keypoints` list (0 = absolute, 1 = relative)
- `keypoints`: remaining float values (flattened XYZ per joint)


### `stream()`

Main data loop, invoked by the ROS2 timer at `freq` Hz. On each call:
1. Returns immediately if `sensor_active` is `False`
2. Calls `.recv()` on all three ZMQ sockets (blocking)
3. Parses the right-hand keypoint token and publishes a `RawHandKeypoint` message to `right_hand_keypoint_raw`
4. Interprets `reset_info`: `b'Low'` → `0` (stop signal), anything else → `1` (continue)


### `register_timer()`

Destroys any existing timers, then creates a new ROS2 timer with period `1.0 / freq` seconds, bound to `stream()`.

---

## Data I/O

### Published Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `right_hand_keypoint_raw` | `component_interfaces/msg/RawHandKeypoint` | Raw right-hand keypoint data from VR glove |

The topic name and message type must be defined in the data stream configuration (separate YAML). The publisher is registered dynamically during `register_pub_sub()`.

### Subscribed Topics

None. The node receives all external data via ZMQ sockets (not ROS2 subscriptions).

### ZMQ Sockets (Input)

| Socket | Direction | Transport | Description |
|--------|-----------|-----------|-------------|
| `reset_socket` | PULL (bind) | TCP | Reset/stop signal from VR platform |
| `raw_keypoint_right_socket` | PULL (bind) | TCP | Right-hand keypoint stream |
| `button_keypoint_socket` | PULL (bind) | TCP | Button feedback |

All sockets use `CONFLATE=1` to always process the most recent message and discard stale data.

---

## Acknowledgements

The ZMQ-based streaming design and hand keypoint data protocol used in this driver (`_register_zmq_sub`, `_extract_data_from_token`, `stream`) are adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach

---

---

# Input Data Processor: VRHandKeypointProcessor Node Documentation

## Overview

`VRHandKeypointProcessor` is a ROS2 data processor node that consumes raw hand keypoint data from a VR glove driver, transforms it into a stable, wrist-centred coordinate frame, applies a moving average filter, and publishes the result as a `ProcessedHandKeypoint` message. It inherits from `DataProcessorBase` and is driven entirely by ROS2 subscription callbacks — it has no internal timer.


## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── DataProcessorBase (ABC)
              └── VRHandKeypointProcessor
```


## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/input_control_data_processor.yaml
```

### Config File Format

```yaml
# config_files/component_node/input_control_data_processor.yaml

layer_cn_id: input_control_data_processor
pkg_name: input_data_processor

plugins:
  - id: vr_handkeypoint_processor
    params:
      moving_average_limit: 5           # FIFO queue length, used for smoothing the poses across n frames.
```

### Parameter Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `moving_average_limit` | `int` | — | FIFO queue length, used for smoothing the poses across n frames. |

> **Note:** Unlike `VRDetectorDriver`, this node does not register a periodic timer. All processing is triggered by incoming ROS2 messages.

---

## What the Node Does

`VRHandKeypointProcessor` sits between the raw sensor driver and downstream motion controllers. On every incoming `RawHandKeypoint` message it:

1. **Validates** the keypoint payload — exactly 72 floats (24 joints × 3 axes) are required.
2. **Transforms** the raw 24-joint hand pose into a normalised, wrist-centred coordinate frame via `transform_keypoints`.
3. **Smooths** both the joint coordinates and the hand frame descriptor using independent moving-average queues.
4. **Publishes** the result as a `ProcessedHandKeypoint` message.


## Features and Functions

### Initialisation

```python
VRHandKeypointProcessor(layer_cn_id: str, ros2_domain_id: Optional[int] = None)
```

Chains through `DataProcessorBase.__init__`, then:

- Defines the Oculus joint index map (`OCULUS_JOINTS`) to name anatomical groups.
- Extracts `knuckle_points` — the index and pinky knuckle joint indices used as the hand-frame reference axes.
- Initialises two independent moving-average queues (`coord_moving_average_queue`, `frame_moving_average_queue`) with a window of 5 frames.
- Calls `_load_my_config()` to read YAML parameters.
- Optionally calls `_setup_latency_simulation()` if `simulate_latency > 0`.

### Oculus Joint Index Map

The processor uses a fixed joint index convention matching the Oculus hand tracking SDK:

| Group | Joint Indices |
|-------|--------------|
| `metacarpals` | 2, 6, 9, 12, 15 |
| `knuckles` | 6, 9, 12, 16 |
| `thumb` | 2, 3, 4, 5, 19 |
| `index` | 6, 7, 8, 20 |
| `middle` | 9, 10, 11, 21 |
| `ring` | 12, 13, 14, 22 |
| `pinky` | 15, 16, 17, 18, 23 |

The **knuckle reference pair** is joints `6` (index knuckle) and `16` (pinky knuckle). These two points define the plane of the palm and are used to construct the hand coordinate frame.


### `_load_my_config()`

Reads the YAML plugin block for `vr_handkeypoint_processor` and overwrites `simulate_latency` from `params`. All other parameters in the YAML are currently informational.


### `register_pub_sub()`

Iterates over `data_stream_config["topics"]` and registers:

- A **subscription** for every topic where `self.layer_cn_id` appears in the `subscribers` list and `msg_type` is `component_interfaces/msg/RawHandKeypoint`. The callback is `raw_data_callback`.
- A **publisher** for every topic where `publisher == self.layer_cn_id` and `msg_type` is `component_interfaces/msg/ProcessedHandKeypoint`.


### `raw_data_callback(msg: RawHandKeypoint)`

The central processing callback, triggered on every received message:

1. Calls `pong_reply` on `right_hand_keypoint_raw` (latency/heartbeat bookkeeping).
2. Maps `msg.mode` → `data_type`: `0` = `'absolute'`, `1` = `'relative'`.
3. **Validates** `len(msg.keypoints) == 72`; logs an error and returns early if not.
4. Reshapes the flat 72-float array to `(24, 3)` and calls `transform_keypoints`.
5. Applies `_moving_average` separately to the 24×3 joint array and the 4×3 hand frame descriptor.
6. Assembles and publishes a `ProcessedHandKeypoint` message.

---

## Data I/O

### Subscribed Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `right_hand_keypoint_raw` | `component_interfaces/msg/RawHandKeypoint` | Raw 24-joint hand keypoints from the VR driver |

### Published Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `right_hand_keypoint_processed` | `component_interfaces/msg/ProcessedHandKeypoint` | Transformed, smoothed hand keypoints and frame descriptor |

Both topic names and their `msg_type` values are resolved dynamically from the shared data stream configuration YAML at runtime — they are not hardcoded in this node except for the callback registration logic.

### `ProcessedHandKeypoint` Message Fields

| Field | Type | Content |
|-------|------|---------|
| `header` | `std_msgs/Header` | Copied from incoming message, stamp updated to now |
| `data_type` | `string` | `'absolute'` or `'relative'` |
| `hand_coords` | `float64[]` | Flattened 24×3 transformed and smoothed joint coordinates (72 floats) |
| `hand_frame` | `float64[]` | Flattened 4×3 hand frame descriptor (12 floats): `[origin, x_axis, y_axis, z_axis]` |

---

## Hand Keypoint Transformation

This is the core processing pipeline. The goal is to produce a **pose-invariant, wrist-centred** representation of the hand that is stable enough to drive a robot regardless of where the operator's hand is in space.

The pipeline consists of four sequential steps:

### Step 1 — Wrist-Relative Translation (`_translate_coords`)

```python
translated_coords = hand_coords - hand_coords[0]
```

Joint `0` is the wrist. Subtracting it from all joints moves the origin to the wrist, eliminating the global position of the hand in the VR tracking volume. After this step all joint positions are expressed as **offsets from the wrist**.

### Step 2 — Palm Coordinate Frame Construction (`_get_coord_frame`)

Two anatomical landmarks define the palm plane: the **index knuckle** (joint 6) and the **pinky knuckle** (joint 16), both already in wrist-relative coordinates.

```
Z  =  normalise( index_knuckle × pinky_knuckle )   ← palm normal
Y  =  normalise( index_knuckle + pinky_knuckle )    ← palm bisector / forward
X  =  normalise( Y × Z )                            ← palm lateral
```

The three vectors `[X, Y, Z]` form a right-handed orthonormal basis that captures the **orientation of the palm**. It changes as the operator rotates their wrist.

### Step 3 — Rotation into Canonical Frame (`transform_keypoints`)

The rotation that takes the *current* palm frame `[X, Y, Z]` to the *identity* frame `I` is found by solving:

```
[X, Y, Z] · R  =  I   →   R  =  solve([X, Y, Z], I).T
```

Applying `R` to all 24 joint vectors:

```python
transformed_hand_coords = (rotation_matrix @ translated_coords.T).T
```

produces coordinates expressed in a **fixed canonical frame** aligned with the palm geometry. The representation is now **invariant to wrist rotation** — a clenched fist looks the same regardless of whether the operator's palm faces up or down.

### Step 4 — Hand Direction Frame (`_get_hand_dir_frame`)

A separate 4-vector descriptor is computed in the *original* (non-rotated) wrist-relative space to preserve the global orientation information that Step 3 discards. This is needed downstream so the robot knows how to orient its end-effector.

```
origin   =  hand_coords[0]          ← wrist position in world space
X        =  normalise( index_knuckle - pinky_knuckle )   ← lateral axis
Y        =  normalise( index_knuckle × pinky_knuckle )   ← palm normal
Z        =  normalise( index_knuckle + pinky_knuckle )   ← forward / pointing direction
```

The result `[origin, X, Y, Z]` is a compact wrist pose descriptor in world that encodes where the wrist is and which way the palm faces, without encoding individual finger poses.

### Step 5 — Moving Average Smoothing (`_moving_average`)

Both the 24×3 joint array and the 4×3 frame descriptor pass through **independent FIFO queues** of length 5. On each callback:

```python
queue.append(new_value)
if len(queue) > limit:
    queue.pop(0)
mean = np.mean(queue, axis=0)
```

The element-wise mean across the queue suppresses high-frequency jitter from sensor noise while keeping latency low (≤5 frames at 90 Hz ≈ 56 ms). The two queues are independent so transient noise in joint positions does not corrupt the frame descriptor and vice versa.

### Design Summary

```
Raw 24×3 keypoints (world space)
         │
         ▼
  Translate to wrist origin
         │
         ▼
  Build palm frame [X, Y, Z]
         │
         ├──────────────────────────────────────────────────────┐
         ▼                                                      ▼
  Rotate into canonical frame                    Build hand direction frame
  (rotation-invariant joint coords)              (wrist pose + palm orientation)
         │                                                      │
         ▼                                                      ▼
  Moving average (window=5)                      Moving average (window=5)
         │                                                      │
         └────────────────────┬─────────────────────────────────┘
                              ▼
                  ProcessedHandKeypoint message
                  hand_coords (72 floats)
                  hand_frame  (12 floats)
```

---

## Acknowledgements

The keypoint transformation design used in this processor (`_normalize_vector`, `_moving_average`, `_translate_coords`, `_get_coord_frame`, `_get_hand_dir_frame`, `transform_keypoints`) is adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach

---
---
# Motion Retarget: MotionRetargetOT Node Documentation

## Overview

`MotionRetargetOT` is a ROS2 motion retargeting node that maps processed VR hand pose data onto robot Cartesian and joint commands. It subscribes to smoothed hand keypoints from `VRHandKeypointProcessor`, translates wrist-relative position deltas and palm orientation changes into end-effector target poses, detects finger-pinch gestures for gripper control, and supports in-operation recalibration via a dedicated hand gesture. All processing is callback-driven — the node has no internal timer.


## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── MotionRetargetBase (ABC)
              └── MotionRetargetOT
```


## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/motion_retarget.yaml
```

### Config File Format

```yaml
# config_files/component_node/motion_retarget.yaml

layer_cn_id: motion_retarget
pkg_name: motion_retarget

plugins:
  - id: motion_retarget_ot
    params:
      position_scale: 1.0                    # Linear scale on position deltas
      rotation_scale: 1.0                    # Linear scale on rotation deltas
      gripper_distance_threshold: 0.05       # Thumb-pinky distance (m) to trigger pinch
      gripper_debounce_frames: 3             # Frames pinch must hold before toggling gripper
      calibration_distance_threshold: 0.04   # Thumb-middle distance (m) to trigger calibration gesture
      calibration_debounce_frames: 5         # Frames gesture must hold before triggering
      wrist_rotation_threshold: 1.57         # Accumulated wrist rotation (rad) to fire a base joint command
      wrist_rotation_scale: 0.5             # Scale factor applied to accumulated wrist rotation
```

### Parameter Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `position_scale` | `float` | `1.0` | Multiplier applied to all Cartesian position deltas before sending to the robot |
| `rotation_scale` | `float` | `1.0` | Multiplier applied to end-effector rotation delta (rotation vector magnitude) |
| `gripper_distance_threshold` | `float` | `0.05` | Euclidean distance (m) between thumb tip and pinky tip below which a pinch is detected |
| `gripper_debounce_frames` | `int` | `3` | Number of consecutive frames the pinch must be held to toggle the gripper |
| `calibration_distance_threshold` | `float` | `0.04` | Distance (m) between thumb tip and middle finger tip to detect the calibration gesture |
| `calibration_debounce_frames` | `int` | `5` | Frames the calibration gesture must be held to trigger a state change |
| `wrist_rotation_threshold` | `float` | `1.57` | Accumulated wrist rotation in radians (~90°) before a base joint command is emitted |
| `wrist_rotation_scale` | `float` | `0.5` | Scale applied to accumulated wrist rotation when computing the joint angle delta |
| `simulate_latency` | `int` | `0` | Artificial latency in ms; `0` or omitted disables simulation |

> **Note:** `workspace_offset` (an XYZ array added to every target position) is supported in code but not currently exposed in the YAML — it defaults to `[0.0, 0.0, 0.0]`.

---

## What the Node Does

`MotionRetargetOT` bridges the human operator's hand pose to robot motion in four parallel functions on every received message:

1. **Calibration gesture detection** — monitors thumb-middle finger proximity; a confirmed touch toggles between *pause-and-reset* and *resume-and-recalibrate* states.
2. **Auto-initialisation** — on the first message after calibration is ready, records the current hand origin and robot pose as the reference frame.
3. **End-effector command generation** — maps hand position deltas and palm frame rotation changes to absolute Cartesian target poses for the robot arm.
4. **Gripper command generation** — monitors thumb-pinky proximity and toggles the gripper between open and closed on each confirmed pinch.


## Features and Functions

### Initialisation

```python
MotionRetargetOT(layer_cn_id: str, ros2_domain_id: Optional[int] = None)
```

Sets up all state variables and then calls `_load_my_config()`:

- `position_scale`, `rotation_scale`, `workspace_offset` — motion scaling
- `hand_origin_offset`, `robot_origin`, `last_hand_frame` — calibration anchors
- `gripper_flag`, `gripper_cnt`, `gripper_debounce_frames` — gripper debounce state
- `calibration_gesture_cnt`, `calibration_debounce_frames`, `system_paused` — gesture debounce state
- `accumulated_wrist_rotation`, `base_joint_angle`, `prev_hand_y_axis` — wrist rotation tracking
- The Oculus joint index map (identical to `VRHandKeypointProcessor`)


### `register_pub_sub()`

Iterates over `data_stream_config["topics"]` and registers:

- **Subscriptions** for topics where `self.layer_cn_id` is in `subscribers`:
  - `component_interfaces/msg/ProcessedHandKeypoint` → `hand_keypoint_callback`
  - `component_interfaces/msg/RobotState` → `robot_state_callback`
- **Publishers** for topics where `publisher == self.layer_cn_id`:
  - `component_interfaces/msg/ControlCmd` (used for both arm and gripper commands)


### `hand_keypoint_callback(msg)`

Central processing callback. On every message:

1. Validates `hand_coords` (must be 72 floats) and `hand_frame` (must be 12 floats).
2. Reshapes into `(24, 3)` joint array and parses the 4×3 frame descriptor into named axes.
3. Calls `_check_calibration_gesture()`.
4. If not yet initialised and not paused, calls `_initialize_calibration()`.
5. If initialised and not paused, calls `_generate_robot_command()` and `_generate_gripper_command()`.

### `robot_state_callback(msg)`

Receives `RobotState` messages. Routes by `robot_id`:
- `"xarm5"` → updates `self.current_robot_pose` (end-effector `Pose`)
- anything else → treated as gripper state feedback

---

## Data I/O

### Subscribed Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `right_hand_keypoint_processed` | `component_interfaces/msg/ProcessedHandKeypoint` | Smoothed, canonical-frame hand keypoints from `VRHandKeypointProcessor` |
| `robot_state` | `component_interfaces/msg/RobotState` | Current robot end-effector pose (used for absolute command computation) |

### Published Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `robot_control_cmd` | `component_interfaces/msg/ControlCmd` | Absolute Cartesian target pose for the robot arm |
| `gripper_control_cmd` | `component_interfaces/msg/ControlCmd` | Joint-space gripper open/close command |

All topic names are resolved dynamically from the shared data stream configuration at runtime.

### `ControlCmd` Message Fields Used

| Field | Arm command | Gripper command |
|-------|-------------|-----------------|
| `robot_id` | `"xarm5"` | `"xarm_gripper"` |
| `space` | `"cartesian"` | `"joint"` |
| `relative` | `False` (absolute) | `False` (absolute) |
| `pose` | Target `Pose` (position + orientation) | — |
| `joint_angles` | — | `[0.85]` (closed) or `[0.0]` (open) |

---

## Hand-to-Robot Motion Retargeting

The retargeting pipeline converts the operator's hand pose — expressed in Oculus tracking space — into absolute robot Cartesian commands. It operates in three stages: calibration, position mapping, and orientation mapping.

### Stage 0 — Calibration (`_initialize_calibration`)

Before any commands are emitted the node captures two reference frames:

- `hand_origin_offset` — the wrist world position (`hand_frame['origin']`) at the moment of calibration.
- `robot_origin` — the robot end-effector XYZ position at the same moment.
- `last_hand_frame` — the full `{x, y, z}` axis triad of the palm frame at calibration time.

From this point on, all position and rotation commands are expressed as *deltas from these references*, making the mapping independent of where in the tracking volume the operator's hand happens to be.

### Stage 1 — Position Mapping (`_generate_robot_command`)

```
hand_position_delta  =  current_hand_frame['origin']  −  hand_origin_offset
```

The Oculus hand frame axes are:
- **X** — index → pinky (lateral, rightward)
- **Y** — palm normal (upward out of palm)
- **Z** — palm forward (pointing direction)

These are remapped to the robot base frame (X forward, Y left, Z up):

```
robot_delta_X  =  +hand_delta_Z   (hand forward  → robot forward)
robot_delta_Y  =  −hand_delta_X   (hand rightward → robot leftward)
robot_delta_Z  =  +hand_delta_Y   (hand palm-up   → robot upward)
```

The delta is scaled by `position_scale` and added to `robot_origin` and `workspace_offset` to yield an absolute target position.

### Stage 2 — Orientation Mapping (`_calculate_target_orientation`)

End-effector orientation is tracked as an *incremental* rotation — the change in palm frame between the previous and current callback:

```
R_delta  =  R_current · R_last^T
```

where `R_current` and `R_last` are 3×3 matrices whose columns are the current and previous palm `{x, y, z}` axis triples respectively.

The incremental rotation is then transformed into robot frame using the same axis permutation as the position mapping:

```
hand_to_robot  =  [[0, 0, 1],    # Hand Z → Robot X
                   [−1, 0, 0],   # Hand −X → Robot Y
                   [0, 1, 0]]    # Hand Y → Robot Z

R_robot_delta  =  hand_to_robot · R_delta · hand_to_robot^T
```

The rotation vector of `R_robot_delta` is scaled by `rotation_scale`, then composed onto the current robot orientation quaternion:

```
q_target  =  q_current  ⊗  q(R_robot_delta_scaled)
```

Publishing a fully specified `Pose` (position + orientation) on every callback gives the robot controller an absolute target in its base frame on each update.

### Stage 3 — Calibration Gesture (`_check_calibration_gesture` / `_handle_calibration_gesture`)

A dedicated in-operation recalibration flow is triggered by touching the **thumb tip to the middle finger tip** (joint indices 19 and 21 in the canonical frame):

```
distance  =  ‖ hand_coords[19] − hand_coords[21] ‖
```

A FIFO debounce counter (`calibration_gesture_cnt`) must reach `calibration_debounce_frames` before the gesture fires, preventing accidental triggers. The gesture is **edge-triggered** — it fires once per touch, not continuously. The state machine has two transitions:

```
System RUNNING  ──(gesture)──►  PAUSED + calibration reset
System PAUSED   ──(gesture)──►  RUNNING + new calibration initialised
```

While paused, all robot and gripper commands are suppressed. On resume, `_initialize_calibration()` is called immediately with the current hand and robot state as the new reference frame.

### Stage 4 — Gripper Control (`_get_gripper_state_from_hand_keypoints`)

The gripper is toggled by pinching the **thumb tip to the pinky tip** (joint indices 19 and 23):

```
distance  =  ‖ hand_coords[19] − hand_coords[23] ‖
```

A separate debounce counter (`gripper_cnt`) must hit `gripper_debounce_frames` exactly (equality, not `>=`) to fire a single toggle. After toggling, the counter continues to accumulate until the fingers separate, preventing repeated rapid toggles on a sustained pinch.

```
gripper_flag  =  NOT gripper_flag    (on each confirmed pinch)
target_angle  =  0.85  if closed     (joint-space command)
               =  0.0   if open
```

A `ControlCmd` is published only when the state changes, avoiding redundant messages.

<!-- ### Full Pipeline Summary

```
ProcessedHandKeypoint (24×3 joints + 4×3 frame)
            │
            ▼
  _check_calibration_gesture()
  [thumb-middle touch → pause/reset or resume/recalibrate]
            │
            ├──── system paused? ──── YES ──► suppress all commands
            │
            NO
            ▼
  _initialize_calibration()  (first callback only)
  [snapshot hand_origin, robot_origin, last_hand_frame]
            │
            ├──────────────────────────────────────────────┐
            ▼                                              ▼
  _generate_robot_command()              _generate_gripper_command()
  ┌─ position delta (hand→robot axes) ┐  [thumb-pinky pinch → toggle]
  │  + _calculate_target_orientation()│       │
  │    (incremental palm frame diff)  │       ▼
  └───────────────────────────────────┘  gripper_control_cmd (joint)
            │
            ▼
  robot_control_cmd (cartesian, absolute)
``` -->

---

## Acknowledgements

The gesture detection and motion retargeting design used in this node (`_check_calibration_gesture`, `_handle_calibration_gesture`, `_initialize_calibration`, `_calculate_target_orientation`, `_calculate_wrist_rotation`, `_get_gripper_state_from_hand_keypoints`, `reset_calibration`) is adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach

---
---

# Robot Driver: SimXArm5Driver Node Documentation

## Overview

`SimXArm5Driver` is a ROS2 robot driver node for a **simulated XArm5 robot arm and gripper** running in Gazebo. It bridges the high-level teleoperation command pipeline (`ControlCmd` messages) to the low-level `ros2_control` interfaces exposed by the Gazebo simulation: a Cartesian motion controller topic, and two `FollowJointTrajectory` action servers (one for the arm, one for the gripper). It simultaneously reads the simulation's joint state feedback and publishes it upstream as `RobotState` messages at a configurable frequency.

---

## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── RobotDriverBase (ABC, simulated=True)
              └── SimXArm5Driver
```


## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/robot_driver.yaml
```

### Config File Format

```yaml
# config_files/component_node/robot_driver.yaml

layer_cn_id: robot_driver
pkg_name: robot_driver
simulated: True

plugins:
  - id: sim_xarm5_driver
    priority: 1
    params:
      freq: 90                                              # State publishing frequency (Hz)
      scale_factor: 1.0                                    # Scale applied to relative motion commands
      joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5"]
```

### Parameter Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `freq` | `int` | `120` | Timer frequency in Hz for `publish_state()` |
| `scale_factor` | `float` | `1.0` | Multiplier applied to relative Cartesian and joint position deltas |
| `joint_names` | `list[str]` | `["joint1"…"joint5"]` | Names of the five robot arm joints, used to filter the `/joint_states` topic |

> **Note:** `simulated: True` at the top level is read by `RobotDriverBase` and sets `self.simulated = True`, which causes `_initialize_robot()` to skip any physical hardware connection and immediately mark the robot as connected.

---

## What the Node Does

`SimXArm5Driver` has three concurrent responsibilities:

1. **Command reception** — subscribes to `robot_control_cmd` and `gripper_control_cmd`, decodes each `ControlCmd`, and dispatches it to either the Cartesian controller or a `FollowJointTrajectory` action server.
2. **State feedback** — subscribes to `/joint_states` from `ros2_control`, separates arm and gripper joints, and queries TF2 for end-effector poses. A timer at `freq` Hz assembles and publishes `RobotState` messages upstream.
3. **Robot initialisation** — on startup, sets `robot_active = True` immediately (simulation mode assumes the robot is always reachable) and registers all required publishers, subscribers, and action clients.

---

## Features and Functions

### Initialisation (`__init__`)

After chaining through `RobotDriverBase.__init__(simulated=True)`:

- Sets default values for `freq`, `scale_factor`, `joint_names`, and `gripper_joint_names = ["drive_joint"]`.
- Defines hardcoded initial poses for the arm (`robot_initial_pose`) and gripper fingers (`gripper_initial_pose`).
- Creates a TF2 `Buffer` and `TransformListener` for end-effector pose queries.
- Calls `_load_my_config()`.
- Calls `_initialize_robot()` to set `robot_active = True`.


### `_load_my_config()`

Reads the YAML plugin block for `sim_xarm5_driver` and overwrites `freq`, `scale_factor`, and `joint_names` from `params`.


### `register_pub_sub()`

Creates all ROS2 interfaces needed for simulation control. There are two categories:

**From `data_stream_config` (dynamic):**

| Direction | Topic | Type | Condition |
|-----------|-------|------|-----------|
| Publisher | `robot_state` | `component_interfaces/msg/RobotState` | `publisher == layer_cn_id` |
| Publisher | `gripper_state` | `component_interfaces/msg/RobotState` | `publisher == layer_cn_id` |
| Subscriber | `robot_control_cmd` | `component_interfaces/msg/ControlCmd` | `layer_cn_id` in `subscribers` |
| Subscriber | `gripper_control_cmd` | `component_interfaces/msg/ControlCmd` | `layer_cn_id` in `subscribers` |

Both `ControlCmd` subscribers share the same callback (`cmd_sub_callback`) with the `topic_id` captured via closure.

**Fixed (always registered, independent of data stream config):**

| Interface | Type | ROS2 name | Purpose |
|-----------|------|-----------|---------|
| Subscriber | `sensor_msgs/JointState` | `/joint_states` | Gazebo joint feedback |
| Publisher | `geometry_msgs/PoseStamped` | `/xarm5_cartesian_motion_controller/target_frame` | Cartesian motion target |
| Action client | `FollowJointTrajectory` | `/xarm5_traj_controller/follow_joint_trajectory` | Arm joint trajectory execution |
| Action client | `FollowJointTrajectory` | `/xarm_gripper_traj_controller/follow_joint_trajectory` | Gripper trajectory execution |


### `register_timer()`

Cancels any existing timers and creates a single new ROS2 timer at period `1.0 / freq` seconds, bound to `publish_state()`.


### `joint_state_callback(msg: JointState)`

Receives `/joint_states` from `ros2_control` and splits the unified message into two separate `JointState` objects by name matching:

- Names in `self.joint_names` → `self.current_robot_joint_state`
- Names in `self.gripper_joint_names` → `self.current_gripper_joint_state`

Position, velocity, and effort arrays are all filtered and stored in parallel.


### `cmd_sub_callback(msg: ControlCmd, topic_id: str)`

Entry point for all incoming motion commands:

1. Calls `pong_reply(topic_id, msg)` for latency bookkeeping.
2. Rejects commands for unknown `robot_id` values.
3. Determines arm vs. gripper from `topic_id` or `msg.robot_id`.
4. Dispatches by `msg.space`:
   - `"cartesian"` → `move_cartesian(msg)` (arm only; gripper does not support Cartesian)
   - `"joint"` → `move_joint(msg, group='robot'|'gripper')`


### `publish_state()`

Called by the timer at `freq` Hz:

1. Calls `get_state("xarm5")` to assemble arm state (joint positions + TF-derived EEF pose).
2. Calls `get_state("xarm_gripper")` to assemble gripper state (joint position + finger-averaged pose).
3. Converts each to a `RobotState` message via `_create_robot_state_msg()` and publishes to `robot_state` / `gripper_state`.

---

## Data I/O

### Subscribed Topics

| Topic | Message Type | Source | Description |
|-------|-------------|--------|-------------|
| `robot_control_cmd` | `component_interfaces/msg/ControlCmd` | `MotionRetargetOT` | Arm Cartesian or joint-space command |
| `gripper_control_cmd` | `component_interfaces/msg/ControlCmd` | `MotionRetargetOT` | Gripper open/close joint command |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo / `ros2_control` | Unified joint feedback for all joints |

### Published Topics

| Topic | Message Type | Consumers | Description |
|-------|-------------|-----------|-------------|
| `robot_state` | `component_interfaces/msg/RobotState` | `MotionRetargetOT` | Arm joint state + EEF pose |
| `gripper_state` | `component_interfaces/msg/RobotState` | `MotionRetargetOT` | Gripper joint state + center pose |
| `/xarm5_cartesian_motion_controller/target_frame` (in use) | `geometry_msgs/msg/PoseStamped` | Gazebo Cartesian controller | Target EEF pose for Cartesian motion |

### Action Clients

| Action server | Type | Purpose |
|--------------|------|---------|
| `/xarm5_traj_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Execute arm joint trajectories |
| `/xarm_gripper_traj_controller/follow_joint_trajectory` (in use) | `control_msgs/action/FollowJointTrajectory` | Execute gripper open/close trajectories |

---

## Gazebo / ros2_control Integration

The node interfaces with the Gazebo simulation through two complementary `ros2_control` mechanisms — one for Cartesian control and one for joint-space control.

### Cartesian Control — Topic Publisher

```
cartesian_target_pub  →  /xarm5_cartesian_motion_controller/target_frame
                          (geometry_msgs/PoseStamped)
```

The `xarm5_cartesian_motion_controller` is a `ros2_control` controller loaded in the Gazebo simulation that accepts target end-effector poses and computes the required joint commands internally (via inverse kinematics or a Cartesian impedance law). The node simply publishes the desired `PoseStamped` in the `link_base` frame on every Cartesian command; the controller streams it to the simulated hardware interface.

**Absolute commands** — the target pose from `ControlCmd.pose` is published directly.

**Relative commands** — `_apply_relative_cartesian()` is called first: the current EEF pose is fetched from TF2 (`link_base` → `link5`), then the position delta is added component-wise with `scale_factor`, and the orientation delta is composed via quaternion multiplication after scaling the rotation vector.

---

### Joint-Space Control — `FollowJointTrajectory` Action

```
robot_traj_action_client   →  /xarm5_traj_controller/follow_joint_trajectory
gripper_action_client      →  /xarm_gripper_traj_controller/follow_joint_trajectory
                               (control_msgs/action/FollowJointTrajectory)
```

`FollowJointTrajectory` is the standard `ros2_control` joint trajectory interface. The node constructs a two-point trajectory on each command — start at the current joint position (from the last `/joint_states` message), end at the target — and sends it as an asynchronous action goal.

#### Trajectory Construction (`_send_robot_trajectory` / `_send_gripper_trajectory`)

Both helpers follow the same pattern:

```
point1  →  current_positions,  velocities=[0],  time=0 s
point2  →  target_positions,   velocities=[0],  time=T s
```

The execution time `T` is computed dynamically from the maximum joint displacement, assuming a conservative peak velocity of 1.0 rad/s:

```
T = max( max_displacement / 1.0,  0.1 )   [seconds]
```

This guarantees a minimum motion duration of 100 ms and scales linearly with the commanded displacement. The goal is sent with `send_goal_async()` — the node does not block waiting for completion, allowing subsequent commands to arrive while motion is executing.

For the **gripper**, the target position is clamped to `[0.0, 1.0]` before submission, and relative commands are scaled by `scale_factor`.

---

### TF2 State Feedback

Rather than relying on a dedicated state topic from the robot SDK (which would be used for a physical robot), the simulated driver reads the robot's current pose directly from the TF2 tree, which is populated by `robot_state_publisher` running alongside the Gazebo simulation:

| Query | `base_frame` | `eef_frame` | Used for |
|-------|-------------|------------|---------|
| Arm EEF pose | `link_base` | `link5` | `robot_state` message, `move_cartesian` current pose |
| Gripper center pose | `xarm_gripper_base_link` | `right_finger` + `left_finger` | `gripper_state` message |

The gripper center pose is the element-wise mean of the right and left finger positions in the gripper base frame (`get_gripper_center_pose`).

<!-- ---

### Full Command Flow

```
ControlCmd  (robot_control_cmd / gripper_control_cmd)
        │
        ▼
cmd_sub_callback()
        │
        ├── space == "cartesian" ──► move_cartesian()
        │                               │
        │                          get_eef_pose_from_tf()  [TF2]
        │                               │
        │                          _apply_relative_cartesian()  [if relative]
        │                               │
        │                          cartesian_target_pub.publish(PoseStamped)
        │                               │
        │                               ▼
        │                    /xarm5_cartesian_motion_controller/target_frame
        │                    [ros2_control → Gazebo hardware interface]
        │
        └── space == "joint"  ──► move_joint(group='robot'|'gripper')
                                        │
                                   _send_robot_trajectory()
                                   _send_gripper_trajectory()
                                        │
                                   FollowJointTrajectory.Goal
                                   (2-point trajectory, adaptive time)
                                        │
                                        ▼
                           /xarm5_traj_controller/...
                           /xarm_gripper_traj_controller/...
                           [ros2_control action server → Gazebo]


/joint_states  ────────────────────────────────────────────────────►
[Gazebo → ros2_control]                                             │
        │                                                           │
        ▼                                                      TF2 tree
joint_state_callback()                                        (robot_state_publisher)
  ├── current_robot_joint_state                                     │
  └── current_gripper_joint_state                                   │
                │                                                   │
                ▼                                                   ▼
         publish_state()  [timer @ freq Hz]             get_eef_pose_from_tf()
         ├── RobotState (robot_state)
         └── RobotState (gripper_state)
``` -->


---
---

# Feedback Data Processor: MultiFeedbackDataProcessor Node Documentation

## Overview

`MultiFeedbackDataProcessor` is a ROS2 data processor node that generates visual feedback data from processed hand keypoints. Currently it renders a **real-time 2D skeletal hand plot** and publishes it as a `Hand2DPlotData` message carrying a BGR image. The node is callback-driven and extensible.

---

## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── DataProcessorBase (ABC)
              └── MultiFeedbackDataProcessor
```

The helper class `PlotHand2D` is a self-contained Matplotlib renderer and is instantiated as a member object — it does not participate in the ROS2 node hierarchy.

---

## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/feedback_data_processor.yaml
```

### Config File Format

```yaml
# config_files/component_node/feedback_data_processor.yaml

layer_cn_id: feedback_data_processor
pkg_name: feedback_data_processor

plugins:
  - id: multi_feedback
    params:
      types: ["hand_2d_plot"]   # List of active feedback processing types
```

---

## What the Node Does

`MultiFeedbackDataProcessor` sits at the end of the teleoperation pipeline and produces operator-facing visual feedback. On every incoming `ProcessedHandKeypoint` message it:

1. Stores the latest message for reference.
2. Checks which feedback types are active (from `self.processing_types`).
3. For `"hand_2d_plot"`: renders a 2D skeletal hand image via `PlotHand2D` and publishes it as a `Hand2DPlotData` message.

---

## Features and Functions

### Initialisation

```python
MultiFeedbackDataProcessor(layer_cn_id: str, ros2_domain_id: Optional[int] = None)
```

After chaining through `DataProcessorBase.__init__`:

- Sets `processing_types = ["hand_2d_plot"]` as the default active pipeline list.
- Creates a `CvBridge` instance for converting OpenCV images to ROS2 `sensor_msgs/Image`.
- Creates a `tf2_ros.Buffer` and `TransformListener` for end-effector pose lookup.
- Calls `_load_my_config()` to overwrite `processing_types` from YAML if provided.
- Instantiates `PlotHand2D` as `self.plotter2D`.


### `_load_my_config()`

Reads the YAML plugin block for `multi_feedback` and overwrites `self.processing_types` from `params.types` if present. Logs a warning (but does not error) if no config is found, falling back to the default list.


### `register_pub_sub()`

Iterates over `data_stream_config["topics"]` and registers:

- A **subscription** for every topic where `self.layer_cn_id` is in `subscribers` and `msg_type` is `component_interfaces/msg/ProcessedHandKeypoint`. Callback: `hand_keypoint_callback`.
- A **publisher** for every topic where `publisher == self.layer_cn_id` and `msg_type` is `component_interfaces/msg/Hand2DPlotData`.


### `hand_keypoint_callback(msg)`

Stores the incoming message in `self.latest_hand_keypoints`, then conditionally calls `process_hand_data(msg)` if `"hand_2d_plot"` is in `self.processing_types`.


### `process_hand_data(msg)`

The main rendering pipeline (see full detail in the [2D Hand Plot section](#2d-hand-plot-visualisation) below):

1. Reshapes `msg.hand_coords` (72 floats) to `(24, 3)`.
2. Passes the X and Y columns to `self.plotter2D.draw()`, which returns an RGB NumPy array.
3. Converts RGB → BGR with `cv2.cvtColor`.
4. Wraps the image in a `Hand2DPlotData` message using `CvBridge`.
5. Publishes to `"handkeypoint_plot_2d"`.

---

## Data I/O

### Subscribed Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `right_hand_keypoint_processed` | `component_interfaces/msg/ProcessedHandKeypoint` | Smoothed 24-joint hand coordinates from `VRHandKeypointProcessor` |

### Published Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| `handkeypoint_plot_2d` | `component_interfaces/msg/Hand2DPlotData` | Rendered 2D hand skeleton image + metadata |

### `Hand2DPlotData` Message Fields Used

| Field | Content |
|-------|---------|
| `header` | Copied from input message, stamp updated to now |
| `hand_id` | Fixed string `"right_hand"` |
| `plotter` | `sensor_msgs/Image` (BGR8 encoding, 360×360 px at 60 DPI) |

---

## 2D Hand Plot Visualisation

### Helper Class: `PlotHand2D`

`PlotHand2D` is a stateful Matplotlib renderer that creates **one persistent figure** at construction time and reuses it on every frame, avoiding the overhead of creating and destroying figure objects at 90 Hz.

```python
self.fig = plt.figure(figsize=(6, 6), dpi=60)   # → 360×360 pixel output
self.canvas = FigureCanvas(self.fig)
self.ax = self.fig.add_subplot(111)
```

Using the non-interactive `Agg` backend (`matplotlib.use("Agg")`) means no GUI window is ever opened — the figure renders directly to an in-memory RGBA buffer.


### Coordinate Input

The `draw(X, Y)` method receives two 1D arrays of length 24 — the **X** and **Y** columns of the 24×3 canonical joint coordinate array produced by `VRHandKeypointProcessor`. These are already in the wrist-centred, rotation-normalised frame, so the plot is stable regardless of the operator's hand position or orientation in the VR tracking volume.

The fixed axis limits are:

```
X: [−0.12,  0.12]  metres
Y: [−0.02,  0.20]  metres
```


### Skeleton Drawing (`draw_hand`)

The skeleton is drawn by connecting joint pairs according to the Oculus joint index map:

| Connection group | How drawn |
|-----------------|-----------|
| Wrist → each metacarpal (joints 2, 6, 9, 12, 15) | Lines from joint 0 to each metacarpal |
| Within each finger group | Consecutive joint pairs along `knuckles`, `thumb`, `index`, `middle`, `ring`, `pinky` lists |
| All 24 joints | Red dots (`'ro'`) |

This produces a complete skeletal hand figure with five rays from the wrist and four segments per finger (plus the thumb chain).
<!-- 

### Thumb Constraint Boundary

A fixed 8-vertex polygon (`thumb_bounds`) is drawn over the plot every frame. It represents the **admissible region** for thumb tip motion — a spatial constraint boundary that provides the operator with visual feedback on whether the thumb is within the expected operating range. The polygon vertices are hardcoded in metres and match the canonical coordinate frame of the processed keypoints.

The polygon is redrawn on every frame via `_draw_thumb_bounds`, which connects consecutive vertices cyclically (including the closing edge from last back to first). -->


---

## Acknowledgements

The `PlotHand2D` class and 2D hand skeleton visualisation design are adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach

---
---

# Feedback Projector Driver: VRDriver Node Documentation

## Overview

`VRDriver` is a ROS2 feedback projector driver node that acts as the **outbound display bridge** between the ROS2 pipeline and the VR headset. It subscribes to processed visual feedback data (scene camera images and 2D hand skeleton plots), compresses them as JPEG, and streams them over ZeroMQ PUB sockets to the VR execution platform for display inside the headset. All processing is callback-driven — the node has no active internal timer.

The node also contains a helper class `ZMQCompressedImageTransmitter` that encapsulates the ZMQ publishing logic for a single image stream.


## Inheritance

```
rclpy.Node
  └── ComponentNode
        └── FeedbackProjectorDriverBase (ABC)
              └── VRDriver
```

The helper class `ZMQCompressedImageTransmitter` is a plain Python object instantiated as member attributes — it does not participate in the ROS2 node hierarchy.


## Configuration

### Config File Location and Naming

The configuration file **must** be named after the `layer_cn_id` and placed in:

```
config_files/component_node/<layer_cn_id>.yaml
```

For this node:

```
config_files/component_node/feedback_projector_driver.yaml
```

### Config File Format

```yaml
# config_files/component_node/feedback_projector_driver.yaml

layer_cn_id: feedback_projector_driver
pkg_name: feedback_projector_driver

plugins:
  - id: vr_driver
    params:
      width: 640                             # Target image width (pixels)
      height: 480                            # Target image height (pixels)
      execution_platform_ip: 192.168.178.92  # IP of the server connecting to the VR headset
      vr_cam_port: 10505                     # ZMQ port for scene camera stream
      vr_graph_port: 15001                   # ZMQ port for hand 2D plot stream
```

### Parameter Reference

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `width` | `int` | `1280` | Reference image width in pixels; actual output size is controlled per-callback by the `rescale_factor` passed to `_rescale_image` |
| `height` | `int` | `720` | Reference image height in pixels; see `width` note |
| `execution_platform_ip` | `str` | `192.168.178.54` | IP address of the machine running the VR application (ZMQ subscriber side) |
| `vr_cam_port` | `int` | `10505` | ZMQ PUB port for the scene camera RGB image stream |
| `vr_graph_port` | `int` | `15001` | ZMQ PUB port for the 2D hand skeleton plot image stream |


---

## What the Node Does

`VRDriver` is the final stage of the visual feedback pipeline. It receives rendered images produced upstream in the ROS2 graph and forwards them into the VR environment via compressed ZMQ streams:

1. **Binds two ZMQ PUB sockets** at initialisation — one per image channel — to the configured execution platform IP and ports.
2. **Subscribes to two ROS2 image topics**: a scene camera image and a 2D hand skeleton plot.
3. **On each incoming message**: decodes the image using `CvBridge`, rescales it (converting BGR→RGB in the process), and immediately transmits the JPEG-compressed frame over the corresponding ZMQ socket.

The VR application on the execution platform connects to these ZMQ sockets and renders each stream as a display panel inside the headset.

---

## Features and Functions

### Helper Class: `ZMQCompressedImageTransmitter`

A lightweight wrapper around a single ZMQ socket that handles context creation, socket lifecycle, and JPEG compression.

```python
ZMQCompressedImageTransmitter(host: str, port: int)
```

Calls `_init_publisher()` on construction. A `_init_push_socket()` variant (PUSH topology) is defined but unused.

**`_init_publisher()`** — Creates a ZMQ `PUB` socket and binds it to `tcp://<host>:<port>`.

**`send_image(rgb_image)`** — JPEG-encodes the RGB image via `cv2.imencode('.jpg', ...)` using `IMWRITE_WEBP_QUALITY = 10` (aggressive compression to minimise bandwidth) and sends the raw byte buffer over the socket.

**`stop()`** — Closes the socket and terminates the ZMQ context cleanly.

---

### Initialisation (`__init__`)

After chaining through `FeedbackProjectorDriverBase.__init__`:

- Sets default values for all network parameters and image dimensions.
- Creates a `CvBridge` instance for ROS2 ↔ OpenCV image conversion.
- Calls `_load_my_config()` to overwrite defaults from YAML.
- Calls `_register_zmq_pub()` to bind both ZMQ PUB sockets.


### `_load_my_config()`

Reads the YAML plugin block for `vr_driver` and overwrites `width`, `height`, `execution_platform_ip`, `vr_cam_port`, and `vr_graph_port` from `params`. Falls back silently to hardcoded defaults if the file or plugin block is absent.


### `_register_zmq_pub()`

Stops the existing `rgb_viz_zmq_publisher` if already running (safe re-registration), then creates two `ZMQCompressedImageTransmitter` instances bound to the configured host and ports:

| Attribute | Port param | Purpose |
|-----------|-----------|---------|
| `rgb_viz_zmq_publisher` | `vr_cam_port` | Scene camera RGB stream |
| `hand_2d_zmq_publisher` | `vr_graph_port` | 2D hand skeleton plot stream |


### `register_pub_sub()`

Iterates over `data_stream_config["topics"]` and creates ROS2 subscriptions for every topic where `self.layer_cn_id` appears in `subscribers`:

- `sensor_msgs/msg/Image` → `scene_image_callback`
- `component_interfaces/msg/Hand2DPlotData` → `hand_plot_callback`

This node registers **no publishers** — it is a pure sink in the ROS2 topic graph.


### `scene_image_callback(msg: Image)`

Triggered on every incoming scene camera frame:

1. Decodes `sensor_msgs/Image` to a BGR OpenCV array via `CvBridge`.
2. Calls `_rescale_image(img, rescale_factor=1)` — no spatial downscale, but performs BGR → RGB conversion.
3. Stores the result in `self.latest_scene_image`.
4. Calls `rgb_viz_zmq_publisher.send_image(...)` to transmit the compressed frame immediately.


### `hand_plot_callback(msg: Hand2DPlotData)`

Triggered on every incoming hand skeleton plot frame:

1. Calls `pong_reply('handkeypoint_plot_2d', msg)` for latency/heartbeat bookkeeping.
2. Decodes `msg.plotter` (a `sensor_msgs/Image`, BGR8 encoding) to an OpenCV BGR array via `CvBridge`.
3. Calls `_rescale_image(img, rescale_factor=4)` — downscales to ¼ linear dimensions and converts BGR → RGB.
4. Stores the result in `self.latest_hand_plot_data`.
5. Calls `hand_2d_zmq_publisher.send_image(...)` to transmit immediately.

The larger rescale factor (4×) for the hand plot versus the scene image (1×) reflects its role as a small overlay panel in the headset rather than a main viewport.


### `_rescale_image(image, rescale_factor) -> np.ndarray`

Shared utility used by both callbacks:

1. Converts BGR → RGB via `cv2.cvtColor`.
2. Computes new dimensions: `(original_width / rescale_factor, original_height / rescale_factor)`.
3. Resizes with `cv2.INTER_AREA` interpolation (optimal quality for downscaling).

Returns an RGB NumPy array ready for `send_image`.

---

## Data I/O

### Subscribed Topics

| Topic ID | Message Type | Description |
|----------|-------------|-------------|
| *(configured in data stream)* | `sensor_msgs/msg/Image` | Scene camera image from an upstream camera node |
| `handkeypoint_plot_2d` | `component_interfaces/msg/Hand2DPlotData` | Rendered 2D hand skeleton image from `MultiFeedbackDataProcessor` |

Topic names are resolved dynamically from the shared data stream configuration at runtime.

### Published Topics

None. `VRDriver` is a **pure sink** in the ROS2 topic graph. All outbound data is transmitted over ZMQ.

### ZMQ Output Sockets (Egress)

| Attribute | Socket type | Port | Content | Scale |
|-----------|-------------|------|---------|-------|
| `rgb_viz_zmq_publisher` | ZMQ PUB, TCP | `vr_cam_port` (10505) | Scene camera image | 1× (full resolution) |
| `hand_2d_zmq_publisher` | ZMQ PUB, TCP | `vr_graph_port` (15001) | Hand skeleton plot | ¼ (360→90 px linear) |

Both sockets **bind** on the local machine; the VR application **connects** to them from the execution platform. Each message is a raw JPEG byte buffer.

<!-- ---

## Data Flow Summary

```
ROS2 Pipeline                              ZMQ → VR Headset
────────────────────────────────────       ──────────────────────────────
sensor_msgs/Image
  (scene camera topic)
          │
          ▼
  scene_image_callback()
  CvBridge → BGR
  _rescale_image (×1) → RGB
          │
          └──► rgb_viz_zmq_publisher ──► TCP port 10505 ──► Scene viewport


component_interfaces/Hand2DPlotData
  (handkeypoint_plot_2d topic)
          │
          ▼
  hand_plot_callback()
  CvBridge → BGR
  _rescale_image (×4) → RGB
          │
          └──► hand_2d_zmq_publisher ──► TCP port 15001 ──► Overlay panel
``` -->

---

## Acknowledgements

The `ZMQCompressedImageTransmitter` class are adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach