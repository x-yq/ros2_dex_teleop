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
# MotionRetargetOT Node Documentation

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

### Full Pipeline Summary

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
```

---

## Acknowledgements

The gesture detection and motion retargeting design used in this node (`_check_calibration_gesture`, `_handle_calibration_gesture`, `_initialize_calibration`, `_calculate_target_orientation`, `_calculate_wrist_rotation`, `_get_gripper_state_from_hand_keypoints`, `reset_calibration`) is adapted from the **Open-Teach** teleoperation system:

> Iyer, A., Peng, Z., Dai, Y., Guzey, I., Haldar, S., Chintala, S., & Pinto, L. (2024).
> *OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation.*
> arXiv:2403.07870 [cs.RO]. https://arxiv.org/abs/2403.07870

Source repository: https://github.com/aadhithya14/Open-Teach