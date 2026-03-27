# DataCollector Node — Technical Documentation

---

## Overview

`DataCollector` is a ROS 2 node responsible for subscribing to a configured set of ROS 2 topics, buffering incoming messages in memory, and persisting them as structured demonstration datasets to disk. It is designed for robot learning data collection workflows, where teleoperation sessions or autonomous episodes need to be recorded in a format suitable for downstream model training.

The node is controlled entirely through ROS 2 service calls — there are no timers or internal triggers. An external orchestration script (or human operator) starts and stops recording sessions by calling the `/start_data_collection` and `/stop_data_collection` services.

---

## Architecture & Inheritance

`DataCollector` extends `ComponentNode`, which itself extends ROS 2's `Node`. The `ComponentNode` base class provides:

- Automatic status publishing to `/component_states` (topic: `ComponentStatus` message)
- Shared service clients for `Logging` and `GetConfig`, connected to the `SystemManager`
- Lifecycle helpers: `set_state()`, `start_component()`, `pause_component()`, `stop_component()`
- Config loader APIs: `load_demo_config()`, `load_data_stream_config()`, `load_my_config()`

One important distinction: unlike most component nodes, `DataCollector` does **not** register a `ComponentService` server (`cn_srv_{id}`). This is an explicit design choice — it is controlled exclusively through its own dedicated `StartDataCollection` / `StopDataCollection` services (see `component_node.py` line 76–77).

---

## Features & Capabilities

### Multi-session Recording
The node can manage multiple independent recording sessions concurrently, each identified by a unique `demo_id`. Active sessions are tracked in `self.active_collections`, and each session has its own set of subscribers, data buffer, and trajectory object.

### Dynamic Topic Subscription
When a session starts, the node dynamically creates ROS 2 subscribers for the topics specified in the request (or loaded from the demo config). Subscribers are created with a `ReentrantCallbackGroup` to allow parallel message processing.

### Hierarchical Feature Registration
Before data ingestion begins, the node introspects each subscribed message type and recursively parses its field structure. Every leaf field (scalar, array, known geometry/sensor type) is registered as a named feature using a path notation (e.g., `robot_state/joint_positions`). Fields such as `header`, `send_id`, `robot_id`, `joint_names`, `timeout`, `relative`, and `space` are filtered out automatically.

### In-memory Buffering with Thread Safety
Incoming messages are **not** written directly to disk. Instead, each callback appends `(timestamp_ns, numpy_value)` tuples to a per-demo, per-feature buffer (`self.data_buffer`). A `threading.Lock` per demo (`self.buffer_locks`) protects concurrent writes.

### Timestamping
If the incoming message has a `header.stamp` field, its timestamp is used directly (preserving the original publish time). If no header is present, the node falls back to the current ROS 2 clock.

### Flexible Storage Formats
Two storage backends are supported:

| Format | Description |
|--------|-------------|
| `vla`  | Using Robo-DM `Trajectory` class — the primary format |
| `hdf5` | HDF5 file with gzip compression (compression level 4) as a secondary backup |
 
### Metadata Persistence
At collection start, a `collection_metadata.json` file is written to the session's storage directory containing the demo ID, topic list, format, codec, and start time. At collection stop, this file is updated with the end time, total duration, and a finalization timestamp.

### Config Override at Runtime
If a `StartDataCollection` request supplies non-empty values for `demo_root_path`, `topics`, `video_codec`, or `storage_format`, those values override the config loaded from the `SystemManager`. This allows per-session customization without modifying config files.

### Urgent Stop / Emergency Shutdown
If the `SystemManager` broadcasts an `urgent_stop`, the node force-stops all active sessions, flushes buffers, finalizes metadata for each, and calls the base class `stop_component()`.

---

## Initialization Pipeline

The following steps happen when the node first starts up, inside `__init__` and `_initialize_node()` (inherited from `ComponentNode`):

```
Node.__init__()  (via ComponentNode)
    │
    ├── Create logging_client  → /logging  (to SystemManager)
    ├── Create get_config_client → /get_config  (to SystemManager)
    ├── Create component_channel_pub → /component_states
    └── _wait_for_services()  (blocks until SystemManager is reachable)

DataCollector.__init__()
    │
    ├── Initialize internal state
    │     ├── active_collections = {}
    │     ├── collection_subscribers = {}
    │     ├── data_buffer = {}
    │     └── buffer_locks = {}
    │
    ├── _initialize_node()
    │     ├── _load_root_configs()
    │     │     ├── load_demo_config()     → GetConfig(config_file_name="demo")
    │     │     ├── load_data_stream_config() → GetConfig(config_file_name="data_stream")
    │     │     └── _parse_topic_metadata()
    │     │           └── Builds self.topic_metadata from data_stream_config
    │     │
    │     └── check_plugins()
    │           └── Validates that demo_config, data_stream_config, and
    │               init_topics are all populated
    │
    └── Create service servers
          ├── /start_data_collection  (StartDataCollection)
          └── /stop_data_collection   (StopDataCollection)
```

After this sequence completes, the node logs `"DataCollector node initialized successfully"` and is ready to accept recording requests.

---

## Starting Data Collection

### Service Interface

**Service name:** `/start_data_collection`  
**Service type:** `system_interfaces/srv/StartDataCollection`

**Request fields:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `demo_id` | `string` | Yes | Unique session identifier (e.g., `"HZ_90"`) |
| `demo_root_path` | `string` | No | Root directory for output. Falls back to config if empty |
| `topics` | `string[]` | No | Topics to record. Falls back to config if empty |
| `video_codec` | `string` | No | Video codec override. Falls back to config if empty |
| `storage_format` | `string` | No | Storage format override. Falls back to config if empty |

**Response fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether recording started |
| `topic_ids` | `string[]` | The topics actually being recorded |
| `storage_path` | `string` | Full path to the output directory |
| `start_time` | `builtin_interfaces/Time` | Recording start timestamp |

### Command Line (Minimal — using defaults from config)

```bash
ros2 service call /start_data_collection \
  system_interfaces/srv/StartDataCollection \
  "{demo_id: 'HZ_90'}"
```

### Command Line (Full — overriding all fields)

```bash
ros2 service call /start_data_collection \
  system_interfaces/srv/StartDataCollection \
  "{demo_id: 'HZ_90', \
    demo_root_path: '/root/dev_ws/demos', \
    topics: ['robot_control_cmd', 'gripper_control_cmd', 'robot_state', 'gripper_state'], \
    video_codec: 'libx265', \
    storage_format: 'vla'}"
```

### What Happens Internally on Start

```
start_data_collection_srv_callback(request)
    │
    ├── Validate: demo_id must be non-empty
    ├── Validate: demo_id must not already be active
    ├── _update_demo_config(request)
    │     └── Merge request fields into self.demo_config[demo_id]
    │         (creates new entry if demo_id is new; updates existing)
    │
    ├── Resolve storage_path = demo_root_path / demo_id
    ├── os.makedirs(storage_path)
    ├── Validate storage_format and video_codec (fallback to defaults)
    │
    ├── _start_topic_collection(demo_id, topics, storage_path)
    │     ├── Create robodm.Trajectory(path="{demo_id}.vla", mode="w")
    │     ├── Initialize data_buffer[demo_id] = defaultdict(list)
    │     ├── Initialize buffer_locks[demo_id] = Lock()
    │     ├── Initialize registered_features[demo_id] = set()
    │     │
    │     └── For each topic_id in topics:
    │           ├── Resolve msg_type_class from data_stream_config
    │           ├── _register_topic_features(demo_id, topic_id, msg_type_class)
    │           │     └── Recursively parse message fields → register leaf paths
    │           └── Create ROS 2 subscriber with _create_topic_callback(...)
    │
    ├── Record start_time = get_clock().now()
    ├── active_collections[demo_id] = collection_info
    ├── _save_collection_metadata(...)  → writes collection_metadata.json
    └── logging("Started data collection ...", 'INFO')  → SystemManager
```

---

## Stopping Data Collection

### Service Interface

**Service name:** `/stop_data_collection`  
**Service type:** `system_interfaces/srv/StopDataCollection`

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `demo_id` | `string` | The session identifier to stop (must match the one used to start) |

**Response fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the session was cleanly stopped |
| `storage_path` | `string` | Path to the completed recording |
| `collection_duration` | `builtin_interfaces/Duration` | Total elapsed time |
| `message` | `string` | Result description or error detail |

### Command Line

```bash
ros2 service call /stop_data_collection \
  system_interfaces/srv/StopDataCollection \
  "{demo_id: 'HZ_90'}"
```

### What Happens Internally on Stop

```
stop_data_collection_srv_callback(request)
    │
    ├── Validate: demo_id must be in active_collections
    ├── Calculate duration = end_time - start_time
    │
    ├── _stop_topic_collection(demo_id)
    │     ├── Destroy all ROS 2 subscribers for this demo
    │     ├── _save_buffer_to_hdf5(demo_id)
    │     │     └── Write all buffered data to {demo_id}.h5 with gzip compression
    │     ├── _flush_buffer_to_trajectory(demo_id)
    │     │     ├── Sort each feature's samples by timestamp
    │     │     └── trajectory.add(feature_name, value, timestamp, time_unit="ns")
    │     ├── trajectory.close()  → finalizes the .vla file
    │     └── Clean up: registered_features, collection_log_files
    │
    ├── _finalize_collection_metadata(collection_info, end_time)
    │     └── Overwrites collection_metadata.json with end_time, duration,
    │         and finalized_at timestamp
    │
    └── del active_collections[demo_id]
```

---


## Storage & Data Format

### Directory Layout

Each session is saved to `{demo_root_path}/{demo_id}/`. Based on the example `demo.yaml`:

```
/root/dev_ws/demos/HZ_90/
    ├── HZ_90.vla
    ├── HZ_90.h5
    └── collection_metadata.json
```

### `collection_metadata.json` — Initial (written at start)

```json
{
  "demo_id": "HZ_90",
  "topics": ["robot_control_cmd", "gripper_control_cmd", "robot_state", "gripper_state"],
  "storage_format": "vla",
  "video_codec": "libx265",
  "start_time": 1720000000000000000,
  "created_by": "data_collector"
}
```

### `collection_metadata.json` — Finalized (written at stop)

```json
{
  "demo_id": "HZ_90",
  "topics": ["robot_control_cmd", "gripper_control_cmd", "robot_state", "gripper_state"],
  "storage_format": "vla",
  "video_codec": "libx265",
  "start_time": 1720000000000000000,
  "end_time":   1720000060000000000,
  "duration_seconds": 60.0,
  "created_by": "data_collector",
  "finalized_at": "2024-07-03T12:01:00.000000"
}
```

---

## Emergency Stop

If an `urgent_stop` signal arrives (from `SystemManager` or directly):

1. All active `demo_id` sessions are enumerated from `self.active_collections`.
2. For each, `_stop_topic_collection()` is called — subscribers are destroyed, buffers are flushed, the trajectory is closed.
3. Emergency metadata (without a clean duration) is written via `_finalize_collection_metadata()`.
4. All tracking dictionaries (`active_collections`, `collection_subscribers`, `collection_log_files`) are cleared.
5. `stop_component()` (base class) is called to transition the node to `disconnected` state.

---

## Configuration Reference

### `demo.yaml` — Example

```yaml
demo_id: HZ_90
demo_root_path: /root/dev_ws/demos
description: "Gazebo Xarm5 teleoperation with input data publish frequency 90HZ"
topics:
  - robot_control_cmd
  - gripper_control_cmd
  - robot_state
  - gripper_state
video_codec: libx265
storage_format: vla
```

This file is loaded at node startup via `GetConfig(config_file_name="demo")`. All fields can be overridden at runtime through the `StartDataCollection` service request.

<!-- ### Supported Values

| Parameter | Valid Values |
|-----------|-------------|
| `storage_format` | `hdf5`, `vla` |
| `video_codec` | `libx264`, `libx265`, `libaom-av1`, `ffv1`, `rawvideo`, `rawvideo_pickle`, `rawvideo_pyarrow` |

If an unsupported value is supplied, the node falls back to `vla` and `libx265` respectively and logs a warning. -->

---

## Hierarchical Feature Registration

### The Problem It Solves

ROS 2 messages are deeply nested objects. A message like `robot_state` might contain a `JointState` with positions and velocities, which itself contains arrays and nested sub-messages. To record this in a trajectory format, you need to flatten the tree into individually addressable named streams — one per scalar or array leaf. Feature registration is the step that builds this flat map before any data arrives.

### When It Happens

Feature registration is triggered once per topic, at the moment `_start_topic_collection()` sets up subscribers. For each topic:

1. The message type string (e.g., `"sensor_msgs/JointState"`) is resolved to a Python class via `_resolve_msg_type()`.
2. An empty instance of that class is instantiated: `msg_instance = msg_type_class()`.
3. `_register_topic_features()` is called, which immediately delegates to `_parse_and_register_fields()`.

### The Recursive Walk: `_parse_and_register_fields()`

This function walks a message instance recursively, using both value-based and type-definition-based inspection. The `prefix` argument tracks the current path, starting from the topic name itself (e.g., `robot_state`), and grows as the recursion descends (e.g., `robot_state/position`, `robot_state/pose/position/x`).

At each level, the algorithm is:

```
_parse_and_register_fields(demo_id, topic_id, msg_obj, prefix)
    │
    ├── Check: is msg_obj itself a leaf? (_is_leaf_field)
    │     └── If YES → register prefix as a feature name → STOP recursion
    │
    └── If NO → call msg_obj.get_fields_and_field_types()
          └── For each (field_name, field_type) in the message:
                │
                ├── SKIP fields: header, send_id, space, joint_names,
                │                timeout, relative, robot_id
                │
                ├── Build sub_feature_name = f"{prefix}/{field_name}"
                │
                ├── Check: is this a leaf by TYPE? (_is_leaf_field_by_type)
                │     └── If YES → register sub_feature_name → continue to next field
                │
                └── If NO → recurse into field_value
                      _parse_and_register_fields(sub_feature_name, ...)
```

The result is a flat set of strings stored in `self.registered_features[demo_id]`, for example:

```
robot_state/position
robot_state/velocity
robot_state/effort
gripper_state/position
robot_control_cmd/pose/position/x   ← if Pose fields are recursed into
robot_control_cmd/pose/orientation
```

### Two-Pass Leaf Detection

The node uses two complementary leaf checks, applied in priority order:

**Pass 1 — Value-based (`_is_leaf_field`):** Checks what the *runtime object* actually is. Matches on Python primitives, `np.ndarray`, ROS sensor/geometry message instances, `Time`/`Duration`, and lists/tuples of basic types.

**Pass 2 — Type-string-based (`_is_leaf_field_by_type`):** Checks the *declared type string* from `get_fields_and_field_types()`. This handles edge cases that value-based inspection can't catch, particularly empty arrays at registration time (before any messages have arrived). For example, a `sequence<float32>` field will be empty in a default-constructed message, but the type string tells us it's a leaf.

The type-string check explicitly recognizes:

| Type pattern | Examples | Decision |
|---|---|---|
| `sequence<primitive>` | `sequence<float32>`, `sequence<uint8>` | Leaf |
| `primitive[]` | `float64[]`, `int32[]` | Leaf |
| Primitive scalars | `float32`, `int64`, `bool`, `string` | Leaf |
| Known sensor types | `sensor_msgs/Image`, `sensor_msgs/PointCloud2`, `sensor_msgs/LaserScan` | Leaf |
| Known geometry types | `geometry_msgs/Pose`, `geometry_msgs/Vector3`, `geometry_msgs/Quaternion`, `geometry_msgs/Transform`, `geometry_msgs/PoseStamped` | Leaf |
| Navigation types | `nav_msgs/OccupancyGrid` | Leaf |
| Builtin time types | `builtin_interfaces/Time`, `builtin_interfaces/Duration` | Leaf |
| Everything else | nested ROS message types | Recurse deeper |

### Filtered Fields

The following fields are **always skipped** during the walk, regardless of their type:

- `header` — ROS standard header; timestamp is extracted separately
- `send_id` — Internal latency tracking field
- `space`, `relative`, `timeout` — Motion planning metadata not relevant to demonstration data
- `joint_names` — String labels; positions are recorded numerically
- `robot_id` — Identifier, not a data stream

### Feature Name Format

Feature names follow a slash-delimited path: `{topic_id}/{field_path}`. The topic name is stripped of its leading `/` before being used as the root prefix. For example, for a topic `/robot_state` with a nested `position` array, the feature name becomes `robot_state/position`.

---

## Converting Fields to NumPy

Once a message arrives at the callback, the node iterates over all registered features for that topic and extracts each value. Then `_convert_field_to_numpy()` converts every extracted value into a numpy array (or compatible type) before it is buffered.

### The Conversion Map

The function is structured as a cascade of `isinstance` checks. Below is the complete mapping:

#### Scalar Python types

| Python type | Output | dtype |
|---|---|---|
| `int`, `float` | `np.array([value])` | `float32` |
| `bool` | `np.array([value])` | `bool_` |
| `str` | string (unchanged) | — |
| `bytes` | `np.frombuffer(value)` | `uint8` |

#### ROS Builtin types

| Type | Output | dtype |
|---|---|---|
| `Time`, `Duration` | `np.array([sec, nanosec])` | `int64` |

#### ROS Geometry types

| Type | Output shape | dtype |
|---|---|---|
| `Point` | `[x, y, z]` → shape `(3,)` | `float64` |
| `Vector3` | `[x, y, z]` → shape `(3,)` | `float64` |
| `Quaternion` | `[x, y, z, w]` → shape `(4,)` | `float64` |
| `Pose` | `[x, y, z, qx, qy, qz, qw]` → shape `(7,)` | `float64` |
| `PoseStamped` | Same as `Pose` (header stripped) → shape `(7,)` | `float64` |
| `Transform` | `[tx, ty, tz, qx, qy, qz, qw]` → shape `(7,)` | `float64` |

#### ROS Sensor types

**`Image`**: Uses `CvBridge` as primary path. Based on the encoding field:
- `rgb8` / `bgr8` → converted via `imgmsg_to_cv2(desired_encoding='rgb8')` → `np.array(cv_image)` → shape `(H, W, 3)`
- `mono8` / `mono16` → passthrough → shape `(H, W)`
- Other encodings → passthrough

If `CvBridge` is unavailable, falls back to `np.frombuffer(data, dtype=uint8)` and reshapes based on encoding.

**`JointState`**: Extracts only `position` (drops `velocity` and `effort`) → `np.array(position, dtype=float64)`

**`PointCloud2`**: Returns raw data buffer as `np.frombuffer(data, dtype=uint8)`. Full field parsing (XYZ extraction) is noted as a future improvement.

**`LaserScan`**: Encoded into a single flat array:
```
[angle_min, angle_max, angle_increment, range_min, range_max, num_ranges, num_intensities,
 range_0, range_1, ..., range_N,
 intensity_0, intensity_1, ..., intensity_M]
```
dtype: `float32`

**`OccupancyGrid`**: Reshaped to `(height, width)`, dtype `int8`.

#### Python `array.array` (used by ROS 2 for typed sequences)

ROS 2 internally uses Python's `array.array` for typed sequence fields. The typecode is inspected and mapped:

| `array.typecode` | numpy dtype |
|---|---|
| `'f'` | `float32` |
| `'d'` | `float64` |
| `'i'`, `'l'` | `int32` |
| `'I'`, `'L'` | `uint32` |
| `'h'` | `int16` |
| `'H'` | `uint16` |
| `'b'` | `int8` |
| `'B'` | `uint8` |
| other | `float32` (fallback) |

#### Lists and tuples

- All `int`/`float` → `np.array(value, dtype=float32)`
- All `Pose` → stacked `(N, 7)` array, `float64`
- All `Point` → stacked `(N, 3)` array, `float64`
- All `Vector3` → stacked `(N, 3)` array, `float64`
- All `Quaternion` → stacked `(N, 4)` array, `float64`
- All `str` → kept as `list[str]`
- Mixed/unknown → kept as `list` (Robo-DM will pickle it)

#### Existing `np.ndarray`

Passed through as-is, no copy.

#### Unknown types

Passed through as-is with a debug log, relying on Robo-DM to handle pickling.

### Empty Value Guard: `_is_empty_field()`

Before conversion is attempted, and again after it, `_is_empty_field()` is called. This prevents storing zero-length or uninitialized data:

- `None` → skip
- Empty string → skip
- Empty `list`/`tuple` → skip
- `Image` with no data, or zero width/height → skip
- `PointCloud2` with no data → skip
- `LaserScan` with no ranges → skip
- `OccupancyGrid` with no data → skip
- `JointState` with no positions → skip
- Geometry types (`Pose`, `Point`, `Vector3`, etc.) → **always kept**, even with zero values (they are valid data points)
- Non-empty `np.ndarray` → always kept
- Non-empty `array.array` → always kept

---

## The In-Memory Buffer

### Structure

All converted data is held in memory until the session is stopped. The buffer is a nested defaultdict:

```python
self.data_buffer[demo_id]           # defaultdict(list)
    └── [feature_name]              # e.g. "robot_state/position"
          └── [(timestamp_ns, numpy_value), ...]
```

Each callback call appends one `(int, np.ndarray)` tuple to the relevant feature list. This means no file I/O occurs during the recording window — everything is in RAM.

Thread safety is enforced by a per-demo `threading.Lock` (`self.buffer_locks[demo_id]`), which is held for the duration of each `with lock:` block in the callback.

### Timestamping

For each incoming message, the timestamp is resolved as follows:

```
if msg has header.stamp:
    timestamp_ns = header.stamp.sec * 1e9 + header.stamp.nanosec
else:
    now = get_clock().now()
    timestamp_ns = now.seconds_nanoseconds() → combined to ns
```

This preserves the publisher's original timestamp wherever available, which is important for accurate time alignment between topics during model training.

---

## Flushing to Robo-DM Trajectory (`.vla`)

### What Is a Robo-DM Trajectory?

`robodm.Trajectory` is a file-backed data store designed for robot demonstration data. It maps named features to time-indexed arrays. A `.vla` file is the primary output format for VLA model training.

### The Flush: `_flush_buffer_to_trajectory()`

This is called at collection stop, after subscribers are destroyed (so no new data can arrive) and after the HDF5 backup has been written.

```python
def _flush_buffer_to_trajectory(self, demo_id):
    trajectory = self.trajectories[demo_id]
    buffer     = self.data_buffer[demo_id]
    lock       = self.buffer_locks[demo_id]

    with lock:
        for feature_name, samples in buffer.items():
            samples.sort(key=lambda x: x[0])        # ensure monotonic timestamps

            for ts_ns, value in samples:
                trajectory.add(
                    feature_name,
                    value,
                    timestamp=ts_ns,
                    time_unit="ns"
                )
```

Each `trajectory.add()` call records one timestamped sample for one feature. The final `trajectory.close()` call seals the file.

### Why Buffer First, Then Flush?

Writing to a `Trajectory` file while many concurrent ROS callbacks are running would require holding a file-level lock on every single message — expensive and potentially blocking. By buffering to a plain Python `defaultdict` in memory, callbacks remain fast and non-blocking. The flush is a single-threaded, ordered write that happens once after recording ends.

---

## Writing the HDF5 Backup (`.h5`)

The HDF5 file is written by `_save_buffer_to_hdf5()`, which runs immediately before the Robo-DM flush at stop time. It serves as a raw backup and is independent of the Robo-DM format.

### File Structure

The HDF5 file mirrors the feature name hierarchy using HDF5 groups. A feature named `robot_state/position` becomes a group path `robot_state/position/` containing two datasets:

```
HZ_90_buffer.h5
└── robot_state/
│     └── position/
│           ├── timestamps    [N,]  int64   (nanoseconds)
│           └── data          [N, D]  float64 (the numpy values)
├── gripper_state/
│     └── ...
└── robot_control_cmd/
      └── ...
```

### Compression

Both `timestamps` and `data` datasets are written with gzip compression at level 4, balancing file size against write speed:

```python
group.create_dataset("timestamps", data=timestamps, compression="gzip", compression_opts=4)
group.create_dataset("data",       data=values,     compression="gzip", compression_opts=4)
```

### Variable-Length Data Handling

If samples for a feature have inconsistent shapes (e.g., a `PointCloud2` scan that varies in point count between frames), `np.stack()` will fail. The fallback is:

```python
values = np.array([v for _, v in samples], dtype=object)
```

This creates an object array that HDF5 stores as a variable-length dataset, preserving all data without loss.

### Sorting

Before writing, all sample lists are sorted by `timestamp_ns` to ensure monotonic time order in the output datasets — matching the sort done before the Robo-DM flush.

---

## End-to-End Data Flow Summary

```
[Message arrives on /robot_state]
         │
         ▼
  topic_callback(msg)
         │
         ├── Extract timestamp_ns (from header or clock)
         ├── For each registered feature under "robot_state":
         │     ├── _extract_field_value(feature_name, topic_id, msg)
         │     │     └── Traverse msg attributes by path parts
         │     │
         │     ├── _is_empty_field(raw_value)?  → skip if True
         │     │
         │     ├── _convert_field_to_numpy(feature_name, raw_value)
         │     │     └── Dispatch by type → numpy array
         │     │
         │     ├── _is_empty_field(numpy_value)?  → skip if True
         │     │
         │     └── buffer["robot_state/position"].append((ts_ns, numpy_array))
         │                             (under lock)
         │
         ▼
  [data sits in RAM during recording window]
         │
[/stop_data_collection called]
         │
         ├── Destroy all subscribers  (no more callbacks)
         │
         ├── _save_buffer_to_hdf5()
         │     └── For each feature:
         │           ├── Sort by timestamp
         │           ├── Stack → (N, D) array
         │           └── Write timestamps + data to HDF5 group (gzip-4)
         │
         ├── _flush_buffer_to_trajectory()
         │     └── For each feature:
         │           ├── Sort by timestamp
         │           └── trajectory.add(name, value, ts, time_unit="ns")
         │
         └── trajectory.close()  →  HZ_90.vla  sealed

[Output]
  /root/dev_ws/demos/HZ_90/
      ├── HZ_90.vla              ← Robo-DM Trajectory (primary, VLA training)
      ├── HZ_90_buffer.h5        ← HDF5 backup (raw, per-feature time series)
      └── collection_metadata.json
```
