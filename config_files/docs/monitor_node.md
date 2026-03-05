# KPI Monitor Node

---

## 1. Architecture Overview

The KPI Monitor is a ROS 2 node responsible for collecting, computing, and logging resource usage metrics (CPU, memory) for Docker containers at runtime. It sits within the broader system as a passive observer: it does not control any hardware or data pipelines, but it watches over the containers that do.

The class hierarchy is:

```
rclpy.node.Node  (ROS 2 base)
    └── ComponentNode          [component_node.py]
            └── KPIMonitorBase [kpi_monitor_base.py]
                    └── DockerMonitor  [docker_monitor.py]
```

Each layer adds responsibility:

- **`ComponentNode`** provides the system-wide foundation: ROS 2 lifecycle, SystemManager communication (logging service, config service), state management (`disconnected → ready → running`), and latency testing utilities.
- **`KPIMonitorBase`** extends `ComponentNode` with monitoring-specific ROS 2 services (`start_monitor`, `stop_monitor`) and declares the abstract interface that every monitor plugin must implement.
- **`DockerMonitor`** is the concrete plugin that implements Docker-based monitoring using the Docker SDK and the Docker stats streaming API.

---

## 2. What the Monitor Node Does

### 2.1 Core Responsibility

When instructed, the `DockerMonitor` node attaches to one or more running Docker containers and periodically samples their resource usage. Every second, it reads raw CPU and memory counters from the Docker stats stream and stores them in memory. When monitoring is stopped, it post-processes all collected samples — computing derived metrics — and emits a structured log report via the system-wide logging service. 

The monitor is **non-intrusive**: it reads container stats through the Docker API socket (`unix://var/run/docker.sock`) without injecting any code into the container or modifying its runtime.

**Note**:

Lattency measurement is not the part of monitor node, for details see the `5. Latency Measurement: End-to-End Setup Guide` in the [file](systemManager_component_node.md)

### 2.2 Metrics Collected

The following metrics are collected per sample:

| Field | Source | Description |
|---|---|---|
| `cpu_usage_total` | `cpu_stats.cpu_usage.total_usage` | Cumulative CPU time used by the container (nanoseconds) |
| `pre_cpu_usage_total` | `precpu_stats.cpu_usage.total_usage` | CPU time at the previous sample interval |
| `system_cpu_usage` | `cpu_stats.system_cpu_usage` | Cumulative CPU time used by the entire host system |
| `pre_system_cpu_usage` | `precpu_stats.system_cpu_usage` | Host system CPU time at the previous interval |
| `online_cpu_count` | `cpu_stats.online_cpus` | Number of CPU cores available to the container |
| `mem_usage` | `memory_stats.usage` | Total memory currently allocated to the container (bytes) |
| `cache` | `memory_stats.stats.*_inactive_file` or `cache` | Page-cache memory held by the OS on behalf of the container (bytes) |

Raw values are collected as-is; derived metrics are computed only at the end of a monitoring session.

### 2.3 Computation Formulas

**CPU Percentage**

Docker reports cumulative CPU nanosecond counters, not instantaneous percentages. To derive a meaningful percentage per sample, the monitor uses the delta between two consecutive readings:

```
cpu_delta    = cpu_usage_total - pre_cpu_usage_total
system_delta = system_cpu_usage - pre_system_cpu_usage

cpu_percent  = (cpu_delta / system_delta) × 100 × online_cpu_count
```

- `cpu_delta` measures how many nanoseconds the container consumed between two consecutive stat reads.
- `system_delta` measures how many nanoseconds the entire system spent in that same interval (all cores combined).
- Multiplying by `online_cpu_count` scales the result so that 100% means "one full CPU core fully utilized". A container using two cores at capacity would report ~200%.
- Samples where either delta is ≤ 0 are skipped to avoid division errors or artefacts from counter resets.

**Memory — Process vs. Cache**

Docker's raw `usage` figure includes OS page cache memory, which is reclaimable and not truly "used" by the container's processes. The monitor separates the two:

```
mem_total_mib   = mem_usage / (1024 × 1024)
cache_mib       = cache    / (1024 × 1024)
mem_process_mib = max(mem_total_mib − cache_mib, 0.0)
```

- `mem_total_mib` — full memory footprint as seen by the kernel, in mebibytes (MiB).
- `cache_mib` — the portion that belongs to the page cache (inactive file-backed pages). This is identified from cgroup v1 (`total_inactive_file`), cgroup v2 (`inactive_file`), or the legacy `cache` key, depending on the host kernel version.
- `mem_process_mib` — the "real" working-set memory of the container's processes. This is the number that matters for capacity planning.

**Summary Statistics**

At stop time, the monitor computes session-level averages across all valid samples:

```
avg_cpu_percent     = Σ cpu_percent    / N
avg_mem_total_mib   = Σ mem_total_mib  / N
avg_mem_process_mib = Σ mem_process_mib / N
avg_cache_mib       = Σ cache_mib      / N
```

where N is the count of valid (non-skipped) samples.

---

## 3. Deployment and Usage: One Monitor Node Per Container

### 3.1 The Rule

**Each Docker container in the system must launch its own dedicated `DockerMonitor` node.** Do not deploy one monitor node per physical device or per host machine.

### 3.2 Why Not One Monitor Per Device?

A single monitor node watching multiple containers on the same host could technically be built — and experimentally, a monitor node running outside a container **can** open a Docker stats stream against a different container and receive data. However, this approach is rejected for the following reasons:

- **Degraded sampling rate (empirically observed)**: When a monitor node accesses the stats stream of a container it does not run inside, the number of samples collected over a fixed monitoring window is significantly lower than when the monitor is co-located inside the target container. The Docker stats streaming API delivers frames in response to `next()` calls; a monitor running outside the target container experiences higher inter-call latency — likely due to namespace crossing overhead and Docker daemon scheduling — causing it to miss frames within the 1-second timer interval. The result is an incomplete sample set and unreliable averages. This sampling degradation alone makes the cross-container approach unsuitable for accurate KPI measurement.

- **Failure coupling**: If a per-device monitor crashes or hangs, monitoring stops for every container on that machine simultaneously. A per-container monitor failure is isolated and does not affect other containers.

- **Lifecycle independence**: Each container starts and stops independently as part of the demo lifecycle. A per-container monitor starts and stops with its container, following the natural process lifecycle without requiring complex coordination logic.

### 3.3 How to Launch

The monitor node is launched via the container's own dedicated launch file. Each container has its own launch file, and the `DockerMonitor` node is declared within it. Refer to the [**config and deploy**](config_deploy.md) documentation for the exact launch file structure. In general, the pattern is:

1. The monitor node's `layer_cn_id` is derived directly from the container name. This naming convention ensures the monitor is unambiguously traceable back to its container.
2. Each container's launch file starts the `DockerMonitor` instance with that `layer_cn_id` alongside the container's other component nodes.
3. The `DockerMonitor` registers itself with the SystemManager on startup, making its `start_monitor` and `stop_monitor` services discoverable.

### 3.4 How to Use

Please refer to the following parts in this [document](ros2_interface.md):

- `system_interfaces/srv/StartMonitor`
- `system_interfaces/srv/StopMonitor`

---

## 4. Full Lifecycle Pipeline

The following describes the complete sequence from node initialization to final log output.

### Phase 1 — Node Initialization

```
DockerMonitor.__init__()
  │
  ├── KPIMonitorBase.__init__()
  │     ├── ComponentNode.__init__()
  │     │     ├── Creates ROS 2 node (node name = layer_cn_id)
  │     │     ├── Connects logging_client → SystemManager /logging service
  │     │     ├── Connects get_config_client → SystemManager /get_config service
  │     │     ├── Publishes initial state on /component_states topic
  │     │     └── Waits for SystemManager services to become available
  │     │
  │     ├── Creates ReentrantCallbackGroup (allows concurrent service handling)
  │     ├── Registers /start_monitor ROS 2 service
  │     └── Registers /stop_monitor ROS 2 service
  │
  ├── Connects to Docker daemon via unix socket
  ├── Initializes empty per-container tracking dictionaries
  └── check_plugins() → sets node state to "ready"
```

At this point the node is alive, registered with the system, and waiting for a `start_monitor` call. It is not yet collecting any data.

### Phase 2 — Start Monitoring

A caller sends a `StartMonitor` service request containing one or more `container_ids`.

```
start_monitor_srv_callback(request, response)
  │
  └── For each container_id in request.container_ids:
        │
        ├── Skip if already active (idempotent)
        │
        ├── _initialize_container_monitor(container_id)
        │     ├── client.containers.get(container_id)   — resolve container object
        │     ├── docker_api.stats(stream=True, decode=True)  — open stats stream
        │     └── container.exec_run("pgrep -f monit")  — discover monit PIDs
        │
        ├── container_kpi_metrics[container_id] = []    — initialize sample buffer
        │
        ├── create_timer(1.0 sec, collect_kpi(container_id))
        │     — fires every second, appending raw samples to the buffer
        │
        └── active_container_ids.add(container_id)

  Response: container_ids[], results_file_paths[], start_time, success
```

From this point on, `collect_kpi()` fires every 1 second per monitored container, calling `get_resource_usage()` which reads the next frame from the Docker stats stream and appends raw values to the buffer.

### Phase 3 — Periodic Collection (1 Hz timer)

```
collect_kpi(container_id)   [called every 1.0 second]
  │
  └── get_resource_usage(container_id)
        ├── get_cpu_percent()   → reads next stats frame, extracts 5 CPU counter values
        ├── get_memory_usage()  → reads next stats frame, extracts usage + cache bytes
        └── Returns dict: { timestamp, cpu_related[5], memory_related[2] }
  │
  └── Appends raw dict to container_kpi_metrics[container_id]
```

Note: CPU and memory each consume one stats stream frame. The Docker streaming API returns one JSON frame per call to `next()`, so each `collect_kpi` call reads two frames (one for CPU, one for memory). The raw counters are stored without any computation at this stage.

### Phase 4 — Stop Monitoring

A `StopMonitor` service request is sent with the `container_ids` to stop.

```
stop_monitor_srv_callback(request, response)
  │
  └── For each container_id in request.container_ids:
        │
        ├── monitor_timers[container_id].destroy()  — stop the 1 Hz timer
        │
        ├── _process_and_log_metrics(container_id)  — compute + emit log
        │     │
        │     ├── log_kpi(header: container_id + container_name)
        │     │
        │     ├── For each raw sample:
        │     │     ├── Compute cpu_percent   (formula in §2.3)
        │     │     ├── Compute mem_total_mib, cache_mib, mem_process_mib
        │     │     └── Append formatted CSV line to data_lines[]
        │     │
        │     ├── log_kpi("\n".join(data_lines))   — single batched log call
        │     │
        │     └── log_kpi(summary: N samples, avg_cpu, avg_mem_total,
        │                          avg_mem_process, avg_cache)
        │
        ├── Clean up:  del metrics, pids, container object, stats stream
        └── active_container_ids.remove(container_id)

  Response: container_ids[], results_file_paths[], success
```

### Phase 5 — Log Output

All log calls go through `log_kpi()`, which routes to two destinations simultaneously:

1. **ROS 2 logger** (`get_logger().warn(msg)`) — visible in the terminal.
2. **SystemManager logging service** (`self.logging(msg, log_level=f"MONITOR_{layer_cn_id}_{container_id[:12]}")`) — the log level string acts as a structured tag that identifies both the monitor node and the specific container, allowing the SystemManager to route and persist the data to the appropriate file (e.g. `/log/monitor_{container_id}.log`).

The final log output for a container follows this structure:

```
container_id: <id>, container_name: <name>
timestamp, cpu_percent, mem_total_mib, mem_process_mib, cache_mib
1712345678.123456, 12.345678, 256.000000, 200.000000, 56.000000
1712345679.124001, 13.012345, 257.000000, 201.000000, 56.000000
...
samples: 45, avg_cpu_percent: 12.678901, avg_mem_total_mib: 256.500000,
         avg_mem_process_mib: 200.500000, avg_cache_mib: 56.000000
```

### Emergency Stop

If the system issues an `urgent_stop` command via the `ComponentService`, `DockerMonitor.urgent_stop()` calls `stop_all_monitoring()`, which iterates over all active containers, destroys their timers, flushes and logs their collected metrics, and closes all stats streams. The node then transitions to the `disconnected` state.
