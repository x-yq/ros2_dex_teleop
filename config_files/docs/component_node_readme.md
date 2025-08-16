# Component Node Configuration

## Overview

These YAML file define component node configuration. Each component node has thier own file. It specifies plugins and their parameters for handling different types of camera sensors.

## File Structure

```yaml
layer_cn_id: <string>               # Must match the node name
pkg_name: <string>                  # ROS 2 package name

meta:
  description: <string>             # Brief description of the component
  layer: <string>                  # Layer classification (e.g., "driver")
  type: [<string>]                 # Types (e.g., "feedback_sensor")
  priority: <integer>              # Execution priority
  ros2_domain_id: <integer|None>   # Optional ROS 2 domain ID

default_plugin: <string>           # Fallback plugin ID

plugins:
  - id: <string>                   # Plugin identifier
    class_path: <string>           # Python class path
    priority: <integer>            # Plugin priority
    params:                        # Plugin-specific parameters
      <param1>: <value>
      <param2>: <value>
      ...
```

## Key Fields

- `layer_cn_id`: Unique identifier matching the node name.

- `pkg_name`: The ROS 2 package containing the node implementation.

- `meta`: Metadata describing the component's role and behavior.

- `default_plugin`: The plugin to use if no other is specified.

- `plugins`: List of available plugins with their configurations.

## Example

Please see the rest of the yaml files in the current folder.




