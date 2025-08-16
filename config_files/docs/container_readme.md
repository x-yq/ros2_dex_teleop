# Container Configuration

## Overview

These YAML files define container configurations for deploying ROS 2 nodes. It specifies on which `execution platform` deploy wich docker container and realted Docker settings, and environment configurations.

## File Structure

```yaml
platforms:
  - hardware_id: <string>          # Device hardware_id defined in hardware.yaml, only the execution plattform
    containers:
      - name: <string>             # Container name
        nodes:                     # List of node IDs (layer_cn_id)
          - <string>
          - <string>
        dockerfile: <string>       # Path to Dockerfile
        env:                       # Environment variables
          <key>: <value>
        network:                   # Network settings
          ipc: <string>
          net: <string>
        permissions:               # Container permissions
          privileged: <boolean>
        volume:                    # Volume mappings
          workdir_local: <string>  # Local workspace path
          workdir_container: <string> # Container workspace path
```
## Key Fields

- platforms: List of platforms where containers will be deployed.

    - hardware_id: The hardware id of the target device.

        - containers: List of containers to run on the device.
            - nodes: Node IDs (must match layer_cn_id from component configs).
            - dockerfile: Path to the Dockerfile for building the container.
            - env: Environment variables for running the container.
            - network: Network configuration (e.g., host mode).
            - permissions: Privileges granted to the container.
            - volume: Workspace mappings between local and container paths.

## Example

Please see the rest of the yaml files in the current folder.

