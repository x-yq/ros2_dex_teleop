# XArm5 Gazebo Simulation — Setup and Usage Guide

## Prerequisites

- x86 machine running Ubuntu
- Docker and Docker Compose installed
- Repository cloned locally

---

## 1. Enable GUI Display (X11 Forwarding)

The Gazebo GUI requires access to an X11 display. Before deploying or launching anything, run the following on the **host machine**:

```bash
xhost +local:
export DISPLAY=:0
```

`xhost +local:` grants the local Docker container permission to connect to the host X server. `DISPLAY=:0` tells applications inside the container which display to render to. Without this step the Gazebo window will fail to open.

---

## 2. Deploy and Launch

### 2a. Run the Deploy Script

The `deploy_system.py` script manages Docker Compose deployment across local and remote machines. To **launch the simulator container locally**, use the `--launch-env` flag:

```bash
python3 deploy_system.py --launch-env --local-mode
```

This flag specifically targets containers marked as `name: simulator` in `config_files/deployment.yaml`, passes `ROBOT` and `GRIPPER` environment variables to the container, and runs Docker Compose locally rather than SSHing to a remote machine.

For detail please see the [document](config_deploy.md)

To **stop** the simulator container:

```bash
python3 deploy_system.py --launch-env --local-mode --stop
```

### 2b. Enter the Container and Launch the Simulation

Once the container is running, open a shell inside it:

```bash
docker exec -it <simulator_container_name> bash
```

Source the ROS 2 workspace inside the container:

```bash
source /root/dev_ws/install/setup.bash
source /opt/ros/humble/setup.bash
```

Then launch the XArm5 Gazebo simulation:

```bash
ros2 launch xarm_gazebo xarm5_beside_table_gazebo.launch.py
```

### 2c. Launch the Monitor Node

Inside the same container (or in a new shell), launch the system monitor node:

```bash
ros2 launch <your_pkg> simulation_monitor_launch.py \
  config_folder_path:=/root/dev_ws/src/config_files \
  ros_domain_id:=0
```

The `simulation_monitor_launch.py` file reads `deployment.yaml` to determine the correct monitor node name (format: `monitor_<hardware_id>_<container_name>`), checks whether a node with that name is already running on the network, and only spawns a new `kpi_monitor_node` if one is not already present. This prevents duplicate monitors when restarting containers.

---

## 3. Known Issue — `Waiting for spawn_entity Service` (gzserver hang)

On first launch you may encounter the terminal output repeatedly printing:

```
[spawn_entity.py] Waiting for spawn_entity Service...
```

This is a known Gazebo bug where `gzserver` starts but does not correctly initialise its ROS service interface, preventing the robot model from being inserted into the world.

**Resolution steps:**

1. Wait until the Gazebo window pops up — even if the robot model is missing from the scene.

2. Find the PID of the stuck `gzserver` process:

```bash
ps -a | grep gzserver
```

3. Kill it:

```bash
kill -9 <PID of gzserver>
```

4. Re-run the launch file:

```bash
ros2 launch xarm_gazebo xarm5_beside_table_gazebo.launch.py
```

On the second launch, Gazebo and `spawn_entity` typically initialise correctly and the robot model is inserted into the scene.

---

## 4. XArm5 Robot Description

The XArm5 is a 5-DOF serial robot arm manufactured by UFACTORY. Its URDF description is assembled from four composable xacro files:

### 4a. Kinematic Structure (`xarm5_urdf.xacro`)

The robot has six rigid bodies connected by five revolute joints, all rotating about their local Z axis:

| Link | Parent | Joint | Effort limit | Velocity limit | Damping / Friction |
|------|--------|-------|-------------|----------------|--------------------|
| `link_base` | `world` | — | — | — | — |
| `link1` | `link_base` | `joint1` | 50 Nm | 3.14 rad/s | 10 / 1 |
| `link2` | `link1` | `joint2` | 50 Nm | 3.14 rad/s | 10 / 1 |
| `link3` | `link2` | `joint3` | 30 Nm | 3.14 rad/s | 5 / 1 |
| `link4` | `link3` | `joint4` | 20 Nm | 3.14 rad/s | 3 / 1 |
| `link5` | `link4` | `joint5` | 20 Nm | 3.14 rad/s | 3 / 1 |

`link5` is the end-effector link — it serves as the attachment point for the gripper and is the frame queried by TF2 for the end-effector pose.

**Default joint limits** (unlimited mode):

| Joint | Lower (rad) | Upper (rad) |
|-------|-------------|-------------|
| joint1 | −2π | +2π |
| joint2 | −2.059 | +2.094 |
| joint3 | −3.927 | +0.192 |
| joint4 | −1.693 | +π |
| joint5 | −2π | +2π |

Each link carries full inertial parameters (mass, centre of mass, inertia tensor) loaded from `xarm5_type5_HT_BR2.yaml` by default, enabling realistic dynamics simulation.

### 4b. ros2_control Interface (`xarm5_ros2_control.xacro`)

Each of the five joints exposes two command interfaces and two state interfaces to `ros2_control`:

- **Command interfaces:** `position`, `velocity`
- **State interfaces:** `position`, `velocity`

The hardware plugin is selected at launch time via the `ros2_control_plugin` argument:
- Simulation: `gazebo_ros2_control/GazeboSystem` — connects joints to Gazebo's physics engine
- Physical hardware: `uf_robot_hardware/UFRobotSystemHardware` — connects to the real robot over the network

### 4c. Scene Composition

At launch the robot is spawned into `table.world` at a fixed offset from the world origin:

```
position:  x=-0.2,  y=-0.5,  z=1.021
rotation:  yaw=π/2  (1.571 rad)
```

A simulated camera (`sim_camera.sdf`) is also spawned into the world as a separate entity immediately after the robot.

---

## 5. ros2_control Controllers

Three controllers are loaded and managed by the `controller_manager`, configured in `xarm5_controllers.yaml`. The controller manager runs at **100 Hz** update rate.

### 5a. `joint_state_broadcaster`

**Type:** `joint_state_broadcaster/JointStateBroadcaster`

Reads the position, velocity, and effort state interfaces of all joints from the hardware and republishes them as a unified `sensor_msgs/msg/JointState` message on the `/joint_states` topic. This is the source of all joint feedback consumed by `SimXArm5Driver` and by `robot_state_publisher` (which uses it to populate the TF2 tree).

### 5b. `xarm5_traj_controller`

**Type:** `joint_trajectory_controller/JointTrajectoryController`

Accepts `FollowJointTrajectory` action goals and executes smooth, time-parameterised joint trajectories. Used for **joint-space** arm commands.

| Parameter | Value |
|-----------|-------|
| Command interfaces | `position`, `velocity` |
| State interfaces | `position`, `velocity` |
| Controlled joints | `joint1` … `joint5` |
| Goal time tolerance | 1.5 s |
| Stopped velocity tolerance | 0.05 rad/s |
| Per-joint trajectory tolerance | 1.0 rad |
| Per-joint goal tolerance | 0.05 rad |
| State publish rate | 25 Hz |
| Action monitor rate | 10 Hz |

> **Note:** This controller is loaded as **inactive** at launch (`--inactive` flag in the spawner) to avoid conflicting with the Cartesian motion controller, which also commands joint positions. It can be activated via the `controller_manager` service if joint-space trajectory control is needed exclusively.

### 5c. `xarm5_cartesian_motion_controller`

**Type:** `cartesian_motion_controller/CartesianMotionController`

Accepts `geometry_msgs/PoseStamped` targets on the topic `/xarm5_cartesian_motion_controller/target_frame` and computes the required joint commands via an iterative Jacobian-based solver. Used for **Cartesian-space** arm commands.

| Parameter | Value |
|-----------|-------|
| End-effector link | `link5` |
| Robot base link | `link_base` |
| Controlled joints | `joint1` … `joint5` |
| Command interfaces | `position` |
| Solver error scale | 1.0 |
| Solver iterations | 10 |
| PD gains (all axes) | P = 0.5 |

All six Cartesian axes (X, Y, Z translation and X, Y, Z rotation) use a proportional gain of 0.5. The solver runs iteratively up to 10 times per control cycle to minimise the Cartesian error before issuing position commands to the joints.

### 5d. `xarm_gripper_traj_controller`

**Type:** `joint_trajectory_controller/JointTrajectoryController`

Controls the gripper's `drive_joint` using the same `FollowJointTrajectory` action interface as the arm trajectory controller. Spawned only when `add_gripper:=true` is passed at launch. `SimXArm5Driver` sends two-point trajectories with adaptive timing to open (0.0) or close (0.85) the gripper.

