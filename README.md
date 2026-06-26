<div align="center">

 <img src="image/moveit.png" alt="DOBOT TCP-IP-ROS-6AXis" style="max-width: 600px; margin-bottom: 20px;" />

 <h1>TCP-IP-ROS-6AXis</h1>

 **Dobot Robot ROS Software Development Kit**  
 High-performance robot control framework based on TCP/IP protocol

 [English](README.md) · [简体中文](README_ZH.md)

 [![Platform](https://img.shields.io/badge/Platform-Ubuntu%2016.04%2F18.04%2F20.04-blue?style=flat-square)](https://ubuntu.com/download/server)
 [![ROS](https://img.shields.io/badge/ROS1-Kinetic%2FMelodic%2FNoetic-green?style=flat-square)](https://docs.ros.org/)
 [![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)](LICENSE)

</div>

---

## Quick Start

### Prerequisites

| Requirement | Version |
|-------------|----------|
| OS | Ubuntu 16.04 / 18.04 / 20.04 LTS |
| ROS | ROS1 Kinetic / Melodic / Noetic |
| Python | 2.7+ / 3.8+ |

### Network Configuration

| Item | Description |
|------|-------------|
| Robot IP (LAN1) | 192.168.5.1 (must be in the same subnet) |
| Robot IP (LAN2) | 192.168.100.1 |
| Robot IP (Wireless) | 192.168.1.6 |
| Control Port (V3/V4) | 29999 |
| Motion Port (V3) | 30003 |
| Feedback Port (V3/V4) | 30004 |

### Installation

```bash
# Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
cd ~/catkin_ws

# Install dependencies
sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-gazebo-* \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-robot-state-publisher

# Build
catkin_make
source devel/setup.bash

# Specify robot type (select based on actual model)
# Example: CR5
echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
# Supported models: CR3, CR5, CR7, CR10, CR12, CR16, me6, nova2, nova5

# Apply configuration
source ~/.bashrc
```

---

## Usage

### 1. RViz Visualization (Standalone Mode)

View model without connecting to real robot:

```bash
roslaunch dobot_description display.launch
```

### 2. RViz with Real Robot (V3 Version)

Display real-time joint states from robot:

```bash
# Terminal 1: Connect to robot (V3)
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1

# Terminal 2: Launch RViz
roslaunch dobot_description display_connected.launch
```

### 3. RViz with Real Robot (V4 Version)

```bash
# Terminal 1: Connect to robot (V4)
roslaunch dobot_v4_bringup bringup_v4.launch robot_ip:=192.168.5.1

# Terminal 2: Launch RViz
roslaunch dobot_description display_connected.launch
```

### 4. MoveIt Virtual Demo

Test motion planning in RViz (no robot required, standalone mode):

```bash
roslaunch dobot_moveit demo.launch
```

> **Note**: `demo.launch` publishes fake joint states. **Do not** run alongside real robot drivers.

### 5. MoveIt with Real Robot (V3)

Complete motion planning and execution:

```bash
# Terminal 1: Connect to robot
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1

# Terminal 2: MoveIt interface
roslaunch dobot_moveit moveit.launch
```

### 6. MoveIt with Real Robot (V4)

```bash
# Terminal 1: Connect to robot
roslaunch dobot_v4_bringup bringup_v4.launch robot_ip:=192.168.5.1

# Terminal 2: MoveIt interface
roslaunch dobot_moveit moveit.launch
```

### 7. Gazebo + MoveIt Simulation

Physics simulation with motion planning:

```bash
# Terminal 1: Launch Gazebo
roslaunch dobot_gazebo gazebo.launch

# Terminal 2: Launch MoveIt
roslaunch dobot_moveit moveit.launch fake_execution:=true
```

### 8. Motion Demo Script (V3 Only)

Run pre-built demo for quick testing (**V3 version only**):

```bash
# Terminal 1: Connect to robot (V3)
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1

# Terminal 2: Run demo
rosrun dobot_bringup demo
```

---

## Launch Parameters

### Environment Variables

| Variable | Default | Description |
|----------|--------|-------------|
| `DOBOT_TYPE` | cr5 | Robot model (cr3, cr5, cr7, cr10, cr12, cr16, me6, nova2, nova5) |

---

## Project Structure

```
TCP-IP-ROS-6AXis/
├── dobot_bringup/           # V3 Driver Node (TCP/IP Communication)
│   ├── include/dobot_bringup/
│   ├── src/
│   ├── launch/
│   ├── msg/
│   └── srv/
├── dobot_v4_bringup/        # V4 Driver Node (TCP/IP Communication)
│   ├── include/dobot_v4_bringup/
│   ├── src/
│   └── launch/
├── dobot_description/       # Robot URDF Model
│   ├── launch/
│   │   ├── display.launch           # Standalone mode
│   │   └── display_connected.launch # Connected to real robot mode
├── dobot_gazebo/            # Gazebo Simulation
├── dobot_moveit/            # Generic MoveIt Launch Package
├── rosdemo_v4/              # V4 Demo Package
├── rviz_dobot_control/      # RViz Control Plugin
├── cr3_moveit/              # CR3 MoveIt Configuration
├── cr5_moveit/              # CR5 MoveIt Configuration
├── cr7_moveit/              # CR7 MoveIt Configuration
├── cr10_moveit/             # CR10 MoveIt Configuration
├── cr12_moveit/             # CR12 MoveIt Configuration
├── cr16_moveit/             # CR16 MoveIt Configuration
├── me6_moveit/              # ME6 MoveIt Configuration
├── nova2_moveit/            # Nova2 MoveIt Configuration
├── nova5_moveit/            # Nova5 MoveIt Configuration
├── image/                   # Documentation Images
├── README.md
├── README_ZH.md
└── LICENSE
```

---

## Architecture & Data Flow

### Driver Architecture

```
ROS Application Layer
  └── CRRobot (Action Server + Services)
        └── CR5Commander (TCP Communication Wrapper)
              └── TCP 29999 (Control & Motion Commands)
                    └── RealTimeData (Joint States)
                          └── /joint_states (ROS Topic)
                                └── MoveIt / RViz
```

### Trajectory Execution Flow

```
MoveIt Planning Layer
  └── RRTConnect Planner generates waypoints
        └── Time Parameterization (adds timestamps and velocities)
              └── FollowJointTrajectoryAction
                    └── Controller sends waypoints sequentially (servoj commands)
                          └── Robot executes smoothly
```

### Key Components

| Component | Package | Responsibility |
|-----------|---------|----------------|
| `cr5_robot.cpp` / `cr5_v4_robot.cpp` | `dobot_bringup` / `dobot_v4_bringup` | ROS services and action server implementation |
| `commander.h` | `dobot_bringup` / `dobot_v4_bringup` | TCP communication wrapper and data parsing |
| `tcp_socket.cpp` | `dobot_bringup` / `dobot_v4_bringup` | Low-level socket communication |

### Core Design

| Design Aspect | Description |
|---------------|-------------|
| **Planner** | RRTConnect, generates dense waypoints |
| **Interpolation** | MoveIt handles time parameterization, controller sends waypoints sequentially |
| **servoj t value** | Dynamically calculated from adjacent waypoint time difference |
| **Stop Functionality** | Supports Action cancel and service call to stop |

---

## Supported Models

| Series | Models |
|--------|--------|
| CR Series | CR3, CR5, CR7, CR10, CR12, CR16 |
| ME Series | ME6 |
| Nova Series | Nova2, Nova5 |

---

## Launch File Reference

| Launch File | Package | Description |
|-------------|---------|-------------|
| `bringup.launch` | `dobot_bringup` | V3 Robot Driver |
| `bringup_v4.launch` | `dobot_v4_bringup` | V4 Robot Driver |
| `display.launch` | `dobot_description` | RViz Visualization |
| `demo.launch` | `{model}_moveit` | MoveIt Demo (Virtual Mode) |
| `{model}_moveit.launch` | `{model}_moveit` | MoveIt Control Interface |
| `gazebo.launch` | `dobot_gazebo` | Gazebo Physics Simulation |

---

## Protocol Specification

### Port Functions

#### V3 Version Ports

| Port | Function | Characteristics |
|------|----------|-----------------|
| 29999 | Control Port | Request-response, single client |
| 30003 | Motion Port | Queue commands |
| 30004 | Real-time Feedback | 8ms update interval |

#### V4 Version Ports

| Port | Function | Characteristics |
|------|----------|-----------------|
| 29999 | Control & Motion Port | Unified port, supports multiple command types |
| 30004 | Real-time Feedback | 8ms update interval |

### Robot Status Codes

| Code | Description |
|------|-------------|
| 1 | Initialization |
| 2 | Brake Open |
| 3 | Power-off |
| 4 | Disabled (Brake not released) |
| 5 | Enabled (Idle) |
| 6 | Backdrive |
| 7 | Running (script and TCP queue) |
| 8 | Single Move (Jog) |
| 9 | Error |
| 10 | Pause |
| 11 | Collision |

### Status Priority

```
1. Error status has highest priority
2. Power-off status is second priority
3. Collision status is third priority
4. Brake open status is fourth priority
   Other statuses are reported based on actual conditions.
```

---

## FAQ

### TCP Connection Issues

- **Port Limitations**: Port 29999 supports only single client connection; Port 30004 supports multiple clients
- **Mode Restriction**: Port 29999 requires robot to be in TCP mode, otherwise returns "Control Mode Is Not Tcp"
- **Feedback Ports**: Ports 30004 has no mode restrictions

### Coordinate System

- TCP/IP mode defaults user/tool coordinate system to 0, restores on exit
- **Global Coordinate**: `User()` and `Tool()` commands set global coordinates
- **Local Coordinate**: Motion commands with user/tool parameters only affect current command

### Queue Command Characteristics

- Queue depth is 64, supports 64 concurrent queue commands
- Queue commands return immediately after sending, not after execution
- Check `CommandID` and `RobotMode` to determine execution completion

### Multi-Robot Control

To control multiple robots, modify `dobot_v4_bringup/launch/bringup_v4.launch`:

```xml
<node name="$(arg robotName)$(env DOBOT_TYPE)_robot" pkg="dobot_v4_bringup" type="dobot_v4_bringup" output="screen" >
  <param name="robot_node_name" type="str" value="$(arg robotName)"/>
  <param name="robot_ip_address" type="str" value="$(arg robot_ip)"/>
</node>
```

Launch commands:
```bash
# Connect robot A
roslaunch dobot_v4_bringup bringup_v4.launch robot_ip:=192.168.100.10 robotName:=robotA

# Connect robot B
roslaunch dobot_v4_bringup bringup_v4.launch robot_ip:=192.168.100.20 robotName:=robotB
```

---

## Notes

> ⚠️ **Safety First**: Ensure robot is in safe position before operation

1. Ensure PC IP is in the same subnet as robot (192.168.X.X)
2. Ensure ports 29999, 30003, and 30004 are not occupied
3. Robot must be in TCP/IP control mode
4. Port 29999 only supports single client connection

---

## Version Information

| Item | Content |
|------|---------|
| Current Version | v1.0.0.0 |
| ROS Version | ROS1 Kinetic / Melodic / Noetic |
| Protocol Version | Dobot TCP/IP V3 / V4 |

---

## License

[MIT License](LICENSE)

<div align="center">
Built by Dobot-Arm
</div>
