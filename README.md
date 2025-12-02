# Autonomous LiDAR Robot

**ROS2 Iron autonomous navigation system with SLAM, Nav2, and web interface for LiDAR-equipped robots**

## Features

- ✅ **Autonomous Navigation**: Full Nav2 stack with path planning and obstacle avoidance
- ✅ **Real-time SLAM**: SLAM Toolbox for simultaneous localization and mapping
- ✅ **Web Interface**: Control and monitor your robot from any browser via RosBridge WebSocket
- ✅ **Autonomous Exploration**: Built-in explore_lite for automatic environment mapping
- ✅ **LiDAR Support**: LDROBOT-LD14P (and other models via configuration)
- ✅ **micro-ROS**: ESP32 communication over UDP
- ✅ **Dockerized**: Complete system in a single container (Podman/Docker)

## Quick Start

### Prerequisites

**Hardware:**
- LiDAR-equipped robot (tested with LDROBOT-LD14P)
- ESP32 with micro-ROS firmware
- Linux server (Ubuntu/Debian recommended)

**Software:**
- Docker or Podman
- docker-compose or podman-compose

### 1. Clone the Repository

```bash
git clone https://github.com/YOUR_USERNAME/autonomous-lidar-robot.git
cd autonomous-lidar-robot
```

### 2. Configure Your LiDAR Model

Edit `config/telem.yaml` if you're not using LDROBOT-LD14P:

```yaml
kaiaai_telemetry_node:
  ros__parameters:
    laser_scan:
      lidar_model: "LDROBOT-LD14P"  # Change if needed
```

Supported models: `YDLIDAR-X4`, `XIAOMI-LDS02RR`, `LDROBOT-LD14P`, etc.

### 3. Start the System

```bash
# Power on your robot first!
podman-compose up -d

# Monitor logs
podman logs -f autonomous-lidar-robot
```

The system will:
1. Start micro-ROS agent (port 8888)
2. Start telemetry node
3. **Wait for robot connection** (up to 120s)
4. Start Robot State Publisher
5. Start SLAM + Nav2 + RosBridge
6. **Automatically activate Nav2**

### 4. Access Web Interface

Open your browser:
```
http://<SERVER_IP>:8082/index.html
```

Click "CONNECTER" to connect to RosBridge WebSocket (ws://<SERVER_IP>:9092)

## Project Structure

```
autonomous-lidar-robot/
├── Dockerfile              # ROS2 Iron Docker image
├── podman-compose.yml      # Container orchestration
├── docker-compose.yml      # Docker alternative
│
├── config/
│   ├── telem.yaml          # Telemetry + LiDAR config
│   └── navigation.yaml     # Nav2 + SLAM Toolbox config
│
├── launch/
│   └── robot_web.launch.py # Main launch file
│
├── scripts/
│   └── entrypoint.sh       # Container entrypoint with robot auto-detection
│
├── web/
│   ├── index.html          # Web control interface
│   └── serve.py            # HTTP server
│
├── maps/                   # Saved maps
├── logs/                   # Runtime logs
└── docs/                   # Documentation
```

## Configuration

### Ports

- **8082**: Web interface HTTP
- **8888**: micro-ROS agent UDP
- **9092**: RosBridge WebSocket

### Nav2 Parameters

All Nav2 and SLAM parameters are in `config/navigation.yaml`:
- Costmaps (global, local)
- Planners (DWB, Smac)
- Controllers
- SLAM Toolbox (solver, scan matcher)

## Autonomous Exploration

Launch autonomous exploration to automatically map your environment:

```bash
podman exec autonomous-lidar-robot bash -c '
source /opt/ros/iron/setup.bash && \
source /app/ros_ws/install/setup.bash && \
ros2 run explore_lite explore --ros-args \
  -p robot_base_frame:=base_footprint \
  -p costmap_topic:=/global_costmap/costmap \
  -p costmap_updates_topic:=/global_costmap/costmap_updates \
  -p visualize:=true \
  -p planner_frequency:=0.33 \
  -p min_frontier_size:=0.5
'
```

## Troubleshooting

### Robot Not Connecting

```bash
# Check ESP32 is on WiFi and reachable
ping <ROBOT_IP>

# Check micro-ROS agent logs
tail -f logs/micro_ros_agent.log | grep "session established"

# Restart robot physically
```

### Nav2 Inactive

```bash
# Enter container
podman exec -it autonomous-lidar-robot bash
source /opt/ros/iron/setup.bash
source /app/ros_ws/install/setup.bash

# Check lifecycle state
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server

# Manually activate if needed
ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
```

### /scan Not Publishing

```bash
# Check LiDAR detection
tail logs/telemetry.log | grep "LDS model"
# Should show: LDS model LDROBOT-LD14P

# Check scan rate
ros2 topic hz /scan
# Should show ~5 Hz
```

## Useful Commands

### Diagnostics

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check transforms
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

# Topic frequencies
ros2 topic hz /scan
ros2 topic hz /map
```

### Robot Control

```bash
# Stop robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Rotate in place
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

### Container Management

```bash
# Start
podman-compose up -d

# Stop
podman-compose down

# Restart
podman-compose restart

# Live logs
podman logs -f autonomous-lidar-robot

# Shell in container
podman exec -it autonomous-lidar-robot bash
```

### Rebuild Image

```bash
# After modifications
podman build -t autonomous-lidar-robot:latest .
podman-compose down
podman-compose up -d
```

## Architecture

### ROS2 Nodes

- `/kaiaai_telemetry_node` - LiDAR data processing → /scan
- `/robot_state_publisher` - URDF → TF tree
- `/slam_toolbox` - /scan → map + TF map→odom
- `/planner_server` - Nav2 global path planner
- `/controller_server` - Nav2 local trajectory controller
- `/bt_navigator` - Nav2 behavior tree executor
- `/rosbridge_websocket` - Web interface bridge (port 9092)

### Data Flow

```
ESP32 (micro-ROS) → UDP:8888 → micro-ROS agent
    ↓
/telemetry (kaiaai_msgs/Telemetry)
    ↓
kaiaai_telemetry_node → /scan (sensor_msgs/LaserScan)
    ↓
SLAM Toolbox → /map (nav_msgs/OccupancyGrid) + TF map→odom
    ↓
Nav2 Stack → /cmd_vel (geometry_msgs/Twist) → ESP32
```

## License

Apache License 2.0

## Acknowledgments

- **ROS2 Iron** - Robotics framework
- **Nav2** - Navigation stack
- **SLAM Toolbox** - SLAM implementation
- **RosBridge** - Web ↔ ROS2 bridge
- **KaiAI** - kaiaai_telemetry and kaiaai_msgs packages
- **m-explore-ros2** - Autonomous exploration (KaiAI fork)

## Support

For issues and questions:
- GitHub Issues: [https://github.com/YOUR_USERNAME/autonomous-lidar-robot/issues](https://github.com/YOUR_USERNAME/autonomous-lidar-robot/issues)

---

**Version**: 1.0.0  
**ROS Distro**: Iron  
**Tested on**: Ubuntu 22.04, Podman 4.x
