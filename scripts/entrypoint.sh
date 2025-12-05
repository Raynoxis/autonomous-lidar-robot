#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source /app/ros_ws/install/setup.bash
source /app/uros_ws/install/setup.bash

echo "==================================="
echo "MakersPet Mini - Web Navigation"
echo "==================================="
echo "ROS_DISTRO: ${ROS_DISTRO}"
echo "Workspace: /app/ros_ws"
echo ""

# Fonction améliorée pour attendre la connexion ESP32 et les scans
wait_for_robot() {
    local max_wait=120
    local count=0
    echo "Waiting for robot connection (ESP32 + LiDAR data)..."
    
    while [ $count -lt $max_wait ]; do
        # Vérifier que /scan existe ET publie des données
        if ros2 topic list 2>/dev/null | grep -q "/scan"; then
            echo "✓ /scan topic detected"
            # Attendre 3 secondes que des données arrivent
            sleep 3
            local scan_rate=$(timeout 3 ros2 topic hz /scan 2>/dev/null | grep "average rate" | head -1 || echo "")
            if [ -n "$scan_rate" ]; then
                echo "✓ Robot fully connected! $scan_rate"
                return 0
            else
                echo "  Waiting for scan data..."
            fi
        fi
        sleep 2
        count=$((count + 2))
        if [ $((count % 10)) -eq 0 ]; then
            echo "  Still waiting... (${count}s / ${max_wait}s)"
        fi
    done
    
    echo ""
    echo "WARNING: Robot not detected after ${max_wait}s"
    echo "Please check:"
    echo "  1. Robot is powered on"
    echo "  2. Robot is connected to WiFi"
    echo "  3. ESP32 can reach micro-ROS agent (port 8888)"
    echo ""
    echo "Container will continue running. Robot can connect later."
    return 0
}

# Attendre la TF odom -> base_footprint (télémétrie opérationnelle)
wait_for_tf() {
    local max_wait=60
    local count=0
    echo "Waiting for TF odom -> base_footprint..."

    while [ $count -lt $max_wait ]; do
        if timeout 3 ros2 topic echo /odom --once >/dev/null 2>&1; then
            if timeout 3 bash -c "ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -n 1 | grep -q 'At time'"; then
                echo "✓ TF odom -> base_footprint available"
                return 0
            fi
        fi
        sleep 2
        count=$((count + 2))
        echo "  Waiting TF... (${count}s / ${max_wait}s)"
    done

    echo "WARNING: TF odom -> base_footprint not detected after ${max_wait}s, continuing anyway"
    return 0
}

# Fonction pour activer Nav2
activate_nav2() {
    echo ""
    echo "Activating Nav2 stack..."
    sleep 10  # Attendre que tous les nodes soient prêts

    # Vérifier que les lifecycle nodes Nav2 existent
    echo "  Checking for Nav2 nodes..."
    local max_wait=30
    local count=0
    while [ $count -lt $max_wait ]; do
        local node_count=$(ros2 node list 2>/dev/null | grep -E "(bt_navigator|controller_server|planner_server)" | wc -l)
        if [ "$node_count" -ge 3 ]; then
            echo "✓ Nav2 nodes detected"
            break
        fi
        sleep 2
        count=$((count + 2))
        if [ $((count % 10)) -eq 0 ]; then
            echo "  Waiting for Nav2 nodes... (${count}s)"
        fi
    done

    if [ "$node_count" -lt 3 ]; then
        echo "⚠ Nav2 nodes not found - skipping activation"
        return 1
    fi

    # Vérifier l'état actuel de bt_navigator
    local nav_state_raw
    nav_state_raw=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1 || true)
    local nav_state="${nav_state_raw:-unknown (timeout)}"
    echo "  Current bt_navigator state: $nav_state"

    if echo "$nav_state" | grep -q "active"; then
        echo "✓ Nav2 already active!"
        return 0
    fi

    # Activer tous les lifecycle nodes Nav2
    echo "  Activating lifecycle nodes..."
    local nodes=("/bt_navigator" "/controller_server" "/planner_server" "/smoother_server" "/waypoint_follower" "/velocity_smoother")

    for node in "${nodes[@]}"; do
        # Vérifier si le node existe
        if ros2 node list 2>/dev/null | grep -q "^${node}$"; then
            echo "    Activating ${node}..."
            timeout 5 ros2 lifecycle set ${node} activate 2>/dev/null
            if [ $? -eq 0 ]; then
                echo "    ✓ ${node} activated"
            else
                echo "    ⚠ ${node} activation failed (may already be active)"
            fi
        fi
    done

    # Vérifier que l'activation a réussi
    sleep 2
    nav_state_raw=$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | head -1 || true)
    nav_state="${nav_state_raw:-unknown (timeout)}"
    if echo "$nav_state" | grep -q "active"; then
        echo "✓ Nav2 fully activated!"
        return 0
    else
        echo "⚠ Nav2 activation incomplete (state: $nav_state)"
        return 1
    fi
}

# 0. Démarrage du serveur web
echo "Starting Web Server on port 8082..."
python3 /app/web/serve.py > /app/logs/webserver.log 2>&1 &
sleep 1

# 1. Démarrage Micro-ROS Agent
echo "Starting Micro-ROS Agent on port 8888..."
/app/uros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 8888 > /app/logs/micro_ros_agent.log 2>&1 &
sleep 3

# 2. Démarrage télémétrie
echo "Starting Telemetry Node..."
ros2 run kaiaai_telemetry telem \
    --ros-args \
    --params-file /app/config/telem.yaml \
    > /app/logs/telemetry.log 2>&1 &
sleep 5

# 3. Robot State Publisher avec fichier URDF Mini
echo "Starting Robot State Publisher (Mini)..."
xacro /app/ros_ws/src/makerspet_mini/urdf/robot.urdf.xacro > /tmp/robot.urdf 2>/dev/null

cat > /tmp/robot_state_params.yaml << 'YAML_EOF'
robot_state_publisher:
  ros__parameters:
    use_sim_time: false
YAML_EOF

ros2 run robot_state_publisher robot_state_publisher \
    /tmp/robot.urdf \
    --ros-args \
    --params-file /tmp/robot_state_params.yaml \
    > /app/logs/robot_state_publisher.log 2>&1 &
sleep 2

# 4. Lancer RosBridge immédiatement pour permettre la connexion WS même sans Nav2
echo "Starting RosBridge WebSocket on port 9092..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9092 > /app/logs/rosbridge.log 2>&1 &
sleep 2

# 5. Attendre le robot
wait_for_robot

# 6. Attendre la TF odom -> base_footprint pour éviter l'échec du costmap
wait_for_tf

# 7. Navigation + SLAM (après TF prête)
echo "Starting Nav2 + SLAM..."
ros2 launch /app/launch/robot_web.launch.py \
    use_sim_time:=false \
    slam:=True \
    params_file:=/app/config/navigation.yaml \
    > /app/logs/navigation.log 2>&1 &

sleep 10

# 8. Activer Nav2
activate_nav2

echo ""
echo "==================================="
echo "✓ System Ready!"
echo "==================================="
echo "Web Interface: http://<HOST>:8082/index.html"
echo "RosBridge WebSocket: ws://<HOST>:9092"
echo "ROS2 Control API: http://<HOST>:8083"
echo ""
echo "Active topics:"
ros2 topic list 2>/dev/null | grep -E "/scan|/map|/odom|/cmd_vel" || echo "  (listing...)"
echo ""
echo "Active nodes: $(ros2 node list 2>/dev/null | wc -l)"
echo ""

# 9. Start ROS2 Control API
echo "Starting ROS2 Control API on port 8083..."
python3 /app/web/ros_api.py > /app/logs/ros_api.log 2>&1 &

echo "Press Ctrl+C to stop"
echo ""

# Garder actif
tail -f /app/logs/*.log
