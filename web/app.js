// MakersPet Loki Control Center - Main Application
// Modern web interface for robot control

// Global state
const state = {
    ros: null,
    connected: false,
    cmdVelTopic: null,
    mapTopic: null,
    tfTopic: null,
    odomTopic: null,
    batteryTopic: null,
    scanTopic: null,
    navAction: null,
    exploreAction: null,
    joystick: null,
    map: null,
    mapData: null,
    mapLayer: null,
    robotMarker: null,
    goalMarker: null,
    robotPose: { x: 0, y: 0, theta: 0 },
    homePosition: { x: 0, y: 0 },
    mapBounds: null,
    services: {
        '/cmd_vel': false,
        '/map': false,
        '/scan': false,
        '/tf': false,
        '/odom': false,
        '/battery_state': false
    },
    nodes: {
        // Container nodes (always running)
        '/slam_toolbox': false,
        '/bt_navigator': false,
        '/controller_server': false,
        '/planner_server': false,
        // Robot nodes (only when robot connected)
        '/kaiaai_telemetry_node': false
    }
};

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initializeUI();
    initializeMap();
    initializeJoystick();
    attachEventListeners();
    logCommand('System initialized. Please connect to ROS Bridge.');
});

// === UI INITIALIZATION ===

function initializeUI() {
    // Set initial button states
    document.getElementById('btnDisconnect').disabled = true;
    document.getElementById('btnConnect').disabled = false;
}

function initializeMap() {
    // Initialize Leaflet map
    state.map = L.map('map', {
        crs: L.CRS.Simple,
        minZoom: -5,
        maxZoom: 10,
        zoomControl: true,
        attributionControl: false
    }).setView([0, 0], 0);

    // Add dark tile layer
    const bounds = [[-10, -10], [10, 10]];
    L.imageOverlay('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==', bounds).addTo(state.map);
    state.map.fitBounds(bounds);

    // Map click handler for navigation
    state.map.on('click', function(e) {
        if (state.mapData) {
            const latlng = e.latlng;
            // Convert Leaflet coordinates to world coordinates
            const worldCoords = leafletToWorld(latlng);

            // Update modal inputs
            document.getElementById('navX').value = worldCoords.x.toFixed(2);
            document.getElementById('navY').value = worldCoords.y.toFixed(2);

            // Show navigation modal
            showNavigationModal();

            logCommand(`Map clicked at (${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)})`);
        }
    });
}

function initializeJoystick() {
    // Initialize nipplejs joystick
    const joystickZone = document.getElementById('joystickZone');

    state.joystick = nipplejs.create({
        zone: joystickZone,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#2563eb',
        size: 150
    });

    // Joystick event handlers
    state.joystick.on('move', (evt, data) => {
        if (!state.connected || !state.cmdVelTopic) return;

        const distance = Math.min(data.distance, 75) / 75; // Normalize to 0-1
        const angle = data.angle.radian;

        // Calculate linear and angular velocities
        const maxLinear = 0.3; // m/s
        const maxAngular = 1.0; // rad/s

        const linear = Math.cos(angle) * distance * maxLinear;
        const angular = -Math.sin(angle) * distance * maxAngular;

        // Publish velocity
        publishVelocity(linear, angular);

        // Update UI
        document.getElementById('joystickLinear').textContent = linear.toFixed(2);
        document.getElementById('joystickAngular').textContent = angular.toFixed(2);
    });

    state.joystick.on('end', () => {
        // Stop robot when joystick is released
        publishVelocity(0, 0);
        document.getElementById('joystickLinear').textContent = '0.00';
        document.getElementById('joystickAngular').textContent = '0.00';
    });
}

// === EVENT LISTENERS ===

function attachEventListeners() {
    // Connection
    document.getElementById('btnConnect').addEventListener('click', connectToROS);
    document.getElementById('btnDisconnect').addEventListener('click', disconnectFromROS);

    // Emergency stop
    document.getElementById('btnStop').addEventListener('click', emergencyStop);

    // Topics
    document.getElementById('btnRefreshTopics').addEventListener('click', refreshTopics);

    // Map commands
    document.getElementById('btnLoadMap').addEventListener('click', loadMap);
    document.getElementById('btnClearMarkers').addEventListener('click', clearMarkers);

    // Navigation commands
    document.getElementById('btnNavigateToPoint').addEventListener('click', showNavigationModal);
    document.getElementById('btnCancelGoal').addEventListener('click', cancelNavigation);
    document.getElementById('btnSetInitialPose').addEventListener('click', setInitialPose);

    // Exploration commands
    document.getElementById('btnStartExplore').addEventListener('click', startExploration);
    document.getElementById('btnStopExplore').addEventListener('click', stopExploration);
    document.getElementById('btnCheckExplore').addEventListener('click', checkExploration);

    // System commands
    document.getElementById('btnReturnHome').addEventListener('click', returnHome);
    document.getElementById('btnSaveMap').addEventListener('click', saveMap);

    // Modal
    document.getElementById('btnCancelNav').addEventListener('click', hideNavigationModal);
    document.getElementById('btnConfirmNav').addEventListener('click', navigateToPoint);
}

// === ROS CONNECTION ===

function connectToROS() {
    const url = document.getElementById('rosUrl').value;

    logCommand('Connecting to ROS Bridge...');
    updateStatusDot('statusROS', 'warning');

    state.ros = new ROSLIB.Ros({ url: url });

    state.ros.on('connection', () => {
        state.connected = true;
        updateStatusDot('statusROS', 'online');
        logCommand('✓ Connected to ROS Bridge');

        document.getElementById('btnConnect').disabled = true;
        document.getElementById('btnDisconnect').disabled = false;

        setupROSTopics();
        checkServices();
    });

    state.ros.on('error', (error) => {
        state.connected = false;
        updateStatusDot('statusROS', 'offline');
        logCommand('✗ Connection error: ' + error);
    });

    state.ros.on('close', () => {
        state.connected = false;
        updateStatusDot('statusROS', 'offline');
        updateStatusDot('statusNav', 'offline');
        updateStatusDot('statusMap', 'offline');
        logCommand('Disconnected from ROS Bridge');

        document.getElementById('btnConnect').disabled = false;
        document.getElementById('btnDisconnect').disabled = true;

        resetServiceStatus();
    });
}

function disconnectFromROS() {
    if (state.ros) {
        state.ros.close();
    }
}

function setupROSTopics() {
    // cmd_vel topic
    state.cmdVelTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });

    // Map topic
    state.mapTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
    });

    // TF topic for robot position
    state.tfTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/tf',
        messageType: 'tf2_msgs/TFMessage'
    });

    state.tfTopic.subscribe((message) => {
        updateRobotPosition(message);
    });

    // Odometry topic
    state.odomTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/odom',
        messageType: 'nav_msgs/Odometry'
    });

    // Battery topic
    state.batteryTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/battery_state',
        messageType: 'sensor_msgs/BatteryState'
    });

    state.batteryTopic.subscribe((message) => {
        const voltage = message.voltage || message.percentage;
        document.getElementById('robotBattery').textContent =
            voltage ? voltage.toFixed(2) + 'V' : 'N/A';
    });

    // Navigation action client
    state.navAction = new ROSLIB.ActionClient({
        ros: state.ros,
        serverName: '/navigate_to_pose',
        actionName: 'nav2_msgs/action/NavigateToPose'
    });

    logCommand('ROS topics initialized');
}

function checkServices() {
    if (!state.ros) return;

    // Set all services to checking state
    Object.keys(state.services).forEach(service => {
        updateServiceStatus(service, 'checking');
    });

    state.ros.getTopics((result) => {
        const topics = result.topics;

        // Check each service with exact match
        Object.keys(state.services).forEach(service => {
            const exists = topics.some(topic => topic === service);
            state.services[service] = exists;
            updateServiceStatus(service, exists ? 'active' : 'inactive');
        });

        // Check if robot hardware is actually connected
        // by verifying if /scan topic is publishing data
        checkRobotHardware();

        // Check nodes status
        checkNodes();

        // Update main status indicators
        const hasNav = state.services['/cmd_vel'] && state.services['/odom'];
        const hasMap = state.services['/map'] && state.services['/scan'];

        updateStatusDot('statusNav', hasNav ? 'online' : 'offline');
        updateStatusDot('statusMap', hasMap ? 'online' : 'offline');

        logCommand('Services checked: ' +
            Object.values(state.services).filter(v => v).length +
            '/' + Object.keys(state.services).length + ' active');
    }, (error) => {
        logCommand('Error checking services: ' + error);
        // Reset all to inactive on error
        Object.keys(state.services).forEach(service => {
            state.services[service] = false;
            updateServiceStatus(service, 'inactive');
        });
        updateStatusDot('statusNav', 'offline');
        updateStatusDot('statusMap', 'offline');
    });
}

function checkRobotHardware() {
    // Check if robot hardware is connected by listening to /scan topic
    const scanListener = new ROSLIB.Topic({
        ros: state.ros,
        name: '/scan',
        messageType: 'sensor_msgs/LaserScan'
    });

    let messageReceived = false;
    const timeout = setTimeout(() => {
        if (!messageReceived) {
            logCommand('⚠ Warning: Robot hardware appears offline (no /scan data)');
            // Mark scan as warning since topic exists but no data
            const serviceItems = document.querySelectorAll('.service-item');
            serviceItems.forEach(item => {
                if (item.textContent.includes('/scan')) {
                    const dot = item.querySelector('.service-status-dot');
                    dot.className = 'service-status-dot checking'; // Orange for warning
                }
            });
        }
        scanListener.unsubscribe();
    }, 3000); // Wait 3 seconds for data

    scanListener.subscribe(() => {
        messageReceived = true;
        clearTimeout(timeout);
        logCommand('✓ Robot hardware connected (receiving /scan data)');
        scanListener.unsubscribe();
    });
}

function checkNodes() {
    if (!state.ros) return;

    state.ros.getNodes((nodes) => {
        // Check each node
        Object.keys(state.nodes).forEach(nodeName => {
            const exists = nodes.includes(nodeName);
            state.nodes[nodeName] = exists;
            updateNodeStatus(nodeName, exists ? 'active' : 'inactive');
        });

        logCommand('Nodes checked: ' +
            Object.values(state.nodes).filter(v => v).length +
            '/' + Object.keys(state.nodes).length + ' active');
    }, (error) => {
        logCommand('Error checking nodes: ' + error);
    });
}

function refreshTopics() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    state.ros.getTopics((result) => {
        const topicsList = document.getElementById('topicsList');
        topicsList.innerHTML = '';

        // Filter important topics
        const importantKeywords = ['cmd_vel', 'map', 'scan', 'battery', 'tf', 'odom', 'nav', 'explore'];
        const filteredTopics = result.topics.filter(topic =>
            importantKeywords.some(keyword => topic.includes(keyword))
        );

        if (filteredTopics.length === 0) {
            topicsList.innerHTML = '<div style="color: var(--text-gray); text-align: center;">No topics found</div>';
        } else {
            filteredTopics.forEach(topic => {
                const div = document.createElement('div');
                div.className = 'topic-item';
                div.textContent = topic;
                topicsList.appendChild(div);
            });
        }

        logCommand(`Refreshed topics: ${filteredTopics.length} found`);
    });
}

// === ROBOT CONTROL ===

function publishVelocity(linear, angular) {
    if (!state.cmdVelTopic) return;

    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
    });

    state.cmdVelTopic.publish(twist);
}

function emergencyStop() {
    publishVelocity(0, 0);
    logCommand('⚠ EMERGENCY STOP');
}

function updateRobotPosition(message) {
    // Find base_link transform
    for (let i = 0; i < message.transforms.length; i++) {
        const transform = message.transforms[i];
        if (transform.header.frame_id === 'map' &&
            transform.child_frame_id === 'base_link') {

            state.robotPose.x = transform.transform.translation.x;
            state.robotPose.y = transform.transform.translation.y;

            // Extract yaw from quaternion
            const q = transform.transform.rotation;
            state.robotPose.theta = Math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            );

            // Update UI
            document.getElementById('robotX').textContent = state.robotPose.x.toFixed(2) + ' m';
            document.getElementById('robotY').textContent = state.robotPose.y.toFixed(2) + ' m';
            document.getElementById('robotTheta').textContent = state.robotPose.theta.toFixed(2) + ' rad';

            // Update robot marker on map
            updateRobotMarker();
        }
    }
}

// === MAP HANDLING ===

function loadMap() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    logCommand('Loading map...');

    state.mapTopic.subscribe((message) => {
        state.mapData = {
            width: message.info.width,
            height: message.info.height,
            resolution: message.info.resolution,
            origin: {
                x: message.info.origin.position.x,
                y: message.info.origin.position.y
            },
            data: message.data
        };

        renderMapOnLeaflet();

        logCommand(`✓ Map loaded: ${state.mapData.width}x${state.mapData.height} @ ${state.mapData.resolution}m/px`);

        // Unsubscribe after first message
        state.mapTopic.unsubscribe();
    });
}

function renderMapOnLeaflet() {
    if (!state.mapData) return;

    // Create canvas for map
    const canvas = document.createElement('canvas');
    canvas.width = state.mapData.width;
    canvas.height = state.mapData.height;
    const ctx = canvas.getContext('2d');

    // Draw occupancy grid
    const imageData = ctx.createImageData(state.mapData.width, state.mapData.height);

    for (let y = 0; y < state.mapData.height; y++) {
        for (let x = 0; x < state.mapData.width; x++) {
            const index = y * state.mapData.width + x;
            const value = state.mapData.data[index];

            const pixelIndex = ((state.mapData.height - 1 - y) * state.mapData.width + x) * 4;

            if (value === -1) {
                // Unknown - gray
                imageData.data[pixelIndex] = 128;
                imageData.data[pixelIndex + 1] = 128;
                imageData.data[pixelIndex + 2] = 128;
                imageData.data[pixelIndex + 3] = 255;
            } else if (value === 0) {
                // Free space - white
                imageData.data[pixelIndex] = 255;
                imageData.data[pixelIndex + 1] = 255;
                imageData.data[pixelIndex + 2] = 255;
                imageData.data[pixelIndex + 3] = 255;
            } else {
                // Occupied - black
                imageData.data[pixelIndex] = 0;
                imageData.data[pixelIndex + 1] = 0;
                imageData.data[pixelIndex + 2] = 0;
                imageData.data[pixelIndex + 3] = 255;
            }
        }
    }

    ctx.putImageData(imageData, 0, 0);

    // Convert canvas to image URL
    const imageUrl = canvas.toDataURL();

    // Calculate map bounds in world coordinates
    const worldWidth = state.mapData.width * state.mapData.resolution;
    const worldHeight = state.mapData.height * state.mapData.resolution;

    const minX = state.mapData.origin.x;
    const minY = state.mapData.origin.y;
    const maxX = minX + worldWidth;
    const maxY = minY + worldHeight;

    // Convert to Leaflet coordinates
    const bounds = [
        worldToLeaflet(minX, minY),
        worldToLeaflet(maxX, maxY)
    ];

    state.mapBounds = { minX, minY, maxX, maxY };

    // Remove old layer
    if (state.mapLayer) {
        state.map.removeLayer(state.mapLayer);
    }

    // Add new map layer
    state.mapLayer = L.imageOverlay(imageUrl, bounds).addTo(state.map);
    state.map.fitBounds(bounds);

    // Update robot marker
    updateRobotMarker();
}

function updateRobotMarker() {
    if (!state.map || !state.mapData) return;

    const latlng = worldToLeaflet(state.robotPose.x, state.robotPose.y);

    if (!state.robotMarker) {
        // Create robot marker with custom icon
        const robotIcon = L.divIcon({
            className: 'robot-marker',
            html: '<div style="background: #2563eb; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
            iconSize: [20, 20],
            iconAnchor: [10, 10]
        });

        state.robotMarker = L.marker(latlng, { icon: robotIcon }).addTo(state.map);
    } else {
        state.robotMarker.setLatLng(latlng);
    }
}

function clearMarkers() {
    if (state.goalMarker) {
        state.map.removeLayer(state.goalMarker);
        state.goalMarker = null;
    }
    logCommand('Markers cleared');
}

// === NAVIGATION ===

function showNavigationModal() {
    document.getElementById('navModal').classList.add('active');
}

function hideNavigationModal() {
    document.getElementById('navModal').classList.remove('active');
}

function navigateToPoint() {
    const x = parseFloat(document.getElementById('navX').value);
    const y = parseFloat(document.getElementById('navY').value);

    hideNavigationModal();

    if (!state.navAction) {
        logCommand('Navigation not available');
        return;
    }

    logCommand(`Navigating to (${x.toFixed(2)}, ${y.toFixed(2)})...`);

    // Create goal marker
    const latlng = worldToLeaflet(x, y);
    if (state.goalMarker) {
        state.map.removeLayer(state.goalMarker);
    }

    const goalIcon = L.divIcon({
        className: 'goal-marker',
        html: '<div style="background: #ef4444; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });

    state.goalMarker = L.marker(latlng, { icon: goalIcon }).addTo(state.map);

    // Send navigation goal
    const goal = new ROSLIB.Goal({
        actionClient: state.navAction,
        goalMessage: {
            pose: {
                header: { frame_id: 'map' },
                pose: {
                    position: { x: x, y: y, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 }
                }
            }
        }
    });

    goal.on('feedback', (feedback) => {
        // Could add progress display here
    });

    goal.on('result', (result) => {
        logCommand('✓ Navigation completed');
    });

    goal.on('error', (error) => {
        logCommand('✗ Navigation error: ' + error);
    });

    goal.send();
}

function cancelNavigation() {
    if (state.navAction) {
        state.navAction.cancel();
        logCommand('Navigation cancelled');
        clearMarkers();
    }
}

function setInitialPose() {
    logCommand('Click on map to set initial pose');
    // This would typically publish to /initialpose topic
    // Implementation depends on Nav2 setup
}

function returnHome() {
    if (state.homePosition) {
        document.getElementById('navX').value = state.homePosition.x;
        document.getElementById('navY').value = state.homePosition.y;
        navigateToPoint();
        logCommand('Returning to home position');
    } else {
        // Set current position as home
        state.homePosition = { x: state.robotPose.x, y: state.robotPose.y };
        logCommand('Home position set to current location');
    }
}

// === EXPLORATION ===

function startExploration() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    // Check if explore_lite is running
    state.ros.getNodes((nodes) => {
        const exploreNode = nodes.find(n => n.includes('explore'));
        if (exploreNode) {
            logCommand('✓ Exploration node found: ' + exploreNode);
            logCommand('Starting autonomous exploration...');
            // explore_lite typically starts automatically when launched
        } else {
            logCommand('✗ Exploration node not found. Is explore_lite running?');
        }
    });
}

function stopExploration() {
    // Cancel any navigation goals
    cancelNavigation();
    logCommand('Stopping exploration');
}

function checkExploration() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    state.ros.getNodes((nodes) => {
        const exploreNodes = nodes.filter(n => n.includes('explore'));
        if (exploreNodes.length > 0) {
            logCommand('✓ Exploration nodes: ' + exploreNodes.join(', '));
        } else {
            logCommand('✗ No exploration nodes found');
        }
    });
}

// === SYSTEM ===

function saveMap() {
    logCommand('Map saving not implemented yet. Use: ros2 run nav2_map_server map_saver_cli');
}

// === UTILITY FUNCTIONS ===

function worldToLeaflet(x, y) {
    // Convert ROS world coordinates to Leaflet coordinates
    // Leaflet uses [lat, lng] which maps to [y, x] in our case
    return [y, x];
}

function leafletToWorld(latlng) {
    // Convert Leaflet coordinates to ROS world coordinates
    return { x: latlng.lng, y: latlng.lat };
}

function updateStatusDot(elementId, status) {
    const dot = document.getElementById(elementId);
    dot.className = 'status-dot ' + status;
}

function updateServiceStatus(serviceName, status) {
    // Find the service item in the grid
    const serviceItems = document.querySelectorAll('.service-item');
    serviceItems.forEach(item => {
        if (item.textContent.includes(serviceName)) {
            const dot = item.querySelector('.service-status-dot');
            // status can be 'active', 'inactive', or 'checking'
            if (typeof status === 'boolean') {
                dot.className = 'service-status-dot ' + (status ? 'active' : 'inactive');
            } else {
                dot.className = 'service-status-dot ' + status;
            }
        }
    });
}

function updateNodeStatus(nodeName, status) {
    // Find the node item in the grid
    const nodeItems = document.querySelectorAll('.node-item');
    nodeItems.forEach(item => {
        if (item.textContent.includes(nodeName)) {
            const dot = item.querySelector('.service-status-dot');
            if (typeof status === 'boolean') {
                dot.className = 'service-status-dot ' + (status ? 'active' : 'inactive');
            } else {
                dot.className = 'service-status-dot ' + status;
            }
        }
    });
}

function resetServiceStatus() {
    Object.keys(state.services).forEach(service => {
        state.services[service] = false;
        updateServiceStatus(service, false);
    });
}

function logCommand(message) {
    const log = document.getElementById('commandLog');
    const timestamp = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = 'topic-item';
    entry.textContent = `[${timestamp}] ${message}`;

    log.insertBefore(entry, log.firstChild);

    // Keep only last 50 entries
    while (log.children.length > 50) {
        log.removeChild(log.lastChild);
    }
}

// Export for debugging
window.robotState = state;
window.robotControl = {
    connect: connectToROS,
    disconnect: disconnectFromROS,
    move: publishVelocity,
    stop: emergencyStop,
    loadMap: loadMap,
    navigate: navigateToPoint
};

console.log('MakersPet Loki Control Center initialized');
console.log('Access robot state: window.robotState');
console.log('Access robot control: window.robotControl');
