// MakersPet Loki Control Center - Main Application
// Modern web interface for robot control

// System States
const SystemState = {
    INITIAL: 'initial',
    CONNECTING_WS: 'connecting_ws',
    WS_CONNECTED: 'ws_connected',
    CONTAINER_READY: 'container_ready',
    ROBOT_READY: 'robot_ready',
    EXPLORATION_AVAILABLE: 'exploration_available',
    EXPLORING: 'exploring',
    NAVIGATING: 'navigating',
    WS_ERROR: 'ws_error',
    CONTAINER_ERROR: 'container_error',
    ROBOT_LOST: 'robot_lost',
    EXPLORATION_ERROR: 'exploration_error'
};

// Global state
const state = {
    // System state
    systemState: SystemState.INITIAL,

    // ROS connection
    ros: null,
    connected: false,

    // Topics
    cmdVelTopic: null,
    mapTopic: null,
    tfTopic: null,
    odomTopic: null,
    batteryTopic: null,
    scanTopic: null,

    // Actions
    navAction: null,
    exploreAction: null,

    // UI
    joystick: null,
    map: null,
    mapData: null,
    mapLayer: null,
    robotMarker: null,
    goalMarker: null,

    // Robot data
    robotPose: { x: 0, y: 0, theta: 0 },
    homePosition: { x: 0, y: 0 },
    mapBounds: null,
    scanDataReceived: false,
    batteryDataReceived: false,

    // Monitoring intervals
    topicsCheckInterval: null,
    nodesCheckInterval: null,
    hardwareCheckInterval: null,
    statusRefreshInterval: null,

    // Topics status
    topics: {
        // Container topics
        '/map': false,
        '/tf': false,
        '/odom': false,
        '/cmd_vel': false,
        // Robot topics
        '/scan': false,
        '/battery_state': false
    },

    // Nodes status
    nodes: {
        // Container nodes
        '/slam_toolbox': false,
        '/bt_navigator': false,
        '/controller_server': false,
        '/planner_server': false,
        '/behavior_server': false,
        '/velocity_smoother': false,
        '/lifecycle_manager_navigation': false,
        '/lifecycle_manager_slam': false,
        '/rosbridge_websocket': false,
        // Robot nodes
        '/kaiaai_telemetry_node': false,
        '/robot_state_publisher': false,
        // Exploration nodes
        '/explore_node': false
    }
};

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    initializeUI();
    initializeMap();
    initializeJoystick();
    attachEventListeners();
    transitionToState(SystemState.INITIAL);
    logCommand('System initialized. Click Connect to start.');
});

// === STATE MACHINE ===

function transitionToState(newState) {
    state.systemState = newState;

    switch (newState) {
        case SystemState.INITIAL:
            updateMainStatus('waiting', 'Système initialisé - Cliquez sur Connect');
            resetAllStatus();
            break;

        case SystemState.CONNECTING_WS:
            updateMainStatus('connecting', 'Connexion au ROS Bridge WebSocket...');
            break;

        case SystemState.WS_CONNECTED:
            updateMainStatus('connected', 'ROS Bridge OK - Vérification conteneur...');
            startMonitoring();
            break;

        case SystemState.CONTAINER_READY:
            updateMainStatus('connected', 'Conteneur opérationnel - En attente du robot...');
            break;

        case SystemState.ROBOT_READY:
            updateMainStatus('connected', 'Robot connecté - Tous systèmes opérationnels');
            // Auto-start map subscription
            if (!state.mapTopic._subscribeId) {
                loadMap();
            }
            break;

        case SystemState.EXPLORATION_AVAILABLE:
            updateMainStatus('connected', 'Prêt pour exploration autonome');
            break;

        case SystemState.EXPLORING:
            updateMainStatus('connected', 'Exploration autonome en cours...');
            break;

        case SystemState.NAVIGATING:
            updateMainStatus('connected', 'Navigation vers objectif...');
            break;

        case SystemState.WS_ERROR:
            updateMainStatus('error', 'Erreur ROS Bridge - Vérifiez le serveur');
            stopMonitoring();
            break;

        case SystemState.CONTAINER_ERROR:
            updateMainStatus('error', 'Erreur conteneur - Nodes manquants');
            break;

        case SystemState.ROBOT_LOST:
            updateMainStatus('error', 'Robot déconnecté - Vérifiez alimentation');
            break;

        case SystemState.EXPLORATION_ERROR:
            updateMainStatus('error', 'Erreur exploration - Node arrêté');
            break;
    }

    logCommand(`State: ${newState}`);
}

function evaluateSystemState() {
    // Determine current system state based on topics and nodes

    if (!state.connected) {
        if (state.systemState !== SystemState.INITIAL) {
            transitionToState(SystemState.WS_ERROR);
        }
        return;
    }

    // Check container nodes (essential ones only)
    const containerNodes = [
        '/slam_toolbox', '/bt_navigator', '/controller_server', '/planner_server'
    ];
    const containerNodesOK = containerNodes.every(n => state.nodes[n]);

    if (!containerNodesOK) {
        if (state.systemState !== SystemState.CONTAINER_ERROR) {
            transitionToState(SystemState.CONTAINER_ERROR);
        }
        return;
    }

    // Container is ready
    if (state.systemState === SystemState.WS_CONNECTED) {
        transitionToState(SystemState.CONTAINER_READY);
    }

    // Check robot data (not just topics - topics exist even when robot is off)
    const robotDataOK = state.scanDataReceived && state.batteryDataReceived;

    if (robotDataOK) {
        const robotNodeOK = state.nodes['/kaiaai_telemetry_node'];

        if (robotNodeOK) {
            if (state.systemState !== SystemState.ROBOT_READY &&
                state.systemState !== SystemState.EXPLORATION_AVAILABLE &&
                state.systemState !== SystemState.EXPLORING &&
                state.systemState !== SystemState.NAVIGATING) {
                transitionToState(SystemState.ROBOT_READY);
            }

            // Check if map is loaded for exploration
            if (state.mapData && state.systemState === SystemState.ROBOT_READY) {
                transitionToState(SystemState.EXPLORATION_AVAILABLE);
            }

            // Check exploration state
            if (state.nodes['/explore_node']) {
                if (state.systemState !== SystemState.EXPLORING) {
                    transitionToState(SystemState.EXPLORING);
                }
            }
        } else {
            if (state.systemState === SystemState.ROBOT_READY ||
                state.systemState === SystemState.EXPLORATION_AVAILABLE ||
                state.systemState === SystemState.EXPLORING) {
                transitionToState(SystemState.ROBOT_LOST);
            }
        }
        return;
    }

    // No robot detected
    if (state.systemState === SystemState.ROBOT_READY ||
        state.systemState === SystemState.EXPLORATION_AVAILABLE ||
        state.systemState === SystemState.EXPLORING) {
        transitionToState(SystemState.ROBOT_LOST);
    }
}

function resetAllStatus() {
    // Reset all topics
    Object.keys(state.topics).forEach(topic => {
        state.topics[topic] = false;
        updateServiceStatus(topic, 'default');
    });

    // Reset all nodes
    Object.keys(state.nodes).forEach(node => {
        state.nodes[node] = false;
        updateNodeStatus(node, 'default');
    });

    state.scanDataReceived = false;
    state.batteryDataReceived = false;
    updateMiniStatus();
}

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

    // Add neutral gray background
    const bounds = [[-10, -10], [10, 10]];
    // Gray pixel instead of green
    L.imageOverlay('data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mM0NDT8DwADHgEz8mY6qgAAAABJRU5ErkJggg==', bounds).addTo(state.map);
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
        size: 150,
        restJoystick: true,
        restOpacity: 0.5
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

// === MONITORING ===

function startMonitoring() {
    logCommand('Starting real-time monitoring...');

    // Topics check: 100ms (real-time)
    state.topicsCheckInterval = setInterval(() => {
        if (state.connected) {
            checkTopics();
        }
    }, 100);

    // Nodes check: 500ms
    state.nodesCheckInterval = setInterval(() => {
        if (state.connected) {
            checkNodes();
        }
    }, 500);

    // Hardware data check: 1s
    state.hardwareCheckInterval = setInterval(() => {
        if (state.connected) {
            checkHardwareData();
        }
    }, 1000);

    // State evaluation: 200ms
    state.statusRefreshInterval = setInterval(() => {
        if (state.connected) {
            evaluateSystemState();
            updateMiniStatus();
            updateHighLevelStatus();
        }
    }, 200);
}

function stopMonitoring() {
    if (state.topicsCheckInterval) {
        clearInterval(state.topicsCheckInterval);
        state.topicsCheckInterval = null;
    }
    if (state.nodesCheckInterval) {
        clearInterval(state.nodesCheckInterval);
        state.nodesCheckInterval = null;
    }
    if (state.hardwareCheckInterval) {
        clearInterval(state.hardwareCheckInterval);
        state.hardwareCheckInterval = null;
    }
    if (state.statusRefreshInterval) {
        clearInterval(state.statusRefreshInterval);
        state.statusRefreshInterval = null;
    }
    logCommand('Monitoring stopped');
}

function checkTopics() {
    if (!state.ros) return;

    state.ros.getTopics((result) => {
        const topics = result.topics;

        Object.keys(state.topics).forEach(topic => {
            const exists = topics.some(t => t === topic);
            const wasActive = state.topics[topic];
            state.topics[topic] = exists;

            // Determine status
            let status = 'default';
            if (exists) {
                // Topic exists
                if (topic === '/scan' || topic === '/battery_state') {
                    // Robot topics - check if data is actually received
                    if (topic === '/scan') {
                        status = state.scanDataReceived ? 'active' : 'default';
                    } else if (topic === '/battery_state') {
                        status = state.batteryDataReceived ? 'active' : 'default';
                    }
                } else {
                    // Container topics - active if they exist
                    status = 'active';
                }
            } else {
                // Topic doesn't exist
                if (state.systemState === SystemState.CONTAINER_READY ||
                    state.systemState === SystemState.ROBOT_READY ||
                    state.systemState === SystemState.EXPLORATION_AVAILABLE ||
                    state.systemState === SystemState.EXPLORING) {
                    // Container topics should always exist
                    if (topic !== '/scan' && topic !== '/battery_state') {
                        status = 'inactive'; // Error - missing container topic
                    }
                    // Robot topics stay default (gray) if they don't exist
                }
            }

            updateServiceStatus(topic, status);
        });
    }, (error) => {
        logCommand('Error checking topics: ' + error);
    });
}

function checkHardwareData() {
    // Reset data flags - will be set by topic callbacks if data arrives
    // This check runs every 1s to verify data is still flowing
}

// === EVENT LISTENERS ===

function attachEventListeners() {
    // Status panel toggle
    document.getElementById('statusPanelToggle').addEventListener('click', toggleStatusPanel);

    // Connection
    document.getElementById('btnConnect').addEventListener('click', connectToROS);
    document.getElementById('btnDisconnect').addEventListener('click', disconnectFromROS);

    // Emergency stop
    document.getElementById('btnStop').addEventListener('click', emergencyStop);

    // Topics
    document.getElementById('btnRefreshTopics').addEventListener('click', refreshTopics);

    // SLAM commands
    document.getElementById('btnSaveMap').addEventListener('click', saveMap);
    document.getElementById('btnLoadSavedMap').addEventListener('click', loadSavedMap);
    document.getElementById('btnClearMap').addEventListener('click', clearMap);

    // Quick actions
    document.getElementById('btnClearMarkers').addEventListener('click', clearMarkers);
    document.getElementById('btnReturnHome').addEventListener('click', returnHome);

    // Navigation commands
    document.getElementById('btnNavigateToPoint').addEventListener('click', showNavigationModal);
    document.getElementById('btnCancelGoal').addEventListener('click', cancelNavigation);
    document.getElementById('btnSetInitialPose').addEventListener('click', setInitialPose);

    // Exploration commands
    document.getElementById('btnStartExplore').addEventListener('click', startExploration);
    document.getElementById('btnStopExplore').addEventListener('click', stopExploration);
    document.getElementById('btnCheckExplore').addEventListener('click', checkExploration);

    // Modal
    document.getElementById('btnCancelNav').addEventListener('click', hideNavigationModal);
    document.getElementById('btnConfirmNav').addEventListener('click', navigateToPoint);
}

// === STATUS PANEL TOGGLE ===

function toggleStatusPanel() {
    const panel = document.getElementById('statusPanel');
    const container = document.getElementById('mainContainer');

    panel.classList.toggle('collapsed');
    container.classList.toggle('status-collapsed');
}

// === STATUS MANAGEMENT ===

function updateMainStatus(state, message) {
    const statusMessage = document.getElementById('mainStatusMessage');
    statusMessage.textContent = message;
    statusMessage.className = 'header-status-message ' + state;
}

function setStatusDotState(dotElement, state) {
    // state: 'default' (gray), 'warning' (yellow blinking), 'online' (green), 'offline' (red)
    dotElement.className = 'service-status-dot ' + state;
}

function updateMiniStatus() {
    // Update mini status icons when panel is collapsed
    const rosConnected = state.connected;
    const containerOK = state.systemState === SystemState.CONTAINER_READY ||
                        state.systemState === SystemState.ROBOT_READY ||
                        state.systemState === SystemState.EXPLORATION_AVAILABLE ||
                        state.systemState === SystemState.EXPLORING;
    const robotOK = state.systemState === SystemState.ROBOT_READY ||
                    state.systemState === SystemState.EXPLORATION_AVAILABLE ||
                    state.systemState === SystemState.EXPLORING;

    document.getElementById('miniStatusROS').className = 'status-dot ' + (rosConnected ? 'online' : 'default');
    document.getElementById('miniStatusTopics').className = 'status-dot ' + (containerOK ? 'online' : 'default');
    document.getElementById('miniStatusNodes').className = 'status-dot ' + (robotOK ? 'online' : 'default');
}

function updateHighLevelStatus() {
    // Container status
    const containerNodes = ['/slam_toolbox', '/bt_navigator', '/controller_server', '/planner_server'];
    const containerOK = containerNodes.every(n => state.nodes[n]);

    let containerStatus = 'default';
    if (state.systemState === SystemState.WS_ERROR) {
        containerStatus = 'inactive';
    } else if (state.systemState === SystemState.CONTAINER_ERROR) {
        containerStatus = 'inactive';
    } else if (containerOK) {
        containerStatus = 'active';
    } else if (state.connected) {
        containerStatus = 'checking';
    }

    document.getElementById('statusContainer').className = 'service-status-dot ' + containerStatus;

    // Robot status
    const robotDataOK = state.scanDataReceived && state.batteryDataReceived;
    const robotNodeOK = state.nodes['/kaiaai_telemetry_node'];

    let robotStatus = 'default';
    if (state.systemState === SystemState.ROBOT_LOST) {
        robotStatus = 'inactive';
    } else if (robotDataOK && robotNodeOK) {
        robotStatus = 'active';
    } else if (state.systemState === SystemState.CONTAINER_READY) {
        robotStatus = 'default'; // Waiting for robot
    }

    document.getElementById('statusRobot').className = 'service-status-dot ' + robotStatus;

    // Exploration status
    const exploreNodeOK = state.nodes['/explore_node'];

    let explorationStatus = 'default';
    if (state.systemState === SystemState.EXPLORING && exploreNodeOK) {
        explorationStatus = 'active';
    } else if (state.systemState === SystemState.EXPLORATION_AVAILABLE) {
        explorationStatus = 'default'; // Ready but not started
    } else if (state.systemState === SystemState.EXPLORING && !exploreNodeOK) {
        explorationStatus = 'inactive'; // Should be running but isn't
    }

    document.getElementById('statusExploration').className = 'service-status-dot ' + explorationStatus;
}

// === ROS CONNECTION ===

function connectToROS() {
    const url = document.getElementById('rosUrl').value;

    logCommand('Connecting to ROS Bridge...');
    transitionToState(SystemState.CONNECTING_WS);

    state.ros = new ROSLIB.Ros({ url: url });

    state.ros.on('connection', () => {
        state.connected = true;
        logCommand('✓ Connected to ROS Bridge WebSocket');

        document.getElementById('btnConnect').disabled = true;
        document.getElementById('btnDisconnect').disabled = false;

        setupROSTopics();
        transitionToState(SystemState.WS_CONNECTED);
    });

    state.ros.on('error', (error) => {
        state.connected = false;
        logCommand('✗ WebSocket error: ' + error);
        transitionToState(SystemState.WS_ERROR);
    });

    state.ros.on('close', () => {
        state.connected = false;
        logCommand('Disconnected from ROS Bridge');

        document.getElementById('btnConnect').disabled = false;
        document.getElementById('btnDisconnect').disabled = true;

        stopMonitoring();
        resetAllStatus();
        transitionToState(SystemState.INITIAL);
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
        // Mark that battery data is being received
        state.batteryDataReceived = true;

        const voltage = message.voltage || message.percentage;
        const batteryText = voltage ? voltage.toFixed(2) + 'V' : 'N/A';

        document.getElementById('robotBattery').textContent = batteryText;
        document.getElementById('telemetryBattery').textContent = batteryText;

        // Color code battery level
        const batteryElement = document.getElementById('telemetryBattery');
        if (voltage) {
            if (voltage > 11.5) {
                batteryElement.style.color = 'var(--success-color)';
            } else if (voltage > 11.0) {
                batteryElement.style.color = 'var(--warning-color)';
            } else {
                batteryElement.style.color = 'var(--danger-color)';
            }
        }
    });

    // Scan topic for telemetry
    state.scanTopic = new ROSLIB.Topic({
        ros: state.ros,
        name: '/scan',
        messageType: 'sensor_msgs/LaserScan'
    });

    state.scanTopic.subscribe((message) => {
        // Mark that scan data is being received
        state.scanDataReceived = true;

        // Get min/max range from scan data
        const ranges = message.ranges.filter(r => r > message.range_min && r < message.range_max);
        if (ranges.length > 0) {
            const minRange = Math.min(...ranges);
            const maxRange = Math.max(...ranges);
            document.getElementById('telemetryScan').textContent =
                `${minRange.toFixed(2)}m - ${maxRange.toFixed(2)}m`;
        }
    });

    // Navigation action client
    state.navAction = new ROSLIB.ActionClient({
        ros: state.ros,
        serverName: '/navigate_to_pose',
        actionName: 'nav2_msgs/action/NavigateToPose'
    });

    logCommand('ROS topics initialized');
}

// OBSOLETE - Removed, replaced by checkTopics() and real-time monitoring

function checkNodes() {
    if (!state.ros) return;

    state.ros.getNodes((nodes) => {
        // Check each node
        Object.keys(state.nodes).forEach(nodeName => {
            const exists = nodes.includes(nodeName);
            const wasActive = state.nodes[nodeName];
            state.nodes[nodeName] = exists;

            // Determine status based on node type and system state
            let status = 'default';
            if (exists) {
                status = 'active';
            } else {
                // Node missing - determine if it's an error
                const containerNodes = [
                    '/slam_toolbox', '/bt_navigator', '/controller_server',
                    '/planner_server', '/behavior_server', '/velocity_smoother',
                    '/lifecycle_manager_navigation', '/lifecycle_manager_slam',
                    '/rosbridge_websocket'
                ];
                const robotNodes = ['/kaiaai_telemetry_node', '/robot_state_publisher'];
                const explorationNodes = ['/explore_node'];

                if (containerNodes.includes(nodeName)) {
                    // Container node missing - always error if we're connected
                    if (state.systemState !== SystemState.INITIAL &&
                        state.systemState !== SystemState.CONNECTING_WS) {
                        status = 'inactive';
                    }
                } else if (robotNodes.includes(nodeName)) {
                    // Robot node - only error if robot should be connected
                    if (state.systemState === SystemState.ROBOT_READY ||
                        state.systemState === SystemState.EXPLORATION_AVAILABLE ||
                        state.systemState === SystemState.EXPLORING) {
                        status = 'inactive';
                    }
                } else if (explorationNodes.includes(nodeName)) {
                    // Exploration node - only matters if we're exploring
                    if (state.systemState === SystemState.EXPLORING) {
                        status = 'inactive';
                    }
                }
            }

            updateNodeStatus(nodeName, status);
        });
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

            // Update UI - Map overlay
            document.getElementById('robotX').textContent = state.robotPose.x.toFixed(2) + ' m';
            document.getElementById('robotY').textContent = state.robotPose.y.toFixed(2) + ' m';
            document.getElementById('robotTheta').textContent = state.robotPose.theta.toFixed(2) + ' rad';

            // Update UI - Telemetry panel
            document.getElementById('telemetryPosX').textContent = state.robotPose.x.toFixed(2) + ' m';
            document.getElementById('telemetryPosY').textContent = state.robotPose.y.toFixed(2) + ' m';
            document.getElementById('telemetryTheta').textContent = (state.robotPose.theta * 180 / Math.PI).toFixed(1) + '°';

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

    // Check if already subscribed
    if (state.mapTopic._subscribeId) {
        logCommand('Already subscribed to map updates');
        return;
    }

    logCommand('Starting live map updates...');

    let firstMapReceived = false;

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

        renderMapOnLeaflet(firstMapReceived);

        if (!firstMapReceived) {
            logCommand(`✓ Map loaded: ${state.mapData.width}x${state.mapData.height} @ ${state.mapData.resolution}m/px`);
            logCommand('✓ Real-time live map updates active');
            firstMapReceived = true;
        }

        // Keep subscribed for live updates (like RViz)
    });
}

function renderMapOnLeaflet(isFirstMap = false) {
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

    // Remove old layer and add new one
    if (state.mapLayer) {
        state.map.removeLayer(state.mapLayer);
    }

    // Add new map layer
    state.mapLayer = L.imageOverlay(imageUrl, bounds).addTo(state.map);

    // Only fit bounds on first map load, not on updates
    // This prevents the map from jumping around during exploration
    if (!isFirstMap) {
        state.map.fitBounds(bounds);
    }

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
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    logCommand('Click on map to set initial robot pose...');
    updateMainStatus('connecting', 'Cliquez sur la carte pour définir la position initiale...');

    // Enable one-time click handler
    state.map.once('click', (e) => {
        const worldCoords = leafletToWorld(e.latlng);

        logCommand(`Setting initial pose at (${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)})`);

        // Publish to /initialpose topic
        const initialPoseTopic = new ROSLIB.Topic({
            ros: state.ros,
            name: '/initialpose',
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
        });

        const poseMsg = new ROSLIB.Message({
            header: {
                frame_id: 'map',
                stamp: { sec: 0, nanosec: 0 }
            },
            pose: {
                pose: {
                    position: { x: worldCoords.x, y: worldCoords.y, z: 0.0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1.0 }
                },
                covariance: [
                    0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0.06853892326654787
                ]
            }
        });

        initialPoseTopic.publish(poseMsg);

        logCommand(`✓ Initial pose set at (${worldCoords.x.toFixed(2)}, ${worldCoords.y.toFixed(2)})`);
        updateMainStatus('connected', 'Position initiale définie');

        // Add temporary marker
        const poseMarker = L.circleMarker(e.latlng, {
            radius: 8,
            color: '#10b981',
            fillColor: '#10b981',
            fillOpacity: 0.5
        }).addTo(state.map);

        setTimeout(() => {
            state.map.removeLayer(poseMarker);
        }, 3000);
    });
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

    logCommand('Starting exploration node...');
    updateMainStatus('connecting', 'Démarrage de l\'exploration...');

    // Call ROS2 Control API to start explore_lite
    fetch('http://192.168.0.10:8083/', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: 'start_explore' })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logCommand('✓ Exploration node started (PID: ' + data.pid + ')');
            updateMainStatus('connected', 'Exploration autonome en cours...');

            // Wait a bit then check node status
            setTimeout(() => {
                checkNodes();
                checkExploration();
            }, 3000);
        } else {
            logCommand('✗ Failed to start exploration: ' + data.message);
            updateMainStatus('error', 'Erreur démarrage exploration: ' + data.message);
        }
    })
    .catch(error => {
        logCommand('✗ Error calling ROS2 API: ' + error);
        updateMainStatus('error', 'Erreur API ROS2 - Vérifiez le serveur');
    });
}

function stopExploration() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    logCommand('Stopping exploration...');
    updateMainStatus('connecting', 'Arrêt de l\'exploration...');

    // Call ROS2 Control API to stop explore_lite
    fetch('http://192.168.0.10:8083/', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: 'stop_explore' })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            logCommand('✓ Exploration stopped');
            updateMainStatus('connected', 'Exploration arrêtée');

            // Cancel any active navigation goals
            if (state.navAction) {
                cancelNavigation();
            }

            // Stop the robot
            publishVelocity(0, 0);

            // Refresh node status
            setTimeout(() => {
                checkNodes();
            }, 2000);
        } else {
            logCommand('✗ Failed to stop exploration: ' + data.message);
            updateMainStatus('error', 'Erreur arrêt exploration: ' + data.message);
        }
    })
    .catch(error => {
        logCommand('✗ Error calling ROS2 API: ' + error);
        updateMainStatus('error', 'Erreur API ROS2');
    });
}

function checkExploration() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    state.ros.getNodes((nodes) => {
        const exploreNode = nodes.find(n => n === '/explore');

        if (exploreNode) {
            logCommand('✓ Exploration node active: /explore');
            updateMainStatus('connected', 'Exploration active - Node /explore opérationnel');

            // Check topics related to exploration
            state.ros.getTopics((result) => {
                const exploreTopics = result.topics.filter(t =>
                    t.includes('explore') || t.includes('frontier')
                );
                if (exploreTopics.length > 0) {
                    logCommand('✓ Exploration topics: ' + exploreTopics.join(', '));
                }
            });
        } else {
            logCommand('✗ Exploration node not found');
            updateMainStatus('error', 'Node /explore introuvable');
        }
    });
}

// === SLAM COMMANDS ===

function saveMap() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    const mapName = prompt('Enter map name:', 'my_map');
    if (!mapName) return;

    logCommand(`Saving map as "${mapName}"...`);

    // Call slam_toolbox serialize map service
    const saveMapService = new ROSLIB.Service({
        ros: state.ros,
        name: '/slam_toolbox/serialize_map',
        serviceType: 'slam_toolbox/srv/SerializePoseGraph'
    });

    const request = new ROSLIB.ServiceRequest({
        filename: `/app/maps/${mapName}`
    });

    saveMapService.callService(request, (result) => {
        if (result.result) {
            logCommand(`✓ Map saved successfully: ${mapName}`);
            updateMainStatus('connected', `Carte "${mapName}" sauvegardée avec succès`);
        } else {
            logCommand(`✗ Failed to save map: ${mapName}`);
            updateMainStatus('error', 'Erreur lors de la sauvegarde de la carte');
        }
    }, (error) => {
        logCommand('✗ Error saving map: ' + error);
        updateMainStatus('error', 'Erreur lors de la sauvegarde de la carte');
    });
}

function clearMap() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    if (!confirm('Are you sure you want to clear the current map?')) {
        return;
    }

    logCommand('Clearing map...');

    // Call slam_toolbox clear changes service
    const clearService = new ROSLIB.Service({
        ros: state.ros,
        name: '/slam_toolbox/clear_changes',
        serviceType: 'slam_toolbox/srv/Clear'
    });

    const request = new ROSLIB.ServiceRequest({});

    clearService.callService(request, (result) => {
        logCommand('✓ Map cleared');
        updateMainStatus('connected', 'Carte effacée - Nouvelle cartographie en cours');

        // Clear map visualization
        if (state.mapLayer) {
            state.map.removeLayer(state.mapLayer);
            state.mapLayer = null;
        }
        state.mapData = null;
    }, (error) => {
        logCommand('✗ Error clearing map: ' + error);
        updateMainStatus('error', 'Erreur lors de l\'effacement de la carte');
    });
}

function loadSavedMap() {
    if (!state.ros) {
        logCommand('Not connected to ROS');
        return;
    }

    const mapName = prompt('Enter map name to load:', 'my_map');
    if (!mapName) return;

    logCommand(`Loading saved map "${mapName}"...`);

    // Call slam_toolbox deserialize map service
    const loadMapService = new ROSLIB.Service({
        ros: state.ros,
        name: '/slam_toolbox/deserialize_map',
        serviceType: 'slam_toolbox/srv/DeserializePoseGraph'
    });

    const request = new ROSLIB.ServiceRequest({
        filename: `/app/maps/${mapName}`,
        match_type: 2 // Start at dock
    });

    loadMapService.callService(request, (result) => {
        if (result.result) {
            logCommand(`✓ Saved map loaded: ${mapName}`);
            updateMainStatus('connected', `Carte "${mapName}" chargée avec succès`);
            // Trigger map visualization update
            loadMap();
        } else {
            logCommand(`✗ Failed to load map: ${mapName}`);
            updateMainStatus('error', 'Erreur lors du chargement de la carte');
        }
    }, (error) => {
        logCommand('✗ Error loading saved map: ' + error);
        updateMainStatus('error', 'Erreur lors du chargement de la carte');
    });
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

function updateServiceStatus(serviceName, status) {
    // Find the service item in the grid
    const serviceItems = document.querySelectorAll('.service-item');
    serviceItems.forEach(item => {
        if (item.textContent.includes(serviceName)) {
            const dot = item.querySelector('.service-status-dot');
            // status: 'default', 'checking', 'active', 'inactive'
            if (typeof status === 'boolean') {
                dot.className = 'service-status-dot ' + (status ? 'active' : 'default');
            } else {
                dot.className = 'service-status-dot ' + status;
            }

            // Gray out inactive items
            if (status === 'default' || status === 'inactive') {
                item.classList.add('disabled');
            } else {
                item.classList.remove('disabled');
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
                dot.className = 'service-status-dot ' + (status ? 'active' : 'default');
            } else {
                dot.className = 'service-status-dot ' + status;
            }

            // Gray out inactive items
            if (status === 'default' || status === 'inactive') {
                item.classList.add('disabled');
            } else {
                item.classList.remove('disabled');
            }
        }
    });
}

// Moved to resetAllStatus() in state machine section - function removed

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
