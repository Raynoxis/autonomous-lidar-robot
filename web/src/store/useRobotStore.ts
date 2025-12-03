import { create } from 'zustand';
import { rosService } from '../services/rosService';
import { apiService } from '../services/apiService';
import type {
  SystemState,
  RobotPose,
  MapData,
  MapBounds,
  TopicStatus,
  NodeStatus,
  LogEntry,
  TFMessage,
  OccupancyGrid,
  BatteryState,
  LaserScan,
} from '../types';

interface RobotStore {
  // Connection state
  connected: boolean;
  systemState: SystemState;
  rosUrl: string;

  // Robot data
  robotPose: RobotPose;
  homePosition: RobotPose | null;
  batteryVoltage: number | null;
  scanRange: { min: number; max: number } | null;

  // Map data
  mapData: MapData | null;
  mapBounds: MapBounds | null;

  // Hardware data flags
  scanDataReceived: boolean;
  batteryDataReceived: boolean;

  // Topics and nodes status
  topics: TopicStatus;
  nodes: NodeStatus;

  // Monitoring intervals
  monitoringIntervals: {
    topics: ReturnType<typeof setTimeout> | null;
    nodes: ReturnType<typeof setTimeout> | null;
    hardware: ReturnType<typeof setTimeout> | null;
    state: ReturnType<typeof setTimeout> | null;
  };

  // Command log
  commandLog: LogEntry[];

  // Actions
  setRosUrl: (url: string) => void;
  connect: () => void;
  disconnect: () => void;
  publishVelocity: (linear: number, angular: number) => void;
  emergencyStop: () => void;
  setInitialPose: (x: number, y: number, theta: number) => void;
  sendNavigationGoal: (x: number, y: number) => void;
  cancelNavigation: () => void;
  saveMap: (mapName: string) => void;
  loadMap: (mapName: string) => void;
  clearMap: () => void;
  startExploration: () => void;
  stopExploration: () => void;
  subscribeToTopics: () => void;
  startMonitoring: () => void;
  stopMonitoring: () => void;
  checkTopics: () => void;
  checkNodes: () => void;
  evaluateSystemState: () => void;
  setHomePosition: () => void;
  addLog: (message: string) => void;
  transitionToState: (newState: SystemState) => void;
}

const CRITICAL_CONTAINER_NODES = [
  '/slam_toolbox',
  '/bt_navigator',
  '/controller_server',
  '/planner_server',
];

export const useRobotStore = create<RobotStore>((set, get) => ({
  // Initial state
  connected: false,
  systemState: 'initial' as SystemState,
  rosUrl: 'ws://192.168.0.10:9092',

  robotPose: { x: 0, y: 0, theta: 0 },
  homePosition: null,
  batteryVoltage: null,
  scanRange: null,

  mapData: null,
  mapBounds: null,

  scanDataReceived: false,
  batteryDataReceived: false,

  topics: {
    '/map': false,
    '/tf': false,
    '/odom': false,
    '/cmd_vel': false,
    '/scan': false,
    '/battery_state': false,
  },

  nodes: {
    '/slam_toolbox': false,
    '/bt_navigator': false,
    '/controller_server': false,
    '/planner_server': false,
    '/behavior_server': false,
    '/velocity_smoother': false,
    '/lifecycle_manager_navigation': false,
    '/lifecycle_manager_slam': false,
    '/rosbridge_websocket': false,
    '/kaiaai_telemetry_node': false,
    '/robot_state_publisher': false,
    '/explore_node': false,
  },

  monitoringIntervals: {
    topics: null,
    nodes: null,
    hardware: null,
    state: null,
  },

  commandLog: [],

  // Set ROS URL
  setRosUrl: (url: string) => {
    set({ rosUrl: url });
  },

  // Add log entry
  addLog: (message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    const entry: LogEntry = { timestamp, message };
    set((state) => ({
      commandLog: [entry, ...state.commandLog].slice(0, 50), // Keep last 50
    }));
  },

  // Transition to new system state
  transitionToState: (newState: SystemState) => {
    const { addLog } = get();
    set({ systemState: newState });
    addLog(`State: ${newState}`);

    // State-specific actions
    if (newState === 'initial') {
      // Reset all status
      set({
        topics: Object.fromEntries(Object.keys(get().topics).map((k) => [k, false])),
        nodes: Object.fromEntries(Object.keys(get().nodes).map((k) => [k, false])),
        scanDataReceived: false,
        batteryDataReceived: false,
      });
    } else if (newState === 'ws_connected') {
      get().startMonitoring();
    } else if (newState === 'robot_ready') {
      // Auto-subscribe to map if not already subscribed
      if (rosService.mapTopic && !rosService.mapTopic._subscribeId) {
        get().subscribeToTopics();
      }
    }
  },

  // Connect to ROS
  connect: () => {
    const { rosUrl, transitionToState, addLog, subscribeToTopics } = get();

    addLog('Connecting to ROS Bridge...');
    transitionToState('connecting_ws');

    rosService.connect(
      rosUrl,
      () => {
        // On connection
        set({ connected: true });
        addLog('✓ Connected to ROS Bridge WebSocket');
        subscribeToTopics();
        transitionToState('ws_connected');
      },
      (error) => {
        // On error
        set({ connected: false });
        addLog(`✗ WebSocket error: ${error}`);
        transitionToState('ws_error');
      },
      () => {
        // On close
        set({ connected: false });
        addLog('Disconnected from ROS Bridge');
        get().stopMonitoring();
        transitionToState('initial');
      }
    );
  },

  // Disconnect from ROS
  disconnect: () => {
    get().stopMonitoring();
    rosService.disconnect();
    set({ connected: false });
    get().transitionToState('initial');
  },

  // Subscribe to ROS topics
  subscribeToTopics: () => {
    const { addLog } = get();

    // Subscribe to TF for robot position
    if (rosService.tfTopic) {
      rosService.tfTopic.subscribe((message: TFMessage) => {
        // Find base_link transform
        for (const transform of message.transforms) {
          if (
            transform.header.frame_id === 'map' &&
            transform.child_frame_id === 'base_link'
          ) {
            const { x, y } = transform.transform.translation;
            const q = transform.transform.rotation;
            const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

            set({ robotPose: { x, y, theta } });
          }
        }
      });
    }

    // Subscribe to battery
    if (rosService.batteryTopic) {
      rosService.batteryTopic.subscribe((message: BatteryState) => {
        set({
          batteryDataReceived: true,
          batteryVoltage: message.voltage || message.percentage,
        });
      });
    }

    // Subscribe to scan
    if (rosService.scanTopic) {
      rosService.scanTopic.subscribe((message: LaserScan) => {
        const ranges = message.ranges.filter(
          (r) => r > message.range_min && r < message.range_max
        );
        if (ranges.length > 0) {
          const min = Math.min(...ranges);
          const max = Math.max(...ranges);
          set({ scanDataReceived: true, scanRange: { min, max } });
        }
      });
    }

    // Subscribe to map
    if (rosService.mapTopic) {
      let firstMap = false;
      rosService.mapTopic.subscribe((message: OccupancyGrid) => {
        const mapData: MapData = {
          width: message.info.width,
          height: message.info.height,
          resolution: message.info.resolution,
          origin: {
            x: message.info.origin.position.x,
            y: message.info.origin.position.y,
          },
          data: message.data,
        };

        const worldWidth = mapData.width * mapData.resolution;
        const worldHeight = mapData.height * mapData.resolution;

        const mapBounds: MapBounds = {
          minX: mapData.origin.x,
          minY: mapData.origin.y,
          maxX: mapData.origin.x + worldWidth,
          maxY: mapData.origin.y + worldHeight,
        };

        set({ mapData, mapBounds });

        if (!firstMap) {
          addLog(
            `✓ Map loaded: ${mapData.width}x${mapData.height} @ ${mapData.resolution}m/px`
          );
          addLog('✓ Real-time live map updates active');
          firstMap = true;
        }
      });
    }

    addLog('ROS topics subscribed');
  },

  // Publish velocity
  publishVelocity: (linear: number, angular: number) => {
    rosService.publishVelocity(linear, angular);
  },

  // Emergency stop
  emergencyStop: () => {
    rosService.publishVelocity(0, 0);
    get().addLog('⚠ EMERGENCY STOP');
  },

  // Set initial pose
  setInitialPose: (x: number, y: number, theta: number) => {
    rosService.publishInitialPose(x, y, theta);
    get().addLog(`✓ Initial pose set at (${x.toFixed(2)}, ${y.toFixed(2)})`);
  },

  // Send navigation goal
  sendNavigationGoal: (x: number, y: number) => {
    const { addLog } = get();

    rosService.sendNavigationGoal(
      x,
      y,
      undefined,
      () => {
        addLog('✓ Navigation completed');
      },
      (error) => {
        addLog(`✗ Navigation error: ${error}`);
      }
    );

    addLog(`Navigating to (${x.toFixed(2)}, ${y.toFixed(2)})...`);
    get().transitionToState('navigating');
  },

  // Cancel navigation
  cancelNavigation: () => {
    rosService.cancelNavigation();
    get().addLog('Navigation cancelled');
  },

  // Save map
  saveMap: (mapName: string) => {
    const { addLog } = get();
    rosService.saveMap(mapName, (success) => {
      if (success) {
        addLog(`✓ Map saved: ${mapName}`);
      } else {
        addLog(`✗ Failed to save map: ${mapName}`);
      }
    });
  },

  // Load map
  loadMap: (mapName: string) => {
    const { addLog } = get();
    rosService.loadMap(mapName, (success) => {
      if (success) {
        addLog(`✓ Map loaded: ${mapName}`);
      } else {
        addLog(`✗ Failed to load map: ${mapName}`);
      }
    });
  },

  // Clear map
  clearMap: () => {
    const { addLog } = get();
    rosService.clearMap((success) => {
      if (success) {
        addLog('✓ Map cleared');
        set({ mapData: null, mapBounds: null });
      } else {
        addLog('✗ Failed to clear map');
      }
    });
  },

  // Start exploration
  startExploration: async () => {
    const { addLog, transitionToState } = get();
    addLog('Starting exploration...');

    const response = await apiService.startExploration();
    if (response.success) {
      addLog(`✓ Exploration started (PID: ${response.data?.pid})`);
      transitionToState('exploring');
    } else {
      addLog(`✗ Failed to start exploration: ${response.message}`);
    }
  },

  // Stop exploration
  stopExploration: async () => {
    const { addLog } = get();
    addLog('Stopping exploration...');

    const response = await apiService.stopExploration();
    if (response.success) {
      addLog('✓ Exploration stopped');
      rosService.cancelNavigation();
      rosService.publishVelocity(0, 0);
    } else {
      addLog(`✗ Failed to stop exploration: ${response.message}`);
    }
  },

  // Start monitoring
  startMonitoring: () => {
    const { checkTopics, checkNodes, evaluateSystemState } = get();

    // Topics check: 100ms
    const topicsInterval = setInterval(() => {
      if (get().connected) {
        checkTopics();
      }
    }, 100);

    // Nodes check: 500ms (with 2s delay)
    const nodesInterval = setTimeout(() => {
      const interval = setInterval(() => {
        if (get().connected) {
          checkNodes();
        }
      }, 500);

      set((state) => ({
        monitoringIntervals: { ...state.monitoringIntervals, nodes: interval },
      }));
    }, 2000);

    // State evaluation: 200ms
    const stateInterval = setInterval(() => {
      if (get().connected) {
        evaluateSystemState();
      }
    }, 200);

    set({
      monitoringIntervals: {
        topics: topicsInterval,
        nodes: nodesInterval as any,
        hardware: null,
        state: stateInterval,
      },
    });

    get().addLog('Monitoring started');
  },

  // Stop monitoring
  stopMonitoring: () => {
    const { monitoringIntervals } = get();

    if (monitoringIntervals.topics) clearInterval(monitoringIntervals.topics);
    if (monitoringIntervals.nodes) clearInterval(monitoringIntervals.nodes);
    if (monitoringIntervals.hardware) clearInterval(monitoringIntervals.hardware);
    if (monitoringIntervals.state) clearInterval(monitoringIntervals.state);

    set({
      monitoringIntervals: {
        topics: null,
        nodes: null,
        hardware: null,
        state: null,
      },
    });

    get().addLog('Monitoring stopped');
  },

  // Check topics
  checkTopics: () => {
    rosService.getTopics((topics) => {
      const topicStatus = { ...get().topics };

      Object.keys(topicStatus).forEach((topic) => {
        topicStatus[topic] = topics.includes(topic);
      });

      set({ topics: topicStatus });
    });
  },

  // Check nodes
  checkNodes: () => {
    rosService.getNodes((nodes) => {
      const nodeStatus = { ...get().nodes };

      Object.keys(nodeStatus).forEach((node) => {
        nodeStatus[node] = nodes.includes(node);
      });

      set({ nodes: nodeStatus });
    });
  },

  // Evaluate system state
  evaluateSystemState: () => {
    const {
      connected,
      systemState,
      nodes,
      scanDataReceived,
      batteryDataReceived,
      mapData,
      transitionToState,
    } = get();

    if (!connected) {
      if (systemState !== 'initial') {
        transitionToState('ws_error');
      }
      return;
    }

    // Check container nodes
    const containerOK = CRITICAL_CONTAINER_NODES.every((n) => nodes[n]);
    const containerCount = CRITICAL_CONTAINER_NODES.filter((n) => nodes[n]).length;

    if (!containerOK && containerCount < 2) {
      if (systemState !== 'container_error' && systemState !== 'ws_connected') {
        transitionToState('container_error');
      }
      return;
    }

    // Container is ready
    if (systemState === 'ws_connected') {
      transitionToState('container_ready');
    }

    // Check robot data
    const robotDataOK = scanDataReceived && batteryDataReceived;
    const robotNodeOK = nodes['/kaiaai_telemetry_node'];

    if (robotDataOK && robotNodeOK) {
      if (
        systemState !== 'robot_ready' &&
        systemState !== 'exploration_available' &&
        systemState !== 'exploring' &&
        systemState !== 'navigating'
      ) {
        transitionToState('robot_ready');
      }

      // Check if map is loaded for exploration
      if (mapData && systemState === 'robot_ready') {
        transitionToState('exploration_available');
      }

      // Check exploration state
      if (nodes['/explore_node']) {
        if (systemState !== 'exploring') {
          transitionToState('exploring');
        }
      }
    } else {
      // Robot lost
      if (
        systemState === 'robot_ready' ||
        systemState === 'exploration_available' ||
        systemState === 'exploring'
      ) {
        transitionToState('robot_lost');
      }
    }
  },

  // Set home position
  setHomePosition: () => {
    const { robotPose, addLog } = get();
    set({ homePosition: { ...robotPose } });
    addLog('Home position set to current location');
  },
}));
