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
  Transform,
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
      // Compose map -> odom -> base_(footprint|link) to always recover robot pose
      let mapToOdom: Transform | null = null;
      let odomToBase: Transform | null = null;

      const toPose = (translation: Transform['translation'], rotation: Transform['rotation']) => {
        const theta = Math.atan2(
          2 * (rotation.w * rotation.z + rotation.x * rotation.y),
          1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z)
        );
        return { x: translation.x, y: translation.y, theta };
      };

      const multiplyQuat = (a: Transform['rotation'], b: Transform['rotation']) => ({
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      });

      const rotateVec = (q: Transform['rotation'], v: Transform['translation']) => {
        // Rotate vector v by quaternion q (formula avoids extra deps)
        const u = { x: q.x, y: q.y, z: q.z };
        const s = q.w;
        const cross = (a: typeof u, b: typeof u) => ({
          x: a.y * b.z - a.z * b.y,
          y: a.z * b.x - a.x * b.z,
          z: a.x * b.y - a.y * b.x,
        });
        const dot = u.x * v.x + u.y * v.y + u.z * v.z;
        const cx = cross(u, v);
        return {
          x: 2 * dot * u.x + (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * v.x + 2 * s * cx.x,
          y: 2 * dot * u.y + (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * v.y + 2 * s * cx.y,
          z: 2 * dot * u.z + (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * v.z + 2 * s * cx.z,
        };
      };

      rosService.tfTopic.subscribe((message: TFMessage) => {
        for (const transform of message.transforms) {
          const parent = transform.header.frame_id;
          const child = transform.child_frame_id;

          // Direct transform map -> base_link (best case)
          if (parent === 'map' && child === 'base_link') {
            const pose = toPose(transform.transform.translation, transform.transform.rotation);
            set({ robotPose: pose });
            return;
          }

          if (parent === 'map' && child === 'odom') {
            mapToOdom = transform.transform;
          }

          if (parent === 'odom' && (child === 'base_footprint' || child === 'base_link')) {
            odomToBase = transform.transform;
          }
        }

        // Compose map->odom with odom->base if direct transform is absent
        if (mapToOdom && odomToBase) {
          const rotated = rotateVec(mapToOdom.rotation, odomToBase.translation);
          const composedTranslation = {
            x: mapToOdom.translation.x + rotated.x,
            y: mapToOdom.translation.y + rotated.y,
            z: mapToOdom.translation.z + rotated.z,
          };
          const composedRotation = multiplyQuat(mapToOdom.rotation, odomToBase.rotation);
          set({ robotPose: toPose(composedTranslation, composedRotation) });
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
    const { addLog, transitionToState } = get();

    apiService.sendNavigationGoal(x, y, 0).then((response) => {
      if (response.success) {
        addLog(`✓ Goal sent (${x.toFixed(2)}, ${y.toFixed(2)})`);
        transitionToState('navigating');
      } else {
        addLog(`✗ Navigation error: ${response.message || 'send goal failed'}`);
      }
    });
  },

  // Cancel navigation
  cancelNavigation: () => {
    apiService.cancelNavigationGoal().then((response) => {
      if (response.success) {
        get().addLog('Navigation cancelled');
      } else {
        get().addLog(`✗ Cancel failed: ${response.message || ''}`);
      }
    });
  },

  // Save map
  saveMap: (mapName: string) => {
    const { addLog } = get();
    apiService.saveMap(mapName).then((response) => {
      if (response.success) {
        addLog(`✓ Map saved: ${mapName}`);
      } else {
        addLog(`✗ Failed to save map: ${response.message || mapName}`);
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
    const { addLog, systemState, checkNodes } = get();

    if (systemState !== 'exploring') {
      addLog('⚠ Exploration not active');
      return;
    }

    addLog('Stopping exploration...');

    const response = await apiService.stopExploration();
    if (response.success) {
      // 1. Arrêt sécurisé robot
      rosService.publishVelocity(0, 0);

      // 2. Attendre que explore_node disparaisse puis transition
      setTimeout(() => {
        checkNodes();
        if (!get().nodes['/explore_node']) {
          addLog('✓ Exploration stopped - explore_node terminated');
        }
        // 3. Retour à robot_ready après vérification
        addLog('✓ Exploration stopped');
        get().transitionToState('robot_ready');
      }, 2000);
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

    // Check robot data - scan suffit, battery optionnelle
    const robotDataOK = scanDataReceived;
    const robotNodeOK = nodes['/kaiaai_telemetry_node'];

    if (robotDataOK && robotNodeOK) {
      // Transition vers robot_ready si pas déjà en mode opérationnel
      if (
        systemState !== 'robot_ready' &&
        systemState !== 'exploring' &&
        systemState !== 'navigating'
      ) {
        transitionToState('robot_ready');
      }

      // Pas d'auto-transition vers exploring - seulement via startExploration()
      // L'utilisateur contrôle explicitement le démarrage de l'exploration
    } else {
      // Robot lost - perte des données robot
      if (
        systemState === 'robot_ready' ||
        systemState === 'exploring' ||
        systemState === 'navigating'
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
