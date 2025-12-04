// System State Machine - 10 états finaux
export const SystemState = {
  // Démarrage
  INITIAL: 'initial',
  CONNECTING_WS: 'connecting_ws',
  WS_CONNECTED: 'ws_connected',
  CONTAINER_READY: 'container_ready',
  ROBOT_READY: 'robot_ready',
  // Opérationnel
  EXPLORING: 'exploring',
  NAVIGATING: 'navigating',
  // Erreurs
  WS_ERROR: 'ws_error',
  CONTAINER_ERROR: 'container_error',
  ROBOT_LOST: 'robot_lost',
} as const;

export type SystemState = typeof SystemState[keyof typeof SystemState];

// System Status Item pour affichage dynamique
export interface SystemStatusItem {
  label: string;
  status: 'pending' | 'checking' | 'success' | 'error' | 'disabled';
  detail?: string;
}

// Status Indicator Colors
export type StatusColor = 'gray' | 'yellow' | 'green' | 'orange' | 'red';

export interface StateDisplayInfo {
  color: StatusColor;
  pulse: boolean;
  message: string;
  detail: string;
  progress: number;
}

// Robot Pose
export interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

// Map Data
export interface MapData {
  width: number;
  height: number;
  resolution: number;
  origin: {
    x: number;
    y: number;
  };
  data: number[];
}

export interface MapBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

// Topic Status
export type TopicStatus = Record<string, boolean>;

// Node Status
export type NodeStatus = Record<string, boolean>;

// Status indicator states
export type StatusDot = 'default' | 'active' | 'inactive' | 'checking';

// Connection Config
export interface ConnectionConfig {
  rosUrl: string;
}

// Command Log Entry
export interface LogEntry {
  timestamp: string;
  message: string;
}
