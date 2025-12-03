// System State Machine
export const SystemState = {
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
  EXPLORATION_ERROR: 'exploration_error',
} as const;

export type SystemState = typeof SystemState[keyof typeof SystemState];

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
