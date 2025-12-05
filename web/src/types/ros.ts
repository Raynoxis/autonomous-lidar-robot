// ROS Message Types
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Pose {
  position: Point;
  orientation: Quaternion;
}

export interface Transform {
  translation: Vector3;
  rotation: Quaternion;
}

export interface Header {
  frame_id: string;
  stamp: {
    sec: number;
    nanosec: number;
  };
}

// Twist (velocity command)
export interface Twist {
  linear: Vector3;
  angular: Vector3;
}

// LaserScan
export interface LaserScan {
  header: Header;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

// OccupancyGrid (Map)
export interface MapMetaData {
  map_load_time: { sec: number; nanosec: number };
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
}

export interface OccupancyGrid {
  header: Header;
  info: MapMetaData;
  data: number[];
}

// TF Transform Message
export interface TransformStamped {
  header: Header;
  child_frame_id: string;
  transform: Transform;
}

export interface TFMessage {
  transforms: TransformStamped[];
}

// Battery State
export interface BatteryState {
  header: Header;
  voltage: number;
  temperature: number;
  current: number;
  charge: number;
  capacity: number;
  design_capacity: number;
  percentage: number;
  power_supply_status: number;
  power_supply_health: number;
  power_supply_technology: number;
  present: boolean;
}

// Odometry
export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: {
    pose: Pose;
    covariance: number[];
  };
  twist: {
    twist: Twist;
    covariance: number[];
  };
}

// PoseWithCovarianceStamped (for initial pose)
export interface PoseWithCovarianceStamped {
  header: Header;
  pose: {
    pose: Pose;
    covariance: number[];
  };
}

// Goal Pose (for navigation)
export interface PoseStamped {
  header: Header;
  pose: Pose;
}

// Path
export interface PoseStampedWithFrame {
  header: Header;
  pose: Pose;
}

export interface Path {
  header: Header;
  poses: PoseStampedWithFrame[];
}

export interface NavigateToPoseGoal {
  pose: PoseStamped;
}

export interface NavigateToPoseFeedback {
  current_pose: PoseStamped;
  navigation_time: { sec: number; nanosec: number };
  estimated_time_remaining: { sec: number; nanosec: number };
  number_of_recoveries: number;
  distance_remaining: number;
}

export interface NavigateToPoseResult {
  result: number;
}

// ROS Node/Topic Info
export interface ROSNodeInfo {
  name: string;
  namespace: string;
}

export interface ROSTopicInfo {
  name: string;
  type: string;
}

// Map Updates (for incremental map refresh)
export interface OccupancyGridUpdate {
  header: Header;
  x: number;
  y: number;
  width: number;
  height: number;
  data: number[];
}

// MarkerArray (frontiers or viz markers)
export interface Marker {
  header: Header;
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: Pose;
  scale: Vector3;
  color: { r: number; g: number; b: number; a: number };
  points: Point[];
}

export interface MarkerArray {
  markers: Marker[];
}

export interface ROSServiceInfo {
  name: string;
  type: string;
}
