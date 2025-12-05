import { Ros, Topic, Service } from 'roslib';
import type {
  Twist,
  TFMessage,
  OccupancyGrid,
  BatteryState,
  LaserScan,
  PoseWithCovarianceStamped,
} from '../types';

export class ROSService {
  private ros: Ros | null = null;
  private reconnectInterval: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 10;
  private reconnectDelay = 3000; // 3 seconds

  // Topics
  public cmdVelTopic: Topic<Twist> | null = null;
  public mapTopic: Topic<OccupancyGrid> | null = null;
  public tfTopic: Topic<TFMessage> | null = null;
  public batteryTopic: Topic<BatteryState> | null = null;
  public scanTopic: Topic<LaserScan> | null = null;

  // Actions
  // Event callbacks
  private onConnectionCallback?: () => void;
  private onErrorCallback?: (error: any) => void;
  private onCloseCallback?: () => void;

  constructor() {}

  /**
   * Connect to ROS Bridge WebSocket
   */
  connect(
    url: string,
    onConnection?: () => void,
    onError?: (error: any) => void,
    onClose?: () => void
  ): void {
    this.onConnectionCallback = onConnection;
    this.onErrorCallback = onError;
    this.onCloseCallback = onClose;

    console.log(`[ROS] Connecting to ${url}...`);
    this.ros = new Ros({ url });

    this.ros.on('connection', () => {
      console.log('[ROS] Connected to ROS Bridge WebSocket');
      this.reconnectAttempts = 0;
      this.setupTopics();
      this.onConnectionCallback?.();
    });

    this.ros.on('error', (error) => {
      console.error('[ROS] Connection error:', error);
      this.onErrorCallback?.(error);
      this.attemptReconnect(url);
    });

    this.ros.on('close', () => {
      console.log('[ROS] Connection closed');
      this.onCloseCallback?.();
      this.attemptReconnect(url);
    });
  }

  /**
   * Attempt to reconnect to ROS Bridge
   */
  private attemptReconnect(url: string): void {
    if (this.reconnectInterval) return;
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('[ROS] Max reconnect attempts reached');
      return;
    }

    this.reconnectAttempts++;
    console.log(`[ROS] Reconnecting... (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

    this.reconnectInterval = setTimeout(() => {
      this.reconnectInterval = null;
      this.connect(url, this.onConnectionCallback, this.onErrorCallback, this.onCloseCallback);
    }, this.reconnectDelay);
  }

  /**
   * Setup ROS topics
   */
  private setupTopics(): void {
    if (!this.ros) return;

    // cmd_vel topic
    this.cmdVelTopic = new Topic<Twist>({
      ros: this.ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    // Map topic
    this.mapTopic = new Topic<OccupancyGrid>({
      ros: this.ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid',
    });

    // TF topic
    this.tfTopic = new Topic<TFMessage>({
      ros: this.ros,
      name: '/tf',
      messageType: 'tf2_msgs/TFMessage',
    });

    // Battery topic
    this.batteryTopic = new Topic<BatteryState>({
      ros: this.ros,
      name: '/battery_state',
      messageType: 'sensor_msgs/BatteryState',
    });

    // Scan topic
    this.scanTopic = new Topic<LaserScan>({
      ros: this.ros,
      name: '/scan',
      messageType: 'sensor_msgs/LaserScan',
    });

    console.log('[ROS] Topics initialized');
  }

  /**
   * Disconnect from ROS Bridge
   */
  disconnect(): void {
    if (this.reconnectInterval) {
      clearTimeout(this.reconnectInterval);
      this.reconnectInterval = null;
    }

    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }

    // Unsubscribe from all topics
    this.cmdVelTopic = null;
    this.mapTopic = null;
    this.tfTopic = null;
    this.batteryTopic = null;
    this.scanTopic = null;

    console.log('[ROS] Disconnected');
  }

  /**
   * Publish velocity command
   */
  publishVelocity(linear: number, angular: number): void {
    if (!this.cmdVelTopic) {
      console.warn('[ROS] cmd_vel topic not initialized');
      return;
    }

    const twist: Twist = {
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular },
    };

    this.cmdVelTopic.publish(twist as any);
  }

  /**
   * Publish initial pose
   */
  publishInitialPose(x: number, y: number, theta: number): void {
    if (!this.ros) {
      console.warn('[ROS] ROS not connected');
      return;
    }

    const initialPoseTopic = new Topic<PoseWithCovarianceStamped>({
      ros: this.ros,
      name: '/initialpose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    });

    const poseMsg: PoseWithCovarianceStamped = {
      header: {
        frame_id: 'map',
        stamp: { sec: 0, nanosec: 0 },
      },
      pose: {
        pose: {
          position: { x, y, z: 0.0 },
          orientation: {
            x: 0,
            y: 0,
            z: Math.sin(theta / 2),
            w: Math.cos(theta / 2),
          },
        },
        covariance: [
          0.25, 0, 0, 0, 0, 0,
          0, 0.25, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0.06853892326654787,
        ],
      },
    };

    initialPoseTopic.publish(poseMsg as any);
    console.log(`[ROS] Initial pose set to (${x}, ${y}, ${theta})`);
  }

  /**
   * Send navigation goal
   */
  // Navigation action via rosbridge is unreliable on ROS2; goal/cancel handled via API server instead.

  /**
   * Get list of topics
   */
  getTopics(callback: (topics: string[], types: string[]) => void, errorCallback?: (error: any) => void): void {
    if (!this.ros) {
      errorCallback?.('ROS not connected');
      return;
    }

    this.ros.getTopics(
      (result) => {
        callback(result.topics, result.types);
      },
      errorCallback
    );
  }

  /**
   * Get list of nodes
   */
  getNodes(callback: (nodes: string[]) => void, errorCallback?: (error: any) => void): void {
    if (!this.ros) {
      errorCallback?.('ROS not connected');
      return;
    }

    this.ros.getNodes(callback, errorCallback);
  }

  /**
   * Call SLAM service to save map
   */
  saveMap(mapName: string, callback?: (success: boolean) => void): void {
    if (!this.ros) {
      console.warn('[ROS] ROS not connected');
      callback?.(false);
      return;
    }

    const saveMapService = new Service({
      ros: this.ros,
      name: '/slam_toolbox/serialize_map',
      serviceType: 'slam_toolbox/srv/SerializePoseGraph',
    });

    const request = {
      filename: `/app/maps/${mapName}`,
    };

    saveMapService.callService(
      request,
      (result: any) => {
        const success = result.result === true;
        console.log(`[ROS] Save map ${success ? 'success' : 'failed'}: ${mapName}`);
        callback?.(success);
      },
      (error) => {
        console.error('[ROS] Save map error:', error);
        callback?.(false);
      }
    );
  }

  /**
   * Call SLAM service to load map
   */
  loadMap(mapName: string, callback?: (success: boolean) => void): void {
    if (!this.ros) {
      console.warn('[ROS] ROS not connected');
      callback?.(false);
      return;
    }

    const loadMapService = new Service({
      ros: this.ros,
      name: '/slam_toolbox/deserialize_map',
      serviceType: 'slam_toolbox/srv/DeserializePoseGraph',
    });

    const request = {
      filename: `/app/maps/${mapName}`,
      match_type: 2, // Start at dock
    };

    loadMapService.callService(
      request,
      (result: any) => {
        const success = result.result === true;
        console.log(`[ROS] Load map ${success ? 'success' : 'failed'}: ${mapName}`);
        callback?.(success);
      },
      (error) => {
        console.error('[ROS] Load map error:', error);
        callback?.(false);
      }
    );
  }

  /**
   * Call SLAM service to clear map
   */
  clearMap(callback?: (success: boolean) => void): void {
    if (!this.ros) {
      console.warn('[ROS] ROS not connected');
      callback?.(false);
      return;
    }

    const clearService = new Service({
      ros: this.ros,
      name: '/slam_toolbox/clear_changes',
      serviceType: 'slam_toolbox/srv/Clear',
    });

    const request = {};

    clearService.callService(
      request,
      () => {
        console.log('[ROS] Map cleared');
        callback?.(true);
      },
      (error) => {
        console.error('[ROS] Clear map error:', error);
        callback?.(false);
      }
    );
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.ros !== null;
  }
}

// Export singleton instance
export const rosService = new ROSService();
