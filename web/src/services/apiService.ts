/**
 * API Service for ROS2 Control API (ros_api.py)
 * Handles HTTP requests to start/stop exploration and check status
 */

export interface APIResponse<T = any> {
  success: boolean;
  message?: string;
  data?: T;
}

export interface ExplorationStartResponse {
  pid: number;
}

export interface ProcessStatusResponse {
  running: boolean;
  pid?: number;
}

export interface NodeListResponse {
  nodes: string[];
}

export interface NavGoalResponse {
  pid?: number;
}

export class APIService {
  private baseUrl: string;

  constructor(baseUrl?: string) {
    // Default to same-origin /api to leverage serve.py proxy (8082 -> 8083)
    this.baseUrl =
      baseUrl ||
      (typeof window !== 'undefined' ? `${window.location.origin}/api` : 'http://192.168.0.10:8082/api');
  }

  /**
   * Generic POST request
   */
  private async post<T>(action: string, data?: any): Promise<APIResponse<T>> {
    try {
      const response = await fetch(this.baseUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ action, ...data }),
      });

      const result = await response.json();
      return result;
    } catch (error) {
      console.error(`[API] POST ${action} failed:`, error);
      return {
        success: false,
        message: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Generic GET request
   */
  private async get<T>(endpoint: string): Promise<APIResponse<T>> {
    try {
      const response = await fetch(`${this.baseUrl}${endpoint}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      const result = await response.json();
      return result;
    } catch (error) {
      console.error(`[API] GET ${endpoint} failed:`, error);
      return {
        success: false,
        message: error instanceof Error ? error.message : 'Unknown error',
      };
    }
  }

  /**
   * Start exploration
   */
  async startExploration(): Promise<APIResponse<ExplorationStartResponse>> {
    console.log('[API] Starting exploration...');
    return this.post<ExplorationStartResponse>('start_explore');
  }

  /**
   * Stop exploration
   */
  async stopExploration(): Promise<APIResponse> {
    console.log('[API] Stopping exploration...');
    return this.post('stop_explore');
  }

  /**
   * Check process status
   */
  async checkProcess(processName: string): Promise<APIResponse<ProcessStatusResponse>> {
    return this.post<ProcessStatusResponse>('check_process', { process_name: processName });
  }

  /**
   * Get list of ROS2 nodes
   */
  async listNodes(): Promise<APIResponse<NodeListResponse>> {
    return this.post<NodeListResponse>('list_nodes');
  }

  /**
   * Get overall status
   */
  async getStatus(): Promise<APIResponse<{ running_processes: Record<string, boolean> }>> {
    return this.get<{ running_processes: Record<string, boolean> }>('/status');
  }

  /**
   * Get list of managed processes
   */
  async getProcesses(): Promise<APIResponse<{ processes: string[] }>> {
    return this.get<{ processes: string[] }>('/processes');
  }

  /**
   * Send a navigation goal (handled server-side to avoid rosbridge action quirks)
   */
  async sendNavigationGoal(x: number, y: number, theta: number = 0): Promise<APIResponse<NavGoalResponse>> {
    return this.post<NavGoalResponse>('nav_goal', { x, y, theta });
  }

  /**
   * Cancel navigation goal
   */
  async cancelNavigationGoal(): Promise<APIResponse> {
    return this.post('nav_cancel');
  }

  /**
   * Save map on the server
   */
  async saveMap(mapName: string): Promise<APIResponse> {
    return this.post('save_map', { map_name: mapName });
  }

  /**
   * Load map on the server
   */
  async loadMap(mapName: string): Promise<APIResponse> {
    return this.post('load_map', { map_name: mapName });
  }

  /**
   * Clear map (slam_toolbox)
   */
  async clearMap(): Promise<APIResponse> {
    return this.post('clear_map');
  }

  /**
   * Exploration status (process alive)
   */
  async getExplorationStatus(): Promise<APIResponse<{ running: boolean; pid?: number; finished?: boolean }>> {
    return this.get<{ running: boolean; pid?: number; finished?: boolean }>('/explore/status');
  }

  /**
   * Navigation goal status (process + log parsing)
   */
  async getNavigationStatus(): Promise<
    APIResponse<{ running: boolean; pid?: number; status?: string; finished?: boolean }>
  > {
    return this.get<{ running: boolean; pid?: number; status?: string; finished?: boolean }>(
      '/nav/status'
    );
  }

  /**
   * Set base URL
   */
  setBaseUrl(url: string): void {
    this.baseUrl = url;
  }
}

// Export singleton instance
export const apiService = new APIService();
