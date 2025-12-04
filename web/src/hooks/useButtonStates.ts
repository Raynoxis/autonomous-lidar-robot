import { useRobotStore } from '../store';
import type { SystemState } from '../types';

/**
 * Hook to determine button states based on system state
 * Returns an object with boolean flags for each button group
 */
export const useButtonStates = () => {
  const { connected, systemState } = useRobotStore();

  // Helper to check if system is operational
  const isSystemOperational = (state: SystemState): boolean => {
    return (
      state === 'robot_ready' ||
      state === 'exploration_available' ||
      state === 'exploring' ||
      state === 'navigating'
    );
  };

  // Helper to check if Nav2 is ready
  const isNav2Ready = (state: SystemState): boolean => {
    return (
      state === 'container_ready' ||
      state === 'robot_ready' ||
      state === 'exploration_available' ||
      state === 'exploring' ||
      state === 'navigating'
    );
  };

  return {
    // Connection buttons
    canConnect: !connected && systemState === 'initial',
    canDisconnect: connected,

    // Emergency stop - always available when connected
    canEmergencyStop: connected,

    // Joystick - needs robot to be ready
    canUseJoystick: connected && isSystemOperational(systemState),

    // SLAM commands - needs ROS bridge at minimum
    canSaveMap: connected && isNav2Ready(systemState),
    canLoadMap: connected && isNav2Ready(systemState),
    canClearMap: connected && isNav2Ready(systemState),

    // Navigation commands - needs full system
    canNavigate: connected && isSystemOperational(systemState),
    canCancelGoal: connected && isNav2Ready(systemState),
    canSetHome: connected && isSystemOperational(systemState),

    // Exploration - Start available when NOT exploring, Stop available when exploring
    canStartExplore:
      connected &&
      (systemState === 'exploration_available' || systemState === 'robot_ready'),
    canStopExplore: connected && systemState === 'exploring',

    // Informational flags
    isFullyOperational: connected && isSystemOperational(systemState),
    isStartingUp:
      connected &&
      (systemState === 'connecting_ws' ||
        systemState === 'ws_connected' ||
        systemState === 'container_ready'),
    hasError: systemState.includes('error') || systemState === 'robot_lost',
    systemState,
  };
};
