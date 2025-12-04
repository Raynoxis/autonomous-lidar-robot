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
      state === 'exploring' ||
      state === 'navigating'
    );
  };

  // Helper to check if Nav2 is ready
  const isNav2Ready = (state: SystemState): boolean => {
    return (
      state === 'container_ready' ||
      state === 'robot_ready' ||
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

    // Joystick - needs robot to be ready, disabled during autonomous operations
    canUseJoystick: connected && systemState === 'robot_ready',

    // SLAM commands - needs Nav2 at minimum
    canSaveMap: connected && isNav2Ready(systemState),
    canLoadMap: connected && isNav2Ready(systemState),
    canClearMap: connected && isNav2Ready(systemState),

    // Navigation commands - needs full system
    canNavigate: connected && systemState === 'robot_ready',
    canCancelGoal: connected && (systemState === 'navigating' || systemState === 'exploring'),
    canSetHome: connected && isSystemOperational(systemState),

    // Exploration - Start only from robot_ready, Stop only when exploring
    canStartExplore: connected && systemState === 'robot_ready',
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
