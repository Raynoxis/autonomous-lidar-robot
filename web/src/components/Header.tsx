import React from 'react';
import { useRobotStore } from '../store';
import type { SystemState } from '../types';

export const Header: React.FC = () => {
  const { systemState, nodes, topics } = useRobotStore();

  const getStatusMessage = (state: SystemState): string => {
    const messages: Record<SystemState, string> = {
      initial: 'Système initialisé - Cliquez sur Connect',
      connecting_ws: 'Connexion au ROS Bridge WebSocket...',
      ws_connected: 'ROS Bridge OK - Vérification conteneur...',
      container_ready: 'Conteneur opérationnel - En attente du robot...',
      robot_ready: 'Robot connecté - Tous systèmes opérationnels',
      exploration_available: 'Prêt pour exploration autonome',
      exploring: 'Exploration autonome en cours...',
      navigating: 'Navigation vers objectif...',
      ws_error: 'Erreur ROS Bridge - Vérifiez le serveur',
      container_error: 'Erreur conteneur - Nodes manquants',
      robot_lost: 'Robot déconnecté - Vérifiez alimentation',
      exploration_error: 'Erreur exploration - Node arrêté',
    };
    return messages[state];
  };

  const getStatusDetails = (state: SystemState): string => {
    // Compter les nodes critiques actifs
    const nav2Nodes = ['/bt_navigator', '/controller_server', '/planner_server', '/slam_toolbox'];
    const nav2Count = nav2Nodes.filter(n => nodes[n]).length;
    const robotNodes = ['/kaiaai_telemetry_node', '/robot_state_publisher'];
    const robotCount = robotNodes.filter(n => nodes[n]).length;

    const criticalTopics = ['/scan', '/map', '/cmd_vel'];
    const topicCount = criticalTopics.filter(t => topics[t]).length;

    if (state === 'ws_connected') {
      return `Détection des nodes Nav2... (${nav2Count}/4 actifs)`;
    }
    if (state === 'container_ready') {
      return `Nav2 opérationnel (${nav2Count}/4) • Attente robot (${robotCount}/2 nodes)`;
    }
    if (state === 'robot_ready') {
      return `Robot OK • Topics: ${topicCount}/3 actifs • Navigation prête`;
    }
    if (state === 'exploration_available' || state === 'exploring') {
      const exploreActive = nodes['/explore_node'] ? '✓' : '✗';
      return `Explore ${exploreActive} • Nav2: ${nav2Count}/4 • Robot: ${robotCount}/2`;
    }
    return '';
  };

  const getStatusColor = (state: SystemState): string => {
    if (state === 'initial') return 'text-text-gray border-dark-border';
    if (state.includes('error') || state === 'robot_lost') return 'text-danger border-danger';
    if (state === 'connecting_ws' || state === 'ws_connected' || state === 'container_ready')
      return 'text-warning border-warning animate-pulse';
    return 'text-success border-success';
  };

  const getProgressPercentage = (state: SystemState): number => {
    const progress: Record<SystemState, number> = {
      initial: 0,
      connecting_ws: 20,
      ws_connected: 40,
      container_ready: 60,
      robot_ready: 80,
      exploration_available: 100,
      exploring: 100,
      navigating: 100,
      ws_error: 0,
      container_error: 40,
      robot_lost: 60,
      exploration_error: 80,
    };
    return progress[state] || 0;
  };

  const showProgressBar = !systemState.includes('error') && systemState !== 'initial';
  const progressPercentage = getProgressPercentage(systemState);
  const statusDetails = getStatusDetails(systemState);

  return (
    <div className="bg-dark-bg px-8 py-4 border-b-2 border-dark-border">
      <div className="flex justify-between items-center gap-4 flex-wrap mb-2">
        <div className="text-2xl font-bold text-text-light">
          MakersPet Mini - Control Center
        </div>
        <div
          className={`flex-1 text-center font-medium px-4 py-2 bg-dark-card rounded-lg border ${getStatusColor(
            systemState
          )}`}
        >
          <div className="font-bold">{getStatusMessage(systemState)}</div>
          {statusDetails && (
            <div className="text-xs mt-1 opacity-80">{statusDetails}</div>
          )}
        </div>
      </div>

      {/* Progress bar */}
      {showProgressBar && (
        <div className="relative h-1 bg-dark-card rounded-full overflow-hidden">
          <div
            className={`absolute top-0 left-0 h-full transition-all duration-500 ${
              systemState.includes('error') || systemState === 'robot_lost'
                ? 'bg-danger'
                : progressPercentage === 100
                ? 'bg-success'
                : 'bg-warning'
            }`}
            style={{ width: `${progressPercentage}%` }}
          />
        </div>
      )}
    </div>
  );
};
