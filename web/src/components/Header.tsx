import React from 'react';
import { useRobotStore } from '../store';
import type { SystemState } from '../types';

export const Header: React.FC = () => {
  const { systemState, nodes, topics } = useRobotStore();

  const nav2Nodes = ['/bt_navigator', '/controller_server', '/planner_server', '/slam_toolbox'];
  const nav2Count = nav2Nodes.filter(n => nodes[n]).length;
  const missingNav2 = nav2Nodes.filter(n => !nodes[n]);

  const criticalTopics = ['/scan', '/map', '/cmd_vel'];
  const topicCount = criticalTopics.filter(t => topics[t]).length;
  const missingTopics = criticalTopics.filter(t => !topics[t]);

  const getStatusMessage = (state: SystemState): string => {
    const messages: Record<SystemState, string> = {
      initial: 'Système prêt - Cliquez sur Connect',
      connecting_ws: 'Connexion au RosBridge WebSocket...',
      ws_connected:
        nav2Count === 0
          ? 'RosBridge OK - Allume le robot (Nav2 en attente)'
          : 'RosBridge OK - Vérification Nav2...',
      container_ready: 'Nav2 opérationnel - En attente du robot...',
      robot_ready: 'Système opérationnel - Prêt à naviguer',
      exploring: 'Exploration autonome en cours...',
      navigating: 'Navigation vers objectif...',
      ws_error: 'Erreur RosBridge - Connexion perdue',
      container_error: 'Erreur conteneur - Nodes Nav2 manquants',
      robot_lost: 'Robot déconnecté - Reconnexion...',
    };
    return messages[state];
  };

  const getStatusDetails = (state: SystemState): string => {
    const robotNodes = ['/kaiaai_telemetry_node', '/robot_state_publisher'];
    const robotCount = robotNodes.filter(n => nodes[n]).length;

    switch (state) {
      case 'ws_connected':
        return nav2Count === 0
          ? 'Allume le robot pour lancer Nav2 (0/4 actifs)'
          : `Détection Nav2... (${nav2Count}/4 actifs)`;

      case 'container_ready':
        return `Nav2: ${nav2Count}/4${
          missingNav2.length ? ` (manquants: ${missingNav2.join(', ')})` : ' OK'
        } • Attente robot (${robotCount}/2 nodes)`;

      case 'robot_ready':
        return `Robot OK • Nav2 ${nav2Count}/4${
          missingNav2.length ? ` (manquants: ${missingNav2.join(', ')})` : ''
        } • Topics ${topicCount}/3${
          missingTopics.length ? ` (manquants: ${missingTopics.join(', ')})` : ''
        }`;

      case 'exploring':
        const exploreActive = nodes['/explore_node'] ? '✓' : '✗';
        return `Explore ${exploreActive} • Nav2: ${nav2Count}/4 • Frontières actives`;

      case 'navigating':
        return `Nav2 actif • Déplacement vers objectif`;

      case 'robot_lost':
        const scanOK = topics['/scan'] ? '✓' : '✗';
        return `Scan ${scanOK} • Reconnexion en cours...`;

      default:
        return '';
    }
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
      robot_ready: 100,
      exploring: 100,
      navigating: 100,
      ws_error: 0,
      container_error: 40,
      robot_lost: 60,
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
