import React from 'react';
import { useRobotStore } from '../store';
import type { SystemState } from '../types';

export const Header: React.FC = () => {
  const { systemState } = useRobotStore();

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

  const getStatusColor = (state: SystemState): string => {
    if (state === 'initial') return 'text-text-gray border-dark-border';
    if (state.includes('error') || state === 'robot_lost') return 'text-danger border-danger';
    if (state === 'connecting_ws') return 'text-warning border-warning animate-pulse';
    return 'text-success border-success';
  };

  return (
    <div className="bg-dark-bg px-8 py-4 border-b-2 border-dark-border flex justify-between items-center gap-4 flex-wrap">
      <div className="text-2xl font-bold text-text-light">
        MakersPet Mini - Control Center
      </div>
      <div
        className={`flex-1 text-center font-medium px-4 py-2 bg-dark-card rounded-lg border ${getStatusColor(
          systemState
        )}`}
      >
        {getStatusMessage(systemState)}
      </div>
    </div>
  );
};
