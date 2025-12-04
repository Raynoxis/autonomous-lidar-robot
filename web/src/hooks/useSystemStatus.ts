import { useRobotStore } from '../store';
import type { SystemStatusItem } from '../types';

/**
 * Hook qui retourne l'état dynamique du système
 * Adapte l'affichage selon l'état courant de la machine à états
 */
export const useSystemStatus = (): SystemStatusItem[] => {
  const { systemState, nodes, topics, scanDataReceived } = useRobotStore();

  // Selon l'état, on affiche différents items
  switch (systemState) {
    case 'initial':
      return [
        { label: 'WebSocket', status: 'disabled', detail: 'Non connecté' },
        { label: 'Nav2', status: 'disabled', detail: 'En attente' },
        { label: 'Robot', status: 'disabled', detail: 'En attente' },
      ];

    case 'connecting_ws':
      return [
        { label: 'WebSocket', status: 'checking', detail: 'Connexion...' },
        { label: 'Nav2', status: 'pending', detail: 'En attente' },
        { label: 'Robot', status: 'pending', detail: 'En attente' },
      ];

    case 'ws_connected':
      const nav2Count = [
        '/slam_toolbox',
        '/bt_navigator',
        '/controller_server',
        '/planner_server',
      ].filter((n) => nodes[n]).length;

      return [
        { label: 'WebSocket', status: 'success', detail: 'Connecté' },
        {
          label: 'Nav2',
          status: nav2Count >= 3 ? 'checking' : 'checking',
          detail: `${nav2Count}/4 nodes`,
        },
        { label: 'Robot', status: 'pending', detail: 'En attente' },
      ];

    case 'container_ready':
      const nav2Ready = [
        '/slam_toolbox',
        '/bt_navigator',
        '/controller_server',
        '/planner_server',
      ].every((n) => nodes[n]);

      const robotNodes = ['/kaiaai_telemetry_node', '/robot_state_publisher'].filter(
        (n) => nodes[n]
      ).length;

      return [
        { label: 'WebSocket', status: 'success', detail: 'Connecté' },
        { label: 'Nav2', status: nav2Ready ? 'success' : 'checking', detail: '4/4 nodes' },
        {
          label: 'Robot',
          status: 'checking',
          detail: scanDataReceived ? 'Scan détecté' : `${robotNodes}/2 nodes`,
        },
      ];

    case 'robot_ready':
    case 'exploring':
    case 'navigating':
      const nav2Active = [
        '/slam_toolbox',
        '/bt_navigator',
        '/controller_server',
        '/planner_server',
      ].every((n) => nodes[n]);

      const robotActive = scanDataReceived && nodes['/kaiaai_telemetry_node'];

      const items: SystemStatusItem[] = [
        { label: 'WebSocket', status: 'success', detail: 'Connecté' },
        { label: 'Nav2', status: nav2Active ? 'success' : 'error', detail: '4/4 nodes' },
        {
          label: 'Robot',
          status: robotActive ? 'success' : 'error',
          detail: topics['/scan'] ? 'LiDAR actif' : 'Données reçues',
        },
      ];

      // Ajouter status spécifique selon mode
      if (systemState === 'exploring') {
        items.push({
          label: 'Exploration',
          status: nodes['/explore_node'] ? 'success' : 'error',
          detail: nodes['/explore_node'] ? 'Active' : 'Node manquant',
        });
      } else if (systemState === 'navigating') {
        items.push({
          label: 'Navigation',
          status: 'checking',
          detail: 'Vers objectif...',
        });
      }

      return items;

    case 'ws_error':
      return [
        { label: 'WebSocket', status: 'error', detail: 'Connexion perdue' },
        { label: 'Nav2', status: 'disabled', detail: 'Indisponible' },
        { label: 'Robot', status: 'disabled', detail: 'Indisponible' },
      ];

    case 'container_error':
      const nav2Err = [
        '/slam_toolbox',
        '/bt_navigator',
        '/controller_server',
        '/planner_server',
      ].filter((n) => nodes[n]).length;

      return [
        { label: 'WebSocket', status: 'success', detail: 'Connecté' },
        { label: 'Nav2', status: 'error', detail: `${nav2Err}/4 nodes` },
        { label: 'Robot', status: 'disabled', detail: 'En attente Nav2' },
      ];

    case 'robot_lost':
      return [
        { label: 'WebSocket', status: 'success', detail: 'Connecté' },
        { label: 'Nav2', status: 'success', detail: 'Opérationnel' },
        {
          label: 'Robot',
          status: 'error',
          detail: scanDataReceived ? 'Reconnexion...' : 'Données perdues',
        },
      ];

    default:
      return [];
  }
};
