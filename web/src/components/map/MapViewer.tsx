import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { useRobotStore } from '../../store';
import { useButtonStates } from '../../hooks';
import type { SystemState } from '../../types';

export const MapViewer = () => {
  const mapRef = useRef<L.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapLayerRef = useRef<L.ImageOverlay | null>(null);
  const robotMarkerRef = useRef<L.Marker | null>(null);
  const goalMarkerRef = useRef<L.Marker | null>(null);
  const isFirstMapLoadRef = useRef<boolean>(true);
  const lastMapUpdateRef = useRef<number>(0);

  const [clickMode, setClickMode] = useState<'navigation' | 'initialpose' | null>(null);

  const {
    mapData,
    mapBounds,
    robotPose,
    batteryVoltage,
    scanRange,
    sendNavigationGoal,
    setInitialPose,
    cancelNavigation,
    setHomePosition,
    saveMap,
    loadMap,
    clearMap,
    addLog,
    systemState,
    nodes,
    topics,
  } = useRobotStore();
  const buttonStates = useButtonStates();

  // Initialize map
  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return;

    const map = L.map(mapContainerRef.current, {
      crs: L.CRS.Simple,
      minZoom: -3,
      maxZoom: 16,
      zoomControl: true,
      attributionControl: false,
    }).setView([0, 0], 2);

    // Add neutral gray background
    const bounds: L.LatLngBoundsExpression = [[-10, -10], [10, 10]];
    L.imageOverlay(
      'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mM0NDT8DwADHgEz8mY6qgAAAABJRU5ErkJggg==',
      bounds
    ).addTo(map);
    map.fitBounds(bounds);

    mapRef.current = map;

    return () => {
      if (mapRef.current) {
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, []);

  // Handle map clicks
  useEffect(() => {
    const map = mapRef.current;
    if (!map) return;

    const handleClick = (e: L.LeafletMouseEvent) => {
      if (!mapData) return;

      const worldCoords = leafletToWorld(e.latlng);

      if (clickMode === 'navigation') {
        sendNavigationGoal(worldCoords.x, worldCoords.y);

        // Add goal marker
        if (goalMarkerRef.current) {
          map.removeLayer(goalMarkerRef.current);
        }

        const goalIcon = L.divIcon({
          className: 'goal-marker',
          html: '<div style="background: #ef4444; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
          iconSize: [20, 20],
          iconAnchor: [10, 10],
        });

        goalMarkerRef.current = L.marker(e.latlng, { icon: goalIcon }).addTo(map);
        setClickMode(null);
      } else if (clickMode === 'initialpose') {
        setInitialPose(worldCoords.x, worldCoords.y, 0);

        // Add temporary marker
        const poseMarker = L.circleMarker(e.latlng, {
          radius: 8,
          color: '#10b981',
          fillColor: '#10b981',
          fillOpacity: 0.5,
        }).addTo(map);

        setTimeout(() => {
          map.removeLayer(poseMarker);
        }, 3000);

        setClickMode(null);
      }
    };

    map.on('click', handleClick);

    return () => {
      map.off('click', handleClick);
    };
  }, [mapData, clickMode, sendNavigationGoal, setInitialPose]);

  // Render map data with throttling
  useEffect(() => {
    if (!mapData || !mapRef.current || !mapBounds) return;

    // Throttle map updates to max 1 per second
    const now = Date.now();
    const timeSinceLastUpdate = now - lastMapUpdateRef.current;
    const MIN_UPDATE_INTERVAL = 200; // 0.2s for smoother visual updates

    if (timeSinceLastUpdate < MIN_UPDATE_INTERVAL && !isFirstMapLoadRef.current) {
      // Skip this update, too soon after last one
      return;
    }

    lastMapUpdateRef.current = now;
    const map = mapRef.current;

    // Create canvas for map
    const canvas = document.createElement('canvas');
    canvas.width = mapData.width;
    canvas.height = mapData.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas first
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw occupancy grid
    const imageData = ctx.createImageData(mapData.width, mapData.height);

    for (let y = 0; y < mapData.height; y++) {
      for (let x = 0; x < mapData.width; x++) {
        const index = y * mapData.width + x;
        const value = mapData.data[index];

        const pixelIndex = ((mapData.height - 1 - y) * mapData.width + x) * 4;

        if (value === -1) {
          // Unknown - gray
          imageData.data[pixelIndex] = 128;
          imageData.data[pixelIndex + 1] = 128;
          imageData.data[pixelIndex + 2] = 128;
          imageData.data[pixelIndex + 3] = 255;
        } else if (value === 0) {
          // Free space - white
          imageData.data[pixelIndex] = 255;
          imageData.data[pixelIndex + 1] = 255;
          imageData.data[pixelIndex + 2] = 255;
          imageData.data[pixelIndex + 3] = 255;
        } else {
          // Occupied - black
          imageData.data[pixelIndex] = 0;
          imageData.data[pixelIndex + 1] = 0;
          imageData.data[pixelIndex + 2] = 0;
          imageData.data[pixelIndex + 3] = 255;
        }
      }
    }

    ctx.putImageData(imageData, 0, 0);

    // Convert canvas to image URL
    const imageUrl = canvas.toDataURL();

    // Convert to Leaflet coordinates
    const sw = worldToLeaflet(mapBounds.minX, mapBounds.minY);
    const ne = worldToLeaflet(mapBounds.maxX, mapBounds.maxY);
    const bounds: L.LatLngBoundsExpression = [
      [sw.lat, sw.lng],
      [ne.lat, ne.lng],
    ];

    // PROPERLY remove old layer before adding new one
    if (mapLayerRef.current) {
      try {
        map.removeLayer(mapLayerRef.current);
        mapLayerRef.current = null;
      } catch (e) {
        // Layer might already be removed
        mapLayerRef.current = null;
      }
    }

    // Add new layer
    mapLayerRef.current = L.imageOverlay(imageUrl, bounds).addTo(map);

    // Fit bounds only on FIRST map load
    if (isFirstMapLoadRef.current) {
      map.fitBounds(bounds, { padding: [12, 12] });
      map.zoomIn(1);
      isFirstMapLoadRef.current = false;
    }
  }, [mapData, mapBounds]);

  // Update robot marker - responds to robotPose changes in real-time
  useEffect(() => {
    if (!mapRef.current) return;

    const map = mapRef.current;
    const latlng = worldToLeaflet(robotPose.x, robotPose.y);

    // Create marker on first render if we have map data
    if (!robotMarkerRef.current && mapData) {
      const robotIcon = L.divIcon({
        className: 'robot-marker',
        html: '<div style="background: #2563eb; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10],
      });

      robotMarkerRef.current = L.marker(latlng, { icon: robotIcon }).addTo(map);
    }
    // Update marker position in real-time
    else if (robotMarkerRef.current) {
      robotMarkerRef.current.setLatLng(latlng);
    }
  }, [robotPose, mapData]);

  // Helper functions
  const worldToLeaflet = (x: number, y: number): L.LatLng => {
    return L.latLng(y, x);
  };

  const leafletToWorld = (latlng: L.LatLng): { x: number; y: number } => {
    return { x: latlng.lng, y: latlng.lat };
  };

  const handleClearGoal = () => {
    if (goalMarkerRef.current && mapRef.current) {
      mapRef.current.removeLayer(goalMarkerRef.current);
      goalMarkerRef.current = null;
    }
  };

  const handleSaveMap = () => {
    const name = prompt('Nom de la carte :', 'my_map');
    if (name) saveMap(name);
  };

  const handleLoadMap = () => {
    const name = prompt('Nom de la carte à charger :', 'my_map');
    if (name) loadMap(name);
  };

  const handleClearMap = () => {
    if (confirm('Effacer la carte actuelle ?')) clearMap();
  };

  const nav2Nodes = ['/bt_navigator', '/controller_server', '/planner_server', '/slam_toolbox'];
  const nav2Count = nav2Nodes.filter((n) => nodes[n]).length;
  const missingNav2 = nav2Nodes.filter((n) => !nodes[n]);
  const criticalTopics = ['/scan', '/map', '/cmd_vel'];
  const topicCount = criticalTopics.filter((t) => topics[t]).length;
  const missingTopics = criticalTopics.filter((t) => !topics[t]);

  const statusMessage = () => {
    const messages: Record<SystemState, string> = {
      initial: 'Système prêt - Connectez-vous',
      connecting_ws: 'Connexion au RosBridge...',
      ws_connected: nav2Count === 0 ? 'RosBridge OK - Attente Nav2' : 'RosBridge OK - Vérification Nav2',
      container_ready: 'Nav2 opérationnel - Attente robot',
      robot_ready: 'Robot prêt - Navigation OK',
      exploring: 'Exploration autonome en cours',
      navigating: 'Navigation vers objectif',
      ws_error: 'Erreur RosBridge',
      container_error: 'Erreur Nav2/stack',
      robot_lost: 'Robot déconnecté',
    };
    return messages[systemState] || systemState;
  };

  const statusDetails = () => {
    switch (systemState) {
      case 'robot_ready':
        return `Nav2 ${nav2Count}/4${missingNav2.length ? ` (manquants: ${missingNav2.join(', ')})` : ''} • Topics ${topicCount}/3${missingTopics.length ? ` (manquants: ${missingTopics.join(', ')})` : ''}`;
      case 'exploring':
        return `Explore ${nodes['/explore_node'] ? '✓' : '✗'} • Nav2 ${nav2Count}/4`;
      case 'navigating':
        return `Nav2 ${nav2Count}/4 • Commande en cours`;
      case 'ws_connected':
        return nav2Count === 0 ? 'Allume le robot pour lancer Nav2' : `Nav2 détecté (${nav2Count}/4)`;
      case 'container_ready':
        return `Nav2 ${nav2Count}/4 • En attente des données robot`;
      default:
        return '';
    }
  };

  const statusClass = () => {
    if (systemState.includes('error') || systemState === 'robot_lost') return 'border-danger text-danger';
    if (systemState === 'connecting_ws' || systemState === 'ws_connected' || systemState === 'container_ready')
      return 'border-warning text-warning';
    return 'border-success text-success';
  };

  return (
    <div className="relative h-full w-full">
      <div ref={mapContainerRef} className="h-full w-full bg-dark-card min-h-[320px] md:min-h-0" />

      {/* Map Overlay */}
      <div className="absolute bottom-4 left-4 z-[400] bg-dark-bg/95 p-4 rounded-lg border border-dark-border max-w-xs pointer-events-none">
        <div className="pointer-events-auto">
          <div className="font-bold mb-2">Robot Position</div>
          <div className="space-y-1 text-sm">
            <div className="flex justify-between">
              <span className="text-text-gray">X:</span>
              <span>{robotPose.x.toFixed(2)} m</span>
            </div>
            <div className="flex justify-between">
              <span className="text-text-gray">Y:</span>
              <span>{robotPose.y.toFixed(2)} m</span>
            </div>
            <div className="flex justify-between">
              <span className="text-text-gray">Theta:</span>
              <span>{robotPose.theta.toFixed(2)} rad</span>
            </div>
            <div className="flex justify-between">
              <span className="text-text-gray">Battery:</span>
              <span className={batteryVoltage && batteryVoltage > 11.5 ? 'text-success' : 'text-warning'}>
                {batteryVoltage ? `${batteryVoltage.toFixed(2)}V` : '--'}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-text-gray">Scan range:</span>
              <span>
                {scanRange ? `${scanRange.min.toFixed(2)}m - ${scanRange.max.toFixed(2)}m` : '--'}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Map Controls */}
      <div className="absolute top-4 right-4 z-[400] flex flex-col gap-2 bg-dark-bg/95 p-3 rounded-lg border border-dark-border text-sm">
        <div className="space-y-2">
          <div className="text-xs uppercase tracking-wide text-text-gray font-semibold">Navigation</div>
          <div className="flex flex-col gap-2">
            <button
              onClick={() => {
                setClickMode(clickMode === 'navigation' ? null : 'navigation');
                addLog(clickMode === 'navigation' ? 'Navigation mode off' : 'Click map to navigate');
              }}
              disabled={!buttonStates.canNavigate}
              className={`px-3 py-2 rounded-lg font-semibold transition-all ${
                clickMode === 'navigation'
                  ? 'bg-success text-white'
                  : 'bg-dark-card text-text-light border border-dark-border hover:bg-dark-card/80'
              } ${!buttonStates.canNavigate ? 'opacity-50 cursor-not-allowed' : ''}`}
            >
              📍 Naviguer
            </button>
            <button
              onClick={() => {
                setClickMode(clickMode === 'initialpose' ? null : 'initialpose');
                addLog(clickMode === 'initialpose' ? 'Initial pose mode off' : 'Click map to set initial pose');
              }}
              disabled={!buttonStates.canSetHome}
              className={`px-3 py-2 rounded-lg font-semibold transition-all ${
                clickMode === 'initialpose'
                  ? 'bg-warning text-white'
                  : 'bg-dark-card text-text-light border border-dark-border hover:bg-dark-card/80'
              } ${!buttonStates.canSetHome ? 'opacity-50 cursor-not-allowed' : ''}`}
            >
              📌 Pose initiale
            </button>
            <button
              onClick={() => {
                cancelNavigation();
                handleClearGoal();
              }}
              disabled={!buttonStates.canCancelGoal}
              className="px-3 py-2 rounded-lg font-semibold bg-dark-card text-text-light border border-dark-border hover:bg-dark-card/80 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
            >
              ❌ Stop Navigation
            </button>
            <button
              onClick={() => setHomePosition()}
              disabled={!buttonStates.canSetHome}
              className="px-3 py-2 rounded-lg font-semibold bg-dark-card text-text-light border border-dark-border hover:bg-dark-card/80 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
            >
              🏠 Définir Home
            </button>
            <button
              onClick={handleClearGoal}
              className="px-3 py-2 rounded-lg font-semibold bg-dark-card text-text-light border border-dark-border hover:bg-dark-card/80 transition-all"
            >
              🗑️ Effacer le goal
            </button>
          </div>
        </div>
      </div>

      {/* SLAM Controls */}
      <div className="absolute bottom-4 right-4 z-[400] bg-dark-bg/95 p-4 rounded-lg border border-dark-border max-w-xs">
        <div className="text-xs uppercase tracking-wide text-text-gray font-bold mb-2">SLAM</div>
        <div className="flex flex-col gap-2">
          <button
            onClick={handleSaveMap}
            disabled={!buttonStates.canSaveMap}
            className="px-4 py-2 rounded-lg font-bold bg-success text-white hover:bg-success/90 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
          >
            💾 Sauver carte
          </button>
          <button
            onClick={handleLoadMap}
            disabled={!buttonStates.canLoadMap}
            className="px-4 py-2 rounded-lg font-bold bg-primary text-white hover:bg-primary/90 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
          >
            📂 Charger carte
          </button>
          <button
            onClick={handleClearMap}
            disabled={!buttonStates.canClearMap}
            className="px-4 py-2 rounded-lg font-bold bg-danger text-white hover:bg-danger/90 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
          >
            🗑️ Effacer carte
          </button>
        </div>
      </div>

      {/* Click Mode Indicator */}
      {clickMode && (
        <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2 z-[400] bg-warning/90 text-white px-6 py-3 rounded-lg font-bold animate-pulse">
          {clickMode === 'navigation' ? '📍 Click on map to navigate' : '📌 Click on map to set initial pose'}
        </div>
      )}

      {/* Center Status */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 z-[350] pointer-events-none">
        <div className={`bg-dark-bg/95 px-4 py-2 rounded-lg border ${statusClass()} text-center min-w-[260px]`}>
          <div className="font-semibold text-text-light">{statusMessage()}</div>
          {statusDetails() && (
            <div className="text-xs text-text-gray mt-1">{statusDetails()}</div>
          )}
        </div>
      </div>

    </div>
  );
};
