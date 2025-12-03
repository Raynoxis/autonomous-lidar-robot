import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { useRobotStore } from '../../store';

export const MapViewer = () => {
  const mapRef = useRef<L.Map | null>(null);
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapLayerRef = useRef<L.ImageOverlay | null>(null);
  const robotMarkerRef = useRef<L.Marker | null>(null);
  const goalMarkerRef = useRef<L.Marker | null>(null);

  const [clickMode, setClickMode] = useState<'navigation' | 'initialpose' | null>(null);

  const {
    mapData,
    mapBounds,
    robotPose,
    batteryVoltage,
    sendNavigationGoal,
    setInitialPose,
    addLog,
  } = useRobotStore();

  // Initialize map
  useEffect(() => {
    if (!mapContainerRef.current || mapRef.current) return;

    const map = L.map(mapContainerRef.current, {
      crs: L.CRS.Simple,
      minZoom: -5,
      maxZoom: 10,
      zoomControl: true,
      attributionControl: false,
    }).setView([0, 0], 0);

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

  // Render map data
  useEffect(() => {
    if (!mapData || !mapRef.current) return;

    const map = mapRef.current;

    // Create canvas for map
    const canvas = document.createElement('canvas');
    canvas.width = mapData.width;
    canvas.height = mapData.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

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

    if (!mapBounds) return;

    // Convert to Leaflet coordinates
    const sw = worldToLeaflet(mapBounds.minX, mapBounds.minY);
    const ne = worldToLeaflet(mapBounds.maxX, mapBounds.maxY);
    const bounds: L.LatLngBoundsExpression = [
      [sw.lat, sw.lng],
      [ne.lat, ne.lng],
    ];

    // Remove old layer and add new one
    if (mapLayerRef.current) {
      map.removeLayer(mapLayerRef.current);
    }

    mapLayerRef.current = L.imageOverlay(imageUrl, bounds).addTo(map);

    // Fit bounds only on first map load
    if (!mapLayerRef.current) {
      map.fitBounds(bounds);
    }
  }, [mapData, mapBounds]);

  // Update robot marker
  useEffect(() => {
    if (!mapRef.current || !mapData) return;

    const map = mapRef.current;
    const latlng = worldToLeaflet(robotPose.x, robotPose.y);

    if (!robotMarkerRef.current) {
      const robotIcon = L.divIcon({
        className: 'robot-marker',
        html: '<div style="background: #2563eb; width: 20px; height: 20px; border-radius: 50%; border: 3px solid white;"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10],
      });

      robotMarkerRef.current = L.marker(latlng, { icon: robotIcon }).addTo(map);
    } else {
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

  return (
    <div className="relative h-full w-full">
      <div ref={mapContainerRef} className="h-full w-full bg-dark-card" />

      {/* Map Overlay */}
      <div className="absolute top-4 left-4 z-[400] bg-dark-bg/95 p-4 rounded-lg border border-dark-border max-w-xs pointer-events-none">
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
          </div>
        </div>
      </div>

      {/* Map Controls */}
      <div className="absolute top-4 right-4 z-[400] flex flex-col gap-2">
        <button
          onClick={() => {
            setClickMode(clickMode === 'navigation' ? null : 'navigation');
            addLog(clickMode === 'navigation' ? 'Navigation mode off' : 'Click map to navigate');
          }}
          className={`px-4 py-2 rounded-lg font-bold transition-all ${
            clickMode === 'navigation'
              ? 'bg-success text-white'
              : 'bg-dark-bg/95 text-text-light border border-dark-border hover:bg-dark-card'
          }`}
        >
          📍 Navigate
        </button>
        <button
          onClick={() => {
            setClickMode(clickMode === 'initialpose' ? null : 'initialpose');
            addLog(clickMode === 'initialpose' ? 'Initial pose mode off' : 'Click map to set initial pose');
          }}
          className={`px-4 py-2 rounded-lg font-bold transition-all ${
            clickMode === 'initialpose'
              ? 'bg-warning text-white'
              : 'bg-dark-bg/95 text-text-light border border-dark-border hover:bg-dark-card'
          }`}
        >
          📌 Set Pose
        </button>
        <button
          onClick={handleClearGoal}
          className="px-4 py-2 rounded-lg font-bold bg-dark-bg/95 text-text-light border border-dark-border hover:bg-dark-card transition-all"
        >
          🗑️ Clear Goal
        </button>
      </div>

      {/* Click Mode Indicator */}
      {clickMode && (
        <div className="absolute bottom-4 left-1/2 transform -translate-x-1/2 z-[400] bg-warning/90 text-white px-6 py-3 rounded-lg font-bold animate-pulse">
          {clickMode === 'navigation' ? '📍 Click on map to navigate' : '📌 Click on map to set initial pose'}
        </div>
      )}
    </div>
  );
};
