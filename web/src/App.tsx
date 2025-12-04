import { useEffect } from 'react';
import { Header } from './components/Header';
import { Button, Panel } from './components/ui';
import { MapViewer } from './components/map';
import { VirtualJoystick } from './components/joystick';
import { SystemStatus } from './components/SystemStatus';
import { useRobotStore } from './store';
import { useButtonStates } from './hooks';

function App() {
  const {
    connected,
    rosUrl,
    setRosUrl,
    connect,
    disconnect,
    commandLog,
    robotPose,
    batteryVoltage,
    scanRange,
    emergencyStop,
    startExploration,
    stopExploration,
    saveMap,
    loadMap,
    clearMap,
    cancelNavigation,
    setHomePosition,
  } = useRobotStore();

  const buttonStates = useButtonStates();

  useEffect(() => {
    // Cleanup on unmount
    return () => {
      if (connected) {
        disconnect();
      }
    };
  }, []);

  return (
    <div className="min-h-screen flex flex-col bg-dark-darker">
      <Header />

      <div className="flex-1 grid grid-cols-1 lg:grid-cols-[350px_1fr_350px] gap-4 p-4 overflow-hidden">
        {/* Left Panel - Connection & Control */}
        <Panel title="Connection & Control" className="overflow-hidden">
          <div className="space-y-4">
            {/* Connection */}
            <div>
              <label className="block text-sm text-text-gray mb-2">ROS Bridge URL</label>
              <input
                type="text"
                value={rosUrl}
                onChange={(e) => setRosUrl(e.target.value)}
                disabled={connected}
                className="w-full px-3 py-2 bg-dark-card border border-dark-border rounded-lg text-text-light focus:outline-none focus:border-primary disabled:opacity-50"
              />
            </div>

            <div className="flex gap-2">
              <Button
                variant="success"
                fullWidth
                disabled={!buttonStates.canConnect}
                onClick={connect}
              >
                Connect
              </Button>
              <Button
                variant="danger"
                fullWidth
                disabled={!buttonStates.canDisconnect}
                onClick={disconnect}
              >
                Disconnect
              </Button>
            </div>

            {/* Emergency Stop */}
            <Button
              variant="danger"
              fullWidth
              disabled={!buttonStates.canEmergencyStop}
              onClick={emergencyStop}
            >
              ⚠ EMERGENCY STOP
            </Button>

            <hr className="border-dark-border" />

            {/* Virtual Joystick */}
            <VirtualJoystick />

            <hr className="border-dark-border" />

            {/* Robot Telemetry */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm">Robot Telemetry</h3>
              <div className="bg-dark-card rounded-lg p-3 space-y-2 text-sm">
                <div className="flex justify-between">
                  <span className="text-text-gray">Position X:</span>
                  <span>{robotPose.x.toFixed(2)} m</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-text-gray">Position Y:</span>
                  <span>{robotPose.y.toFixed(2)} m</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-text-gray">Orientation:</span>
                  <span>{((robotPose.theta * 180) / Math.PI).toFixed(1)}°</span>
                </div>
                <div className="flex justify-between">
                  <span className="text-text-gray">Battery:</span>
                  <span className={batteryVoltage && batteryVoltage > 11.5 ? 'text-success' : 'text-warning'}>
                    {batteryVoltage ? `${batteryVoltage.toFixed(2)}V` : '--'}
                  </span>
                </div>
                <div className="flex justify-between">
                  <span className="text-text-gray">Scan Range:</span>
                  <span>
                    {scanRange ? `${scanRange.min.toFixed(2)}m - ${scanRange.max.toFixed(2)}m` : '--'}
                  </span>
                </div>
              </div>
            </div>

            <hr className="border-dark-border" />

            {/* Dynamic System Status */}
            <SystemStatus />
          </div>
        </Panel>

        {/* Center Panel - Map */}
        <Panel title="Interactive Map" className="overflow-hidden">
          <div className="h-full">
            <MapViewer />
          </div>
        </Panel>

        {/* Right Panel - Commands */}
        <Panel title="Navigation Commands" className="overflow-hidden">
          <div className="space-y-4">
            {/* SLAM Commands */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">SLAM Control</h3>
              <div className="space-y-2">
                <Button
                  variant="success"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canSaveMap}
                  onClick={() => {
                    const name = prompt('Map name:', 'my_map');
                    if (name) saveMap(name);
                  }}
                >
                  💾 Save Map
                </Button>
                <Button
                  variant="primary"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canLoadMap}
                  onClick={() => {
                    const name = prompt('Map name to load:', 'my_map');
                    if (name) loadMap(name);
                  }}
                >
                  📂 Load Map
                </Button>
                <Button
                  variant="danger"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canClearMap}
                  onClick={() => {
                    if (confirm('Clear current map?')) clearMap();
                  }}
                >
                  🗑️ Clear Map
                </Button>
              </div>
            </div>

            <hr className="border-dark-border" />

            {/* Navigation Commands */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">Navigation</h3>
              <div className="space-y-2">
                <Button
                  variant="danger"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canCancelGoal}
                  onClick={cancelNavigation}
                >
                  ❌ Cancel Goal
                </Button>
                <Button
                  variant="secondary"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canSetHome}
                  onClick={setHomePosition}
                >
                  🏠 Set Home
                </Button>
              </div>
            </div>

            <hr className="border-dark-border" />

            {/* Exploration */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">Exploration</h3>
              <div className="space-y-2">
                <Button
                  variant="primary"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canStartExplore}
                  onClick={startExploration}
                >
                  🔍 Start Exploration
                </Button>
                <Button
                  variant="danger"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canStopExplore}
                  onClick={stopExploration}
                >
                  ⏹️ Stop Exploration
                </Button>
              </div>
            </div>

            <hr className="border-dark-border" />

            {/* Command Log */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">Command Log</h3>
              <div className="bg-dark-card rounded-lg p-2 max-h-[400px] overflow-y-auto font-mono text-xs space-y-1">
                {commandLog.length === 0 ? (
                  <div className="text-text-gray text-center py-4">No commands yet</div>
                ) : (
                  commandLog.map((entry, index) => (
                    <div key={index} className="text-text-gray border-b border-dark-border pb-1 last:border-b-0">
                      <span className="text-text-light font-bold">[{entry.timestamp}]</span>{' '}
                      {entry.message}
                    </div>
                  ))
                )}
              </div>
            </div>
          </div>
        </Panel>
      </div>
    </div>
  );
}

export default App;
